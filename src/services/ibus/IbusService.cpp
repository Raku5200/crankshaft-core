/*
 * Project: Crankshaft
 * This file is part of Crankshaft project.
 * Copyright (C) 2025 OpenCarDev Team
 *
 *  Crankshaft is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  Crankshaft is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Crankshaft. If not, see <http://www.gnu.org/licenses/>.
 */

#include "IbusService.h"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <QDir>
#include <QFileInfo>
#include <QRandomGenerator>
#include <QSocketNotifier>
#include <QTimer>
#include <cerrno>
#include <cstring>

#include "../config/ConfigService.h"
#include "../logging/Logger.h"
#include "IkeText.h"

namespace crankshaft::ibus {

namespace {
constexpr int kReopenIntervalMs = 5000;
// 20 bytes take ~23 ms at 9600 8E1; FTDI adds up to its latency timer (16 ms
// by default, 1 ms with the image's udev rule).
constexpr int kEchoTimeoutMs = 150;
constexpr int kMaxQueue = 32;
// After this many sends with no echo at all, assume the interface does not
// echo and stop retrying.
constexpr int kNoEchoThreshold = 3;

auto boundedInt(const char* key, int fallback, int low, int high) -> int {
  bool ok = false;
  const int value = ConfigService::instance().get(QLatin1String(key), fallback).toInt(&ok);
  return ok ? qBound(low, value, high) : fallback;
}

auto log(const QString& message) -> void {
  Logger::instance().info(QStringLiteral("[IBus] ") + message);
}
auto warn(const QString& message) -> void {
  Logger::instance().warning(QStringLiteral("[IBus] ") + message);
}

// Android Auto key codes (aap_protobuf KeyCode).
constexpr int kKeyDpadCenter = 23;
constexpr int kKeySearch = 84;  // voice assistant
constexpr int kKeyMediaNext = 87;
constexpr int kKeyMediaPrevious = 88;
}  // namespace

IbusService::IbusService(QObject* parent) : QObject(parent) {
  auto& config = ConfigService::instance();
  m_enabled = config.get(QStringLiteral("core.ibus.enabled"), true).toBool();
  m_devicePath = config.get(QStringLiteral("core.ibus.device"), QStringLiteral("auto")).toString();
  m_sourceAddress =
      static_cast<quint8>(boundedInt("core.ibus.source_address", kAddrGt, 0x00, 0xFF));
  m_pollIntervalMs = boundedInt("core.ibus.poll_interval_s", 10, 1, 3600) * 1000;
  m_pdcIntervalMs = boundedInt("core.ibus.pdc.interval_ms", 750, 300, 5000);
  m_parkingSensors = config.get(QStringLiteral("core.ibus.pdc.enabled"), true).toBool();
  // Off by default: a factory nav computer / monitor already asks the cluster,
  // and its answers are read too. The UI setting overrides this.
  m_pollObc = config.get(QStringLiteral("core.ibus.poll_obc"), false).toBool();
  m_idleGapMs = boundedInt("core.ibus.idle_gap_ms", 10, 2, 200);
  m_maxRetries = boundedInt("core.ibus.max_retries", 3, 0, 10);
  m_busAwakeWindowMs = boundedInt("core.ibus.bus_awake_window_s", 30, 2, 600) * 1000;
  m_sendIkeText = config.get(QStringLiteral("core.ibus.ike.send_text"), true).toBool();
  m_inputToAndroidAuto =
      config.get(QStringLiteral("core.ibus.input.to_android_auto"), true).toBool();

  m_keyMap = {
      {QStringLiteral("bmbt_confirm"), kKeyDpadCenter},
      {QStringLiteral("bmbt_next"), kKeyMediaNext},
      {QStringLiteral("bmbt_previous"), kKeyMediaPrevious},
      {QStringLiteral("bmbt_telephone"), kKeySearch},
      {QStringLiteral("mfl_next"), kKeyMediaNext},
      {QStringLiteral("mfl_previous"), kKeyMediaPrevious},
      {QStringLiteral("mfl_telephone"), kKeySearch},
  };
  // core.ibus.input.keys: {"bmbt_menu": 3, "mfl_telephone": -1, ...}
  const QVariantMap overrides = config.get(QStringLiteral("core.ibus.input.keys")).toMap();
  for (auto it = overrides.cbegin(); it != overrides.cend(); ++it) {
    bool ok = false;
    const int code = it.value().toInt(&ok);
    if (ok) {
      m_keyMap.insert(it.key(), code);
    }
  }

  m_reopenTimer = new QTimer(this);
  m_reopenTimer->setInterval(kReopenIntervalMs);
  connect(m_reopenTimer, &QTimer::timeout, this, &IbusService::tryOpen);

  m_txTimer = new QTimer(this);
  m_txTimer->setSingleShot(true);
  connect(m_txTimer, &QTimer::timeout, this, &IbusService::pumpTx);

  m_echoTimer = new QTimer(this);
  m_echoTimer->setSingleShot(true);
  m_echoTimer->setInterval(kEchoTimeoutMs);
  connect(m_echoTimer, &QTimer::timeout, this, &IbusService::onEchoTimeout);

  // A partial frame with no further bytes for 50 ms (~40 byte times) is noise.
  m_staleTimer = new QTimer(this);
  m_staleTimer->setSingleShot(true);
  m_staleTimer->setInterval(50);
  connect(m_staleTimer, &QTimer::timeout, this, [this]() {
    if (m_parser.hasPartialFrame()) {
      m_parser.resyncStale();
      for (const IbusFrame& frame : m_parser.takeFrames()) {
        handleFrame(frame);
      }
      pumpTx();
    }
  });

  m_pollTimer = new QTimer(this);
  connect(m_pollTimer, &QTimer::timeout, this, &IbusService::poll);

  m_pdcTimer = new QTimer(this);
  connect(m_pdcTimer, &QTimer::timeout, this, &IbusService::onParkingTick);

  // Bus awake/asleep changes without any traffic, so re-check it now and then.
  m_statusTimer = new QTimer(this);
  m_statusTimer->setInterval(2000);
  connect(m_statusTimer, &QTimer::timeout, this, [this]() {
    const bool awake = busAwake();
    if (awake != m_wasAwake) {
      m_wasAwake = awake;
      log(awake ? QStringLiteral("Bus awake") : QStringLiteral("Bus asleep"));
      if (!awake) {
        m_unitsRequested = false;
        m_lastIkeLine.clear();
        m_alertShown = false;
        m_pdcTimer->stop();
        updateParkingVisible();
      }
      publishStatus();
    }
  });
}

IbusService::~IbusService() {
  closeSerial(QStringLiteral("shutdown"));
}

void IbusService::start() {
  if (!m_enabled) {
    log(QStringLiteral("Disabled (core.ibus.enabled=false)"));
    return;
  }
  log(QStringLiteral("Starting (device=%1, source=0x%2, OBC polling %3)")
          .arg(m_devicePath)
          .arg(m_sourceAddress, 2, 16, QLatin1Char('0'))
          .arg(m_pollObc ? QStringLiteral("every %1 s").arg(m_pollIntervalMs / 1000)
                         : QStringLiteral("off")));
  m_statusTimer->start();
  tryOpen();
  if (m_fd < 0) {
    m_reopenTimer->start();
  }
}

void IbusService::stop() {
  m_reopenTimer->stop();
  m_statusTimer->stop();
  closeSerial(QStringLiteral("stopped"));
}

auto IbusService::busAwake() const -> bool {
  return m_fd >= 0 && m_lastRx.isValid() && m_lastRx.elapsed() < m_busAwakeWindowMs;
}

auto IbusService::status() const -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("enabled")] = m_enabled;
  map[QStringLiteral("connected")] = m_fd >= 0;
  map[QStringLiteral("device")] = m_openPath;
  map[QStringLiteral("busAwake")] = busAwake();
  map[QStringLiteral("echo")] = m_echoSeen > 0;
  map[QStringLiteral("framesRx")] = static_cast<qulonglong>(m_framesRx);
  map[QStringLiteral("framesTx")] = static_cast<qulonglong>(m_framesTx);
  map[QStringLiteral("txFailures")] = static_cast<qulonglong>(m_txFailures);
  map[QStringLiteral("collisions")] = static_cast<qulonglong>(m_collisions);
  map[QStringLiteral("droppedBytes")] = static_cast<qulonglong>(m_parser.droppedBytes());
  map[QStringLiteral("inputTarget")] = m_inputTarget;
  map[QStringLiteral("pollObc")] = m_pollObc;
  map[QStringLiteral("bmbtInput")] = m_bmbtInput;
  map[QStringLiteral("mflInput")] = m_mflInput;
  map[QStringLiteral("clusterText")] = m_clusterText;
  map[QStringLiteral("clusterTextPaused")] = m_clusterTextPaused;
  map[QStringLiteral("parkingSensors")] = m_parkingSensors;
  map[QStringLiteral("error")] = m_lastError;
  return map;
}

void IbusService::publishStatus() {
  emit statusChanged(status());
}

auto IbusService::resolveDevice() const -> QString {
  if (m_devicePath != QStringLiteral("auto")) {
    return QFileInfo::exists(m_devicePath) ? m_devicePath : QString();
  }
  // The image's udev rule names a Resler/FTDI interface /dev/ibus.
  if (QFileInfo::exists(QStringLiteral("/dev/ibus"))) {
    return QStringLiteral("/dev/ibus");
  }
  const QDir byId(QStringLiteral("/dev/serial/by-id"));
  const QStringList names = byId.entryList(QDir::System | QDir::Files | QDir::NoDotAndDotDot);
  for (const QString& hint : {QStringLiteral("resler"), QStringLiteral("ibus"),
                              QStringLiteral("ftdi"), QStringLiteral("ft232")}) {
    for (const QString& name : names) {
      if (name.contains(hint, Qt::CaseInsensitive)) {
        return byId.absoluteFilePath(name);
      }
    }
  }
  if (QFileInfo::exists(QStringLiteral("/dev/ttyUSB0"))) {
    return QStringLiteral("/dev/ttyUSB0");
  }
  return {};
}

void IbusService::tryOpen() {
  if (m_fd >= 0) {
    m_reopenTimer->stop();
    return;
  }
  const QString path = resolveDevice();
  if (path.isEmpty()) {
    if (m_lastError != QStringLiteral("no interface")) {
      m_lastError = QStringLiteral("no interface");
      log(QStringLiteral("No IBus interface found (looking for /dev/ibus, a Resler/FTDI "
                         "serial device or /dev/ttyUSB0); retrying every %1 s")
              .arg(kReopenIntervalMs / 1000));
      publishStatus();
    }
    return;
  }
  if (openSerial(path)) {
    m_reopenTimer->stop();
  }
}

auto IbusService::openSerial(const QString& path) -> bool {
  const int fd = ::open(path.toLocal8Bit().constData(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) {
    const QString error = QStringLiteral("open %1: %2").arg(path, QString::fromLocal8Bit(strerror(errno)));
    if (error != m_lastError) {
      m_lastError = error;
      warn(error);
      publishStatus();
    }
    return false;
  }

  termios tio{};
  if (tcgetattr(fd, &tio) != 0) {
    warn(QStringLiteral("tcgetattr %1 failed").arg(path));
    ::close(fd);
    return false;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, B9600);
  cfsetospeed(&tio, B9600);
  tio.c_cflag &= ~(CSIZE | CSTOPB | PARODD | CRTSCTS);
  tio.c_cflag |= CS8 | PARENB | CLOCAL | CREAD;
  // Drop bytes with parity errors (noise) instead of passing them on.
  tio.c_iflag |= INPCK | IGNPAR;
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &tio) != 0) {
    warn(QStringLiteral("tcsetattr %1 failed: %2").arg(path, QString::fromLocal8Bit(strerror(errno))));
    ::close(fd);
    return false;
  }
  tcflush(fd, TCIOFLUSH);

  m_fd = fd;
  m_openPath = QFileInfo(path).canonicalFilePath().isEmpty() ? path
                                                             : QFileInfo(path).canonicalFilePath();
  m_lastError.clear();
  m_parser.reset();
  m_notifier = new QSocketNotifier(fd, QSocketNotifier::Read, this);
  connect(m_notifier, &QSocketNotifier::activated, this, &IbusService::onReadable);
  log(QStringLiteral("Opened %1 (%2) at 9600 8E1").arg(path, m_openPath));

  if (m_pollObc) {
    m_pollTimer->start(m_pollIntervalMs);
  }
  publishStatus();
  return true;
}

void IbusService::closeSerial(const QString& why) {
  if (m_fd < 0) {
    return;
  }
  log(QStringLiteral("Closing %1 (%2)").arg(m_openPath, why));
  delete m_notifier;
  m_notifier = nullptr;
  ::close(m_fd);
  m_fd = -1;
  m_openPath.clear();
  m_txQueue.clear();
  m_awaitingEcho = false;
  m_echoTimer->stop();
  m_txTimer->stop();
  m_pollTimer->stop();
  m_lastRx.invalidate();
  m_unitsRequested = false;
  m_lastIkeLine.clear();
  m_alertShown = false;
  m_pdcTimer->stop();
  updateParkingVisible();
  publishStatus();
}

void IbusService::onReadable() {
  char buffer[256];
  for (;;) {
    const ssize_t count = ::read(m_fd, buffer, sizeof(buffer));
    if (count > 0) {
      m_lastRx.start();
      m_parser.feed(QByteArray(buffer, static_cast<int>(count)));
      continue;
    }
    if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    if (errno == EINTR) {
      continue;
    }
    // EIO etc.: the interface was unplugged.
    m_lastError = QStringLiteral("read: %1").arg(QString::fromLocal8Bit(strerror(errno)));
    closeSerial(m_lastError);
    m_reopenTimer->start();
    return;
  }

  const bool wasAwake = m_wasAwake;
  for (const IbusFrame& frame : m_parser.takeFrames()) {
    handleFrame(frame);
  }
  if (m_parser.hasPartialFrame()) {
    m_staleTimer->start();
  }
  if (!wasAwake && busAwake()) {
    m_wasAwake = true;
    log(QStringLiteral("Bus awake"));
    publishStatus();
    // First traffic: ask for units and the current values straight away.
    poll();
  }
}

void IbusService::handleFrame(const IbusFrame& frame) {
  ++m_framesRx;

  if (m_awaitingEcho && !m_txQueue.isEmpty() && frame.raw == m_txQueue.head().frame) {
    m_echoTimer->stop();
    m_awaitingEcho = false;
    ++m_echoSeen;
    ++m_framesTx;
    m_txQueue.dequeue();
    pumpTx();
    return;
  }

  const int ignitionBefore = m_state.ignition;
  const bool reverseBefore = m_state.reverse();
  if (applyClusterFrame(frame, m_state)) {
    emit vehicleStateChanged(m_state.toVariantMap());
    if (m_state.reverse() != reverseBefore) {
      log(m_state.reverse() ? QStringLiteral("Reverse gear") : QStringLiteral("Out of reverse"));
      if (m_state.reverse()) {
        startParking(QStringLiteral("reverse gear"));
      } else {
        m_pdcGaveUp = false;
        updateParkingVisible();
      }
    }
    if (ignitionBefore > 0 && m_state.ignition == 0 && m_clusterTextPaused) {
      m_clusterTextPaused = false;
      log(QStringLiteral("Ignition off: cluster text resumes next drive"));
      publishStatus();
    }
    return;
  }
  if (isPdcActivation(frame)) {
    m_lastPdcActivity.restart();
    startParking(QStringLiteral("PDC switched on"));
    return;
  }
  if (ParkDistances distances; decodePdcDistances(frame, distances)) {
    m_lastPdcAnswer.restart();
    m_pdcUnanswered = 0;
    if (distances.active) {
      m_lastPdcActivity.restart();
    }
    const bool changed = !m_parkingHeard || !(distances == m_parking);
    m_parking = distances;
    m_parkingHeard = true;
    if (distances.active && !m_pdcTimer->isActive()) {
      startParking(QStringLiteral("PDC answer"));  // another unit is polling
    }
    updateParkingVisible();
    if (changed && m_parkingVisible) {
      emit parkingChanged(parkingState());
    }
    return;
  }
  if (frame.source == kAddrIke && frame.command() == kCmdClusterButtons && frame.data.size() >= 2) {
    const quint8 event = static_cast<quint8>(frame.data.at(1));
    log(QStringLiteral("Cluster button 0x%1").arg(event, 2, 16, QLatin1Char('0')));
    if (event == kClusterButtonBc) {
      pauseClusterText(QStringLiteral("BC button"));
    }
    return;
  }
  if (const auto input = decodeInput(frame)) {
    handleInput(*input);
    return;
  }
  logUnknownFrame(frame);
}

auto IbusService::parkingState() const -> QVariantMap {
  QVariantMap map = m_parking.toVariantMap();
  map[QStringLiteral("visible")] = m_parkingVisible;
  map[QStringLiteral("reverse")] = m_state.reverse();
  map[QStringLiteral("heard")] = m_parkingHeard;
  return map;
}

void IbusService::startParking(const QString& why) {
  if (!m_parkingSensors || m_fd < 0 || m_pdcGaveUp) {
    return;
  }
  if (!m_pdcTimer->isActive()) {
    log(QStringLiteral("Parking sensors: %1, asking PDC every %2 ms").arg(why).arg(m_pdcIntervalMs));
    m_pdcUnanswered = 0;
    m_pdcTimer->start(m_pdcIntervalMs);
    onParkingTick();
  }
}

void IbusService::onParkingTick() {
  constexpr int kPdcQuietMs = 3750;  // no PDC activity this long: it switched off
  const bool pdcRecent = m_lastPdcActivity.isValid() && m_lastPdcActivity.elapsed() < kPdcQuietMs;
  if (!m_parkingSensors || m_fd < 0 || !busAwake() || (!m_state.reverse() && !pdcRecent)) {
    m_pdcTimer->stop();
    log(QStringLiteral("Parking sensors: stopped"));
    updateParkingVisible();
    return;
  }
  // The factory nav/monitor may already be asking: then just listen.
  if (m_lastPdcAnswer.isValid() && m_lastPdcAnswer.elapsed() < m_pdcIntervalMs - 150) {
    return;
  }
  if (m_pdcUnanswered >= 4 && !pdcRecent) {
    // Reverse without any PDC module answering: car without PDC.
    log(QStringLiteral("Parking sensors: no answer from a PDC module, not asking again until next reverse"));
    m_pdcGaveUp = true;
    m_pdcTimer->stop();
    updateParkingVisible();
    return;
  }
  for (const Outgoing& out : m_txQueue) {
    if (out.reason == QStringLiteral("PDC request")) {
      return;  // previous one not sent yet
    }
  }
  ++m_pdcUnanswered;
  enqueue(pdcSensorRequestFrame(), QStringLiteral("PDC request"));
}

void IbusService::updateParkingVisible() {
  const bool pdcRecent = m_lastPdcActivity.isValid() && m_lastPdcActivity.elapsed() < 3750;
  const bool visible = m_parkingSensors && m_fd >= 0 && busAwake() && m_parkingHeard &&
                       (m_state.reverse() || (m_parking.active && pdcRecent));
  if (visible == m_parkingVisible) {
    return;
  }
  m_parkingVisible = visible;
  if (!visible) {
    m_parkingHeard = false;
    m_parking = ParkDistances{};
  }
  log(visible ? QStringLiteral("Parking screen on") : QStringLiteral("Parking screen off"));
  emit parkingChanged(parkingState());
}

void IbusService::logUnknownFrame(const IbusFrame& frame) {
  // Once per source/destination/command, so the log shows what else the car
  // sends without flooding it.
  const quint32 kind = (quint32(frame.source) << 16) | (quint32(frame.destination) << 8) |
                       quint32(frame.command() & 0xFF);
  if (m_loggedFrameKinds.size() >= 128 || m_loggedFrameKinds.contains(kind)) {
    return;
  }
  m_loggedFrameKinds.insert(kind);
  log(QStringLiteral("Seen %1").arg(QString::fromLatin1(frame.raw.toHex(' ').toUpper())));
}

void IbusService::pauseClusterText(const QString& why) {
  if (m_clusterTextPaused) {
    return;
  }
  m_clusterTextPaused = true;
  m_lastIkeLine.clear();
  m_pausedClusterContentKey = m_clusterContentKey;
  log(QStringLiteral("%1: cluster shows OBC values, Android Auto text paused until the next song or turn")
          .arg(why));
  if (m_alertShown) {
    m_alertShown = false;
    m_lastAlertLine.clear();
    enqueue(ikeCheckControlClearFrame(), QStringLiteral("IKE alert clear"));
  }
  // Drop text frames not sent yet.
  QQueue<Outgoing> kept;
  for (int i = 0; i < m_txQueue.size(); ++i) {
    const Outgoing& out = m_txQueue.at(i);
    const bool text = out.reason == QStringLiteral("IKE text") ||
                      out.reason == QStringLiteral("IKE turn alert");
    if (!text || (i == 0 && m_awaitingEcho)) {
      kept.enqueue(out);
    }
  }
  m_txQueue = kept;
  publishStatus();
}

void IbusService::resumeClusterText() {
  if (!m_clusterTextPaused) {
    return;
  }
  m_clusterTextPaused = false;
  m_lastIkeLine.clear();
  log(QStringLiteral("Android Auto text on the cluster resumed"));
  publishStatus();
}

auto IbusService::keyCodeFor(const InputEvent& event) const -> int {
  return m_keyMap.value(event.source + QLatin1Char('_') + event.button, -1);
}

void IbusService::handleInput(const InputEvent& event) {
  if ((event.source == QStringLiteral("bmbt") && !m_bmbtInput) ||
      (event.source == QStringLiteral("mfl") && !m_mflInput)) {
    return;  // turned off in the UI: the car's own units handle these buttons
  }
  QVariantMap map = event.toVariantMap();
  map[QStringLiteral("target")] = m_inputTarget;
  emit inputReceived(map);

  if (!m_inputToAndroidAuto || m_inputTarget != QStringLiteral("android-auto")) {
    return;
  }
  if (event.kind == InputEvent::Kind::Dial) {
    if (event.steps != 0) {
      emit androidAutoRotary(event.steps);
    }
    return;
  }
  const int keyCode = keyCodeFor(event);
  if (keyCode < 0) {
    return;
  }
  if (event.state == InputEvent::State::Press) {
    emit androidAutoKey(keyCode, true);
  } else if (event.state == InputEvent::State::Release) {
    emit androidAutoKey(keyCode, false);
  }
}

void IbusService::setSettings(bool pollObc, bool bmbtInput, bool mflInput, bool clusterText,
                              bool parkingSensors) {
  if (pollObc == m_pollObc && bmbtInput == m_bmbtInput && mflInput == m_mflInput &&
      clusterText == m_clusterText && parkingSensors == m_parkingSensors) {
    return;
  }
  if (parkingSensors != m_parkingSensors) {
    log(QStringLiteral("Settings: parking sensors %1").arg(parkingSensors ? "on" : "off"));
    m_parkingSensors = parkingSensors;
    if (!parkingSensors) {
      m_pdcTimer->stop();
    } else if (m_state.reverse()) {
      startParking(QStringLiteral("reverse gear"));
    }
    updateParkingVisible();
  }
  log(QStringLiteral("Settings: OBC polling %1, BMBT input %2, MFL input %3, cluster text %4")
          .arg(pollObc ? "on" : "off", bmbtInput ? "on" : "off", mflInput ? "on" : "off",
               clusterText ? "on" : "off"));
  if (m_clusterText && !clusterText && m_alertShown) {
    m_alertShown = false;
    m_lastAlertLine.clear();
    enqueue(ikeCheckControlClearFrame(), QStringLiteral("IKE alert clear"));
  }
  if (clusterText && !m_clusterText) {
    m_lastIkeLine.clear();
  }
  m_clusterText = clusterText;
  const bool startPolling = pollObc && !m_pollObc;
  m_pollObc = pollObc;
  m_bmbtInput = bmbtInput;
  m_mflInput = mflInput;
  if (!m_pollObc) {
    m_pollTimer->stop();
    // Drop requests not sent yet.
    QQueue<Outgoing> kept;
    for (int i = 0; i < m_txQueue.size(); ++i) {
      const Outgoing& out = m_txQueue.at(i);
      const bool request = out.reason.startsWith(QStringLiteral("OBC request")) ||
                           out.reason == QStringLiteral("units request");
      if (!request || (i == 0 && m_awaitingEcho)) {
        kept.enqueue(out);
      }
    }
    m_txQueue = kept;
  } else if (m_fd >= 0) {
    m_pollTimer->start(m_pollIntervalMs);
    if (startPolling) {
      m_unitsRequested = false;
      poll();
    }
  }
  publishStatus();
}

void IbusService::setInputTarget(const QString& target) {
  const QString normalised =
      target == QStringLiteral("ui") ? QStringLiteral("ui") : QStringLiteral("android-auto");
  if (normalised == m_inputTarget) {
    return;
  }
  m_inputTarget = normalised;
  log(QStringLiteral("Input target -> %1").arg(m_inputTarget));
  publishStatus();
}

void IbusService::enqueue(const QByteArray& frame, const QString& reason) {
  if (m_fd < 0 || frame.isEmpty()) {
    return;
  }
  if (!busAwake()) {
    // Never wake the car: transmitting on a sleeping bus wakes every module.
    return;
  }
  if (m_txQueue.size() >= kMaxQueue) {
    warn(QStringLiteral("Transmit queue full, dropping %1").arg(reason));
    return;
  }
  m_txQueue.enqueue({frame, reason, 0});
  pumpTx();
}

void IbusService::pumpTx() {
  if (m_fd < 0 || m_awaitingEcho || m_txQueue.isEmpty()) {
    return;
  }
  // Wait for a gap: nobody mid-frame and no byte for idle_gap_ms.
  const qint64 quietMs = m_lastRx.isValid() ? m_lastRx.elapsed() : m_idleGapMs;
  if (m_parser.hasPartialFrame() || quietMs < m_idleGapMs) {
    m_txTimer->start(static_cast<int>(qMax<qint64>(2, m_idleGapMs - quietMs)));
    return;
  }

  Outgoing& out = m_txQueue.head();
  ++out.attempts;
  const ssize_t written = ::write(m_fd, out.frame.constData(), static_cast<size_t>(out.frame.size()));
  if (written != out.frame.size()) {
    warn(QStringLiteral("Write failed for %1: %2")
             .arg(out.reason, QString::fromLocal8Bit(strerror(errno))));
    ++m_txFailures;
    m_txQueue.dequeue();
    m_txTimer->start(m_idleGapMs);
    return;
  }
  m_awaitingEcho = true;
  m_echoTimer->start();
}

void IbusService::onEchoTimeout() {
  if (!m_awaitingEcho || m_txQueue.isEmpty()) {
    return;
  }
  m_awaitingEcho = false;
  Outgoing& out = m_txQueue.head();

  if (m_echoSeen == 0) {
    // This interface may simply not echo. Count the frame as sent.
    ++m_echoMissing;
    ++m_framesTx;
    if (m_echoMissing == kNoEchoThreshold) {
      log(QStringLiteral("Interface does not echo sent frames; collision check off"));
    }
    m_txQueue.dequeue();
    pumpTx();
    return;
  }

  // The interface normally echoes, so a missing or garbled echo is a collision.
  ++m_collisions;
  if (out.attempts > m_maxRetries) {
    ++m_txFailures;
    warn(QStringLiteral("Giving up on %1 after %2 attempts").arg(out.reason).arg(out.attempts));
    m_txQueue.dequeue();
    m_txTimer->start(m_idleGapMs);
    return;
  }
  // Random back-off so two senders don't collide again.
  m_txTimer->start(m_idleGapMs + static_cast<int>(QRandomGenerator::global()->bounded(5, 40)));
}

void IbusService::poll() {
  if (!m_pollObc || !busAwake()) {
    return;
  }
  if (!m_unitsRequested) {
    m_unitsRequested = true;
    enqueue(unitsRequestFrame(m_sourceAddress), QStringLiteral("units request"));
  }
  if (m_state.ignition == 0) {
    return;  // key out (KL-30): the cluster display is off
  }
  for (const quint8 property :
       {ObcTime, ObcTemperature, ObcConsumption1, ObcRange, ObcAvgSpeed}) {
    enqueue(obcControlFrame(property, ObcRequestText, m_sourceAddress),
            QStringLiteral("OBC request 0x%1").arg(property, 2, 16, QLatin1Char('0')));
  }
}

void IbusService::sendObcControl(int property, int control) {
  if (property <= 0 || property > 0x3F || control < 0 || control > 0xFF) {
    return;
  }
  log(QStringLiteral("OBC control property=0x%1 control=0x%2")
          .arg(property, 2, 16, QLatin1Char('0'))
          .arg(control, 2, 16, QLatin1Char('0')));
  enqueue(obcControlFrame(static_cast<quint8>(property), static_cast<quint8>(control),
                          m_sourceAddress),
          QStringLiteral("OBC control"));
  // Show the new value straight away.
  if (control != ObcRequestText) {
    enqueue(obcControlFrame(static_cast<quint8>(property), ObcRequestText, m_sourceAddress),
            QStringLiteral("OBC refresh"));
  }
}

void IbusService::setClusterText(const QVariantMap& clusterText) {
  // A scroll step or distance countdown keeps the same key; a new song or a
  // new turn changes it.
  const QString key = clusterText.contains(QStringLiteral("contentKey"))
                          ? clusterText.value(QStringLiteral("contentKey")).toString()
                          : clusterText.value(QStringLiteral("line")).toString();
  m_clusterContentKey = key;
  if (m_clusterTextPaused) {
    if (key.isEmpty() || key == m_pausedClusterContentKey) {
      return;  // still the line the driver switched away from
    }
    m_clusterTextPaused = false;
    m_lastIkeLine.clear();
    log(QStringLiteral("New song or turn: Android Auto text back on the cluster"));
    publishStatus();
  }
  if (!m_sendIkeText || !m_clusterText || m_clusterTextPaused || !busAwake() ||
      m_state.ignition == 0) {
    return;
  }
  const QByteArray line = clusterText.value(QStringLiteral("line")).toString().toLatin1();
  const bool alert = clusterText.value(QStringLiteral("alert")).toBool();

  if (line != m_lastIkeLine) {
    m_lastIkeLine = line;
    if (!line.isEmpty()) {
      enqueue(ikeTelephoneTextFrame(line), QStringLiteral("IKE text"));
    }
  }

  if (alert && !line.isEmpty()) {
    if (!m_alertShown || line != m_lastAlertLine) {
      m_alertShown = true;
      m_lastAlertLine = line;
      enqueue(ikeCheckControlTextFrame(line), QStringLiteral("IKE turn alert"));
    }
  } else if (m_alertShown) {
    m_alertShown = false;
    m_lastAlertLine.clear();
    enqueue(ikeCheckControlClearFrame(), QStringLiteral("IKE alert clear"));
  }
}

}  // namespace crankshaft::ibus
