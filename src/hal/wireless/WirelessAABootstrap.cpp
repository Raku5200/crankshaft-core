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

#include "WirelessAABootstrap.h"

#include <QDBusConnection>
#include <QDBusInterface>
#include <QDBusMessage>
#include <QDBusMetaType>
#include <QDBusPendingCall>
#include <QDBusPendingCallWatcher>
#include <QDBusReply>
#include <QDBusVariant>
#include <QDateTime>
#include <QSocketNotifier>
#include <QTimer>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "../../services/logging/Logger.h"

namespace {

constexpr const char* BLUEZ_SERVICE = "org.bluez";
constexpr const char* BLUEZ_ROOT_PATH = "/org/bluez";
constexpr const char* BLUEZ_ADAPTER_INTERFACE = "org.bluez.Adapter1";
constexpr const char* BLUEZ_DEVICE_INTERFACE = "org.bluez.Device1";
constexpr const char* BLUEZ_PROFILE_MANAGER_INTERFACE = "org.bluez.ProfileManager1";
constexpr const char* BLUEZ_AGENT_MANAGER_INTERFACE = "org.bluez.AgentManager1";
constexpr const char* DBUS_PROPERTIES_INTERFACE = "org.freedesktop.DBus.Properties";
constexpr const char* DBUS_OBJECT_MANAGER_INTERFACE = "org.freedesktop.DBus.ObjectManager";

constexpr const char* AA_WIRELESS_UUID = "4de17a00-52cb-11e6-bdf4-0800200c9a66";
constexpr const char* HSP_HS_UUID = "00001108-0000-1000-8000-00805f9b34fb";
constexpr const char* HSP_AG_UUID = "00001112-0000-1000-8000-00805f9b34fb";

constexpr const char* AA_PROFILE_PATH = "/org/opencardev/crankshaft/aawireless";
constexpr const char* HSP_PROFILE_PATH = "/org/opencardev/crankshaft/hsp_hs";
constexpr const char* AGENT_PATH = "/org/opencardev/crankshaft/agent";

constexpr int MAX_FRAME_PAYLOAD = 4096;

using DBusInterfacePropertyMap = QMap<QString, QVariantMap>;
using DBusManagedObjectMap = QMap<QDBusObjectPath, DBusInterfacePropertyMap>;

void logInfo(const QString& msg) { Logger::instance().info("[WirelessAA] " + msg); }
void logWarn(const QString& msg) { Logger::instance().warning("[WirelessAA] " + msg); }

// --- minimal protobuf writer -------------------------------------------------

void appendVarint(QByteArray& out, quint64 value) {
  while (value >= 0x80) {
    out.append(static_cast<char>((value & 0x7F) | 0x80));
    value >>= 7;
  }
  out.append(static_cast<char>(value));
}

void appendKey(QByteArray& out, quint32 field, quint32 wireType) {
  appendVarint(out, (static_cast<quint64>(field) << 3) | wireType);
}

void appendStringField(QByteArray& out, quint32 field, const QString& value) {
  const QByteArray utf8 = value.toUtf8();
  appendKey(out, field, 2);
  appendVarint(out, static_cast<quint64>(utf8.size()));
  out.append(utf8);
}

void appendVarintField(QByteArray& out, quint32 field, quint64 value) {
  appendKey(out, field, 0);
  appendVarint(out, value);
}

auto readVarint(const QByteArray& in, int* pos, quint64* value) -> bool {
  quint64 result = 0;
  for (int shift = 0; shift < 64 && *pos < in.size(); shift += 7) {
    const auto byte = static_cast<quint8>(in.at((*pos)++));
    result |= static_cast<quint64>(byte & 0x7F) << shift;
    if ((byte & 0x80) == 0) {
      *value = result;
      return true;
    }
  }
  return false;
}

}  // namespace

// =============================================================================
// aawireless wire format
// =============================================================================

namespace aawireless {

auto messageName(quint16 id) -> QString {
  switch (static_cast<MessageId>(id)) {
    case MessageId::WifiStartRequest:
      return QStringLiteral("WifiStartRequest");
    case MessageId::WifiInfoRequest:
      return QStringLiteral("WifiInfoRequest");
    case MessageId::WifiInfoResponse:
      return QStringLiteral("WifiInfoResponse");
    case MessageId::WifiVersionRequest:
      return QStringLiteral("WifiVersionRequest");
    case MessageId::WifiVersionResponse:
      return QStringLiteral("WifiVersionResponse");
    case MessageId::WifiConnectionStatus:
      return QStringLiteral("WifiConnectionStatus");
    case MessageId::WifiStartResponse:
      return QStringLiteral("WifiStartResponse");
  }
  return QStringLiteral("Unknown(%1)").arg(id);
}

auto encodeWifiStartRequest(const QString& ipAddress, quint32 port) -> QByteArray {
  QByteArray out;
  appendStringField(out, 1, ipAddress);
  appendVarintField(out, 2, port);
  return out;
}

auto encodeWifiInfoResponse(const WifiInfo& info) -> QByteArray {
  const bool open = info.password.isEmpty();
  QByteArray out;
  appendStringField(out, 1, info.ssid);
  appendStringField(out, 2, info.password);
  appendStringField(out, 3, info.bssid);
  appendVarintField(out, 4,
                    static_cast<quint32>(open ? SecurityMode::Open : SecurityMode::Wpa2Personal));
  appendVarintField(out, 5, static_cast<quint32>(AccessPointType::Dynamic));
  return out;
}

auto frameMessage(MessageId id, const QByteArray& payload) -> QByteArray {
  QByteArray out;
  const auto length = static_cast<quint16>(payload.size());
  const auto msgId = static_cast<quint16>(id);
  out.append(static_cast<char>(length >> 8));
  out.append(static_cast<char>(length & 0xFF));
  out.append(static_cast<char>(msgId >> 8));
  out.append(static_cast<char>(msgId & 0xFF));
  out.append(payload);
  return out;
}

auto decodeVarintField(const QByteArray& payload, quint32 fieldNumber, qint64* value) -> bool {
  int pos = 0;
  while (pos < payload.size()) {
    quint64 key = 0;
    if (!readVarint(payload, &pos, &key)) {
      return false;
    }
    const auto field = static_cast<quint32>(key >> 3);
    const auto wireType = static_cast<quint32>(key & 0x7);
    switch (wireType) {
      case 0: {
        quint64 raw = 0;
        if (!readVarint(payload, &pos, &raw)) {
          return false;
        }
        if (field == fieldNumber) {
          // Enums/int32 are sign-extended to 64 bits on the wire.
          *value = static_cast<qint64>(static_cast<qint32>(static_cast<quint32>(raw)));
          return true;
        }
        break;
      }
      case 1:
        pos += 8;
        break;
      case 2: {
        quint64 len = 0;
        if (!readVarint(payload, &pos, &len)) {
          return false;
        }
        pos += static_cast<int>(len);
        break;
      }
      case 5:
        pos += 4;
        break;
      default:
        return false;
    }
  }
  return false;
}

auto FrameParser::next(quint16* id, QByteArray* payload) -> bool {
  if (m_buffer.size() < 4) {
    return false;
  }
  const auto* data = reinterpret_cast<const quint8*>(m_buffer.constData());
  const int length = (data[0] << 8) | data[1];
  if (m_buffer.size() < 4 + length) {
    return false;
  }
  *id = static_cast<quint16>((data[2] << 8) | data[3]);
  *payload = m_buffer.mid(4, length);
  m_buffer.remove(0, 4 + length);
  return true;
}

}  // namespace aawireless

// =============================================================================
// BlueZ Profile1 / Agent1 objects
// =============================================================================

WirelessAABluezProfile::WirelessAABluezProfile(WirelessAABootstrap* owner, bool isAndroidAuto)
    : QObject(owner), m_owner(owner), m_isAndroidAuto(isAndroidAuto) {}

void WirelessAABluezProfile::Release() {
  logInfo(QStringLiteral("%1 profile released by BlueZ")
              .arg(m_isAndroidAuto ? QStringLiteral("Android Auto") : QStringLiteral("HSP")));
}

void WirelessAABluezProfile::NewConnection(const QDBusObjectPath& device,
                                           const QDBusUnixFileDescriptor& fd,
                                           const QVariantMap& /*properties*/) {
  m_owner->onNewConnection(m_isAndroidAuto, device, fd);
}

void WirelessAABluezProfile::RequestDisconnection(const QDBusObjectPath& device) {
  m_owner->onRequestDisconnection(m_isAndroidAuto, device);
}

WirelessAABluezAgent::WirelessAABluezAgent(WirelessAABootstrap* owner)
    : QObject(owner), m_owner(owner) {}

auto WirelessAABluezAgent::allowNewPairing(const QDBusObjectPath& device, const char* what)
    -> bool {
  if (m_owner->pairingWindowOpen()) {
    logInfo(QStringLiteral("Accepting %1 from %2 (pairing window open)")
                .arg(QLatin1String(what), device.path()));
    m_owner->markTrusted(device.path());
    return true;
  }
  logWarn(QStringLiteral("Rejecting %1 from %2: pairing window closed")
              .arg(QLatin1String(what), device.path()));
  sendErrorReply(QStringLiteral("org.bluez.Error.Rejected"),
                 QStringLiteral("Pairing window closed"));
  return false;
}

void WirelessAABluezAgent::Release() { logInfo(QStringLiteral("Pairing agent released")); }

QString WirelessAABluezAgent::RequestPinCode(const QDBusObjectPath& device) {
  allowNewPairing(device, "PIN request");
  return QStringLiteral("0000");
}

void WirelessAABluezAgent::DisplayPinCode(const QDBusObjectPath& device, const QString& pincode) {
  logInfo(QStringLiteral("PIN for %1: %2").arg(device.path(), pincode));
}

uint WirelessAABluezAgent::RequestPasskey(const QDBusObjectPath& device) {
  allowNewPairing(device, "passkey request");
  return 0;
}

void WirelessAABluezAgent::DisplayPasskey(const QDBusObjectPath& device, uint passkey,
                                          ushort /*entered*/) {
  logInfo(QStringLiteral("Passkey for %1: %2")
              .arg(device.path())
              .arg(passkey, 6, 10, QLatin1Char('0')));
}

void WirelessAABluezAgent::RequestConfirmation(const QDBusObjectPath& device, uint passkey) {
  // The phone shows the same 6-digit code and asks the user to confirm there.
  logInfo(QStringLiteral("Pairing code for %1: %2")
              .arg(device.path())
              .arg(passkey, 6, 10, QLatin1Char('0')));
  allowNewPairing(device, "pairing confirmation");
}

void WirelessAABluezAgent::RequestAuthorization(const QDBusObjectPath& device) {
  allowNewPairing(device, "pairing authorization");
}

void WirelessAABluezAgent::AuthorizeService(const QDBusObjectPath& device, const QString& uuid) {
  // Already-paired phones may always use our services; anything else needs the window.
  QDBusInterface props(QLatin1String(BLUEZ_SERVICE), device.path(),
                       QLatin1String(DBUS_PROPERTIES_INTERFACE), QDBusConnection::systemBus());
  QDBusReply<QDBusVariant> paired =
      props.call(QStringLiteral("Get"), QLatin1String(BLUEZ_DEVICE_INTERFACE),
                 QStringLiteral("Paired"));
  if (paired.isValid() && paired.value().variant().toBool()) {
    return;
  }
  allowNewPairing(device, qPrintable(QStringLiteral("service %1").arg(uuid)));
}

void WirelessAABluezAgent::Cancel() { logInfo(QStringLiteral("Pairing request cancelled")); }

// =============================================================================
// WirelessAABootstrap
// =============================================================================

WirelessAABootstrap::WirelessAABootstrap(QObject* parent) : QObject(parent) {
  qDBusRegisterMetaType<DBusInterfacePropertyMap>();
  qDBusRegisterMetaType<DBusManagedObjectMap>();
}

WirelessAABootstrap::~WirelessAABootstrap() { stop(); }

auto WirelessAABootstrap::start(WifiInfoProvider provider, const Options& options) -> bool {
  if (m_running) {
    m_provider = std::move(provider);
    return true;
  }

  QDBusConnection bus = QDBusConnection::systemBus();
  if (!bus.isConnected()) {
    logWarn(QStringLiteral("System D-Bus not available; wireless Android Auto disabled"));
    return false;
  }

  m_provider = std::move(provider);
  m_options = options;
  m_adapterPath = findAdapterPath();
  if (m_adapterPath.isEmpty()) {
    logWarn(QStringLiteral("No Bluetooth adapter found; wireless Android Auto disabled"));
    return false;
  }

  setAdapterProperty(QStringLiteral("Powered"), true);
  if (!m_options.adapterAlias.isEmpty()) {
    setAdapterProperty(QStringLiteral("Alias"), m_options.adapterAlias);
  }

  // Android Auto wireless RFCOMM service.
  m_aaProfile = new WirelessAABluezProfile(this, true);
  if (bus.registerObject(QLatin1String(AA_PROFILE_PATH), m_aaProfile,
                         QDBusConnection::ExportAllSlots)) {
    QVariantMap aaOptions;
    aaOptions.insert(QStringLiteral("Name"), QStringLiteral("Android Auto Wireless"));
    aaOptions.insert(QStringLiteral("Role"), QStringLiteral("server"));
    aaOptions.insert(QStringLiteral("Channel"), QVariant::fromValue<quint16>(options.rfcommChannel));
    aaOptions.insert(QStringLiteral("RequireAuthentication"), false);
    aaOptions.insert(QStringLiteral("RequireAuthorization"), false);
    m_aaProfileRegistered =
        registerProfile(QLatin1String(AA_PROFILE_PATH), QLatin1String(AA_WIRELESS_UUID), aaOptions);
  } else {
    logWarn(QStringLiteral("Could not export Android Auto profile object on D-Bus"));
  }

  if (!m_aaProfileRegistered) {
    logWarn(QStringLiteral("Android Auto wireless profile not registered; is the crankshaft user "
                           "allowed to use BlueZ (bluetooth group)?"));
    bus.unregisterObject(QLatin1String(AA_PROFILE_PATH));
    delete m_aaProfile;
    m_aaProfile = nullptr;
    return false;
  }

  // Headset profile: gives paired phones a profile to connect to, which is what
  // makes the phone's Android Auto app look for our RFCOMM service. If audio
  // software (e.g. PipeWire) already owns it, that works just as well.
  m_hspProfile = new WirelessAABluezProfile(this, false);
  if (bus.registerObject(QLatin1String(HSP_PROFILE_PATH), m_hspProfile,
                         QDBusConnection::ExportAllSlots)) {
    QVariantMap hspOptions;
    hspOptions.insert(QStringLiteral("Name"), QStringLiteral("HSP HS"));
    m_hspProfileRegistered =
        registerProfile(QLatin1String(HSP_PROFILE_PATH), QLatin1String(HSP_HS_UUID), hspOptions);
  }
  if (!m_hspProfileRegistered) {
    logInfo(QStringLiteral("HSP headset profile already provided by another service; using it"));
    bus.unregisterObject(QLatin1String(HSP_PROFILE_PATH));
    delete m_hspProfile;
    m_hspProfile = nullptr;
  }

  m_agentRegistered = registerAgent();

  m_running = true;
  openPairingWindow(options.pairingWindowSeconds);

  m_reconnectTimer = new QTimer(this);
  m_reconnectTimer->setInterval(qMax(5, options.reconnectIntervalSeconds) * 1000);
  connect(m_reconnectTimer, &QTimer::timeout, this, &WirelessAABootstrap::reconnectTick);
  m_reconnectTimer->start();
  QTimer::singleShot(1500, this, &WirelessAABootstrap::reconnectTick);

  logInfo(QStringLiteral("Wireless Android Auto ready on %1 (RFCOMM channel %2)")
              .arg(m_adapterPath)
              .arg(options.rfcommChannel));
  return true;
}

void WirelessAABootstrap::stop() {
  if (!m_running) {
    return;
  }
  m_running = false;

  if (m_reconnectTimer) {
    m_reconnectTimer->stop();
    m_reconnectTimer->deleteLater();
    m_reconnectTimer = nullptr;
  }

  closeAllSessions();

  QDBusConnection bus = QDBusConnection::systemBus();
  if (m_aaProfileRegistered) {
    unregisterProfile(QLatin1String(AA_PROFILE_PATH));
    m_aaProfileRegistered = false;
  }
  if (m_hspProfileRegistered) {
    unregisterProfile(QLatin1String(HSP_PROFILE_PATH));
    m_hspProfileRegistered = false;
  }
  bus.unregisterObject(QLatin1String(AA_PROFILE_PATH));
  bus.unregisterObject(QLatin1String(HSP_PROFILE_PATH));
  delete m_aaProfile;
  m_aaProfile = nullptr;
  delete m_hspProfile;
  m_hspProfile = nullptr;

  unregisterAgent();
  m_pairingWindowEndsMs = 0;
  setAdapterProperty(QStringLiteral("Discoverable"), false);

  logInfo(QStringLiteral("Wireless Android Auto stopped"));
}

void WirelessAABootstrap::setProjectionActive(bool active) {
  m_projectionActive = active;
}

void WirelessAABootstrap::openPairingWindow(int seconds) {
  if (seconds <= 0 || m_adapterPath.isEmpty()) {
    m_pairingWindowEndsMs = 0;
    return;
  }
  m_pairingWindowEndsMs = QDateTime::currentMSecsSinceEpoch() + qint64(seconds) * 1000;
  setAdapterProperty(QStringLiteral("Pairable"), true);
  setAdapterProperty(QStringLiteral("DiscoverableTimeout"), static_cast<quint32>(seconds));
  setAdapterProperty(QStringLiteral("Discoverable"), true);
  logInfo(QStringLiteral("Pairing window open for %1 s: pair the phone with this head unit now")
              .arg(seconds));
}

auto WirelessAABootstrap::pairingWindowOpen() const -> bool {
  return m_pairingWindowEndsMs > QDateTime::currentMSecsSinceEpoch();
}

// --- BlueZ plumbing ----------------------------------------------------------

auto WirelessAABootstrap::findAdapterPath() -> QString {
  QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QStringLiteral("/"),
                         QLatin1String(DBUS_OBJECT_MANAGER_INTERFACE),
                         QDBusConnection::systemBus());
  QDBusReply<DBusManagedObjectMap> reply = manager.call(QStringLiteral("GetManagedObjects"));
  if (!reply.isValid()) {
    logWarn(QStringLiteral("BlueZ not reachable: %1").arg(reply.error().message()));
    return {};
  }
  const DBusManagedObjectMap objects = reply.value();
  for (auto it = objects.constBegin(); it != objects.constEnd(); ++it) {
    if (it.value().contains(QLatin1String(BLUEZ_ADAPTER_INTERFACE))) {
      return it.key().path();
    }
  }
  return {};
}

auto WirelessAABootstrap::registerProfile(const QString& objectPath, const QString& uuid,
                                          const QVariantMap& options) -> bool {
  QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QLatin1String(BLUEZ_ROOT_PATH),
                         QLatin1String(BLUEZ_PROFILE_MANAGER_INTERFACE),
                         QDBusConnection::systemBus());
  QDBusMessage reply = manager.call(QStringLiteral("RegisterProfile"),
                                    QVariant::fromValue(QDBusObjectPath(objectPath)), uuid, options);
  if (reply.type() == QDBusMessage::ErrorMessage) {
    logWarn(QStringLiteral("RegisterProfile %1 failed: %2").arg(uuid, reply.errorMessage()));
    return false;
  }
  logInfo(QStringLiteral("Registered Bluetooth profile %1").arg(uuid));
  return true;
}

void WirelessAABootstrap::unregisterProfile(const QString& objectPath) {
  QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QLatin1String(BLUEZ_ROOT_PATH),
                         QLatin1String(BLUEZ_PROFILE_MANAGER_INTERFACE),
                         QDBusConnection::systemBus());
  manager.call(QStringLiteral("UnregisterProfile"), QVariant::fromValue(QDBusObjectPath(objectPath)));
}

auto WirelessAABootstrap::registerAgent() -> bool {
  QDBusConnection bus = QDBusConnection::systemBus();
  m_agent = new WirelessAABluezAgent(this);
  if (!bus.registerObject(QLatin1String(AGENT_PATH), m_agent, QDBusConnection::ExportAllSlots)) {
    delete m_agent;
    m_agent = nullptr;
    return false;
  }
  QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QLatin1String(BLUEZ_ROOT_PATH),
                         QLatin1String(BLUEZ_AGENT_MANAGER_INTERFACE), bus);
  QDBusMessage reply = manager.call(QStringLiteral("RegisterAgent"),
                                    QVariant::fromValue(QDBusObjectPath(AGENT_PATH)),
                                    QStringLiteral("DisplayYesNo"));
  if (reply.type() == QDBusMessage::ErrorMessage) {
    logWarn(QStringLiteral("RegisterAgent failed: %1 (phone-initiated pairing may not work)")
                .arg(reply.errorMessage()));
    bus.unregisterObject(QLatin1String(AGENT_PATH));
    delete m_agent;
    m_agent = nullptr;
    return false;
  }
  manager.call(QStringLiteral("RequestDefaultAgent"),
               QVariant::fromValue(QDBusObjectPath(AGENT_PATH)));
  logInfo(QStringLiteral("Pairing agent registered"));
  return true;
}

void WirelessAABootstrap::unregisterAgent() {
  QDBusConnection bus = QDBusConnection::systemBus();
  if (m_agentRegistered) {
    QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QLatin1String(BLUEZ_ROOT_PATH),
                           QLatin1String(BLUEZ_AGENT_MANAGER_INTERFACE), bus);
    manager.call(QStringLiteral("UnregisterAgent"),
                 QVariant::fromValue(QDBusObjectPath(AGENT_PATH)));
    m_agentRegistered = false;
  }
  bus.unregisterObject(QLatin1String(AGENT_PATH));
  delete m_agent;
  m_agent = nullptr;
}

void WirelessAABootstrap::setAdapterProperty(const QString& name, const QVariant& value) {
  if (m_adapterPath.isEmpty()) {
    return;
  }
  QDBusInterface props(QLatin1String(BLUEZ_SERVICE), m_adapterPath,
                       QLatin1String(DBUS_PROPERTIES_INTERFACE), QDBusConnection::systemBus());
  QDBusMessage reply = props.call(QStringLiteral("Set"), QLatin1String(BLUEZ_ADAPTER_INTERFACE),
                                  name, QVariant::fromValue(QDBusVariant(value)));
  if (reply.type() == QDBusMessage::ErrorMessage) {
    logWarn(QStringLiteral("Setting adapter %1 failed: %2").arg(name, reply.errorMessage()));
  }
}

void WirelessAABootstrap::markTrusted(const QString& devicePath) {
  QDBusInterface props(QLatin1String(BLUEZ_SERVICE), devicePath,
                       QLatin1String(DBUS_PROPERTIES_INTERFACE), QDBusConnection::systemBus());
  props.asyncCall(QStringLiteral("Set"), QLatin1String(BLUEZ_DEVICE_INTERFACE),
                  QStringLiteral("Trusted"), QVariant::fromValue(QDBusVariant(true)));
}

void WirelessAABootstrap::reconnectTick() {
  if (!m_running || m_projectionActive || !m_aaSessions.isEmpty()) {
    return;
  }

  QDBusInterface manager(QLatin1String(BLUEZ_SERVICE), QStringLiteral("/"),
                         QLatin1String(DBUS_OBJECT_MANAGER_INTERFACE),
                         QDBusConnection::systemBus());
  QDBusReply<DBusManagedObjectMap> reply = manager.call(QStringLiteral("GetManagedObjects"));
  if (!reply.isValid()) {
    return;
  }

  QStringList candidates;
  const DBusManagedObjectMap objects = reply.value();
  for (auto it = objects.constBegin(); it != objects.constEnd(); ++it) {
    const QVariantMap device = it.value().value(QLatin1String(BLUEZ_DEVICE_INTERFACE));
    if (device.isEmpty() || !device.value(QStringLiteral("Paired")).toBool()) {
      continue;
    }
    if (device.value(QStringLiteral("Connected")).toBool()) {
      // A paired phone is already connected; its Android Auto app decides
      // whether to start, don't fight it.
      return;
    }
    candidates.append(it.key().path());
  }
  if (candidates.isEmpty()) {
    return;
  }

  // One phone per tick, round-robin, so multiple paired phones don't race.
  const QString devicePath = candidates.at(m_reconnectCursor++ % candidates.size());
  QDBusInterface device(QLatin1String(BLUEZ_SERVICE), devicePath,
                        QLatin1String(BLUEZ_DEVICE_INTERFACE), QDBusConnection::systemBus());
  auto* watcher = new QDBusPendingCallWatcher(
      device.asyncCall(QStringLiteral("ConnectProfile"), QLatin1String(HSP_AG_UUID)), this);
  connect(watcher, &QDBusPendingCallWatcher::finished, this,
          [devicePath](QDBusPendingCallWatcher* w) {
            QDBusPendingReply<> r = *w;
            if (r.isError()) {
              logInfo(QStringLiteral("Phone %1 not reachable yet (%2)")
                          .arg(devicePath, r.error().name()));
            } else {
              logInfo(QStringLiteral("Connected to paired phone %1").arg(devicePath));
            }
            w->deleteLater();
          });
}

// --- RFCOMM handshake --------------------------------------------------------

void WirelessAABootstrap::onNewConnection(bool isAndroidAuto, const QDBusObjectPath& device,
                                          const QDBusUnixFileDescriptor& fd) {
  const QString devicePath = device.path();
  const int ownFd = fd.isValid() ? ::dup(fd.fileDescriptor()) : -1;
  if (ownFd < 0) {
    logWarn(QStringLiteral("NewConnection from %1 without a usable socket").arg(devicePath));
    return;
  }

  if (!isAndroidAuto) {
    // Keep the headset link open; we never send audio on it.
    if (m_hspFds.contains(devicePath)) {
      ::close(m_hspFds.take(devicePath));
    }
    m_hspFds.insert(devicePath, ownFd);
    logInfo(QStringLiteral("Headset profile connected: %1").arg(devicePath));
    return;
  }

  closeSession(devicePath);
  ::fcntl(ownFd, F_SETFL, ::fcntl(ownFd, F_GETFL) | O_NONBLOCK);

  RfcommSession session;
  session.fd = ownFd;
  session.notifier = new QSocketNotifier(ownFd, QSocketNotifier::Read, this);
  connect(session.notifier, &QSocketNotifier::activated, this,
          [this, devicePath]() { onSessionReadable(devicePath); });
  m_aaSessions.insert(devicePath, session);

  logInfo(QStringLiteral("Phone %1 opened the Android Auto wireless channel").arg(devicePath));
  emit handshakeStarted(devicePath);

  const aawireless::WifiInfo info = m_provider ? m_provider() : aawireless::WifiInfo{};
  if (info.ipAddress.isEmpty() || info.port == 0 || info.ssid.isEmpty()) {
    logWarn(QStringLiteral("Hotspot details incomplete (ip=%1 port=%2 ssid=%3); cannot start "
                           "wireless Android Auto")
                .arg(info.ipAddress)
                .arg(info.port)
                .arg(info.ssid));
    emit handshakeFailed(devicePath, QStringLiteral("hotspot not ready"));
    closeSession(devicePath);
    return;
  }

  logInfo(QStringLiteral("Sending WifiStartRequest ip=%1 port=%2").arg(info.ipAddress).arg(info.port));
  sendFrame(devicePath, aawireless::MessageId::WifiStartRequest,
            aawireless::encodeWifiStartRequest(info.ipAddress, info.port));
}

void WirelessAABootstrap::onRequestDisconnection(bool isAndroidAuto, const QDBusObjectPath& device) {
  const QString devicePath = device.path();
  if (isAndroidAuto) {
    closeSession(devicePath);
  } else if (m_hspFds.contains(devicePath)) {
    ::close(m_hspFds.take(devicePath));
  }
}

void WirelessAABootstrap::onSessionReadable(const QString& devicePath) {
  auto it = m_aaSessions.find(devicePath);
  if (it == m_aaSessions.end()) {
    return;
  }

  char buffer[1024];
  for (;;) {
    const ssize_t n = ::read(it->fd, buffer, sizeof(buffer));
    if (n > 0) {
      it->parser.append(QByteArray(buffer, static_cast<int>(n)));
      continue;
    }
    if (n == 0) {
      logInfo(QStringLiteral("Phone %1 closed the wireless channel").arg(devicePath));
      closeSession(devicePath);
      return;
    }
    if (errno == EINTR) {
      continue;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK) {
      break;
    }
    logWarn(QStringLiteral("Read error on wireless channel: %1").arg(QString::fromLocal8Bit(strerror(errno))));
    closeSession(devicePath);
    return;
  }

  quint16 id = 0;
  QByteArray payload;
  while (it != m_aaSessions.end() && it->parser.next(&id, &payload)) {
    handleFrame(devicePath, id, payload);
    it = m_aaSessions.find(devicePath);  // handleFrame may close the session
  }
  if (it != m_aaSessions.end() && it->parser.bufferedBytes() > MAX_FRAME_PAYLOAD + 4) {
    logWarn(QStringLiteral("Oversized frame from phone; dropping channel"));
    closeSession(devicePath);
  }
}

void WirelessAABootstrap::handleFrame(const QString& devicePath, quint16 id,
                                      const QByteArray& payload) {
  using aawireless::MessageId;
  logInfo(QStringLiteral("Phone sent %1 (%2 bytes)").arg(aawireless::messageName(id)).arg(payload.size()));

  switch (static_cast<MessageId>(id)) {
    case MessageId::WifiInfoRequest: {
      const aawireless::WifiInfo info = m_provider ? m_provider() : aawireless::WifiInfo{};
      logInfo(QStringLiteral("Sending WifiInfoResponse ssid=%1 bssid=%2 security=%3")
                  .arg(info.ssid, info.bssid,
                       info.password.isEmpty() ? QStringLiteral("open") : QStringLiteral("WPA2")));
      if (sendFrame(devicePath, MessageId::WifiInfoResponse,
                    aawireless::encodeWifiInfoResponse(info))) {
        auto it = m_aaSessions.find(devicePath);
        if (it != m_aaSessions.end()) {
          it->credentialsSent = true;
        }
        emit credentialsSent(devicePath);
      }
      break;
    }
    case MessageId::WifiStartResponse:
    case MessageId::WifiConnectionStatus: {
      // WifiStartResponse carries status in field 3, WifiConnectionStatus in field 1.
      qint64 status = 0;
      const quint32 field = static_cast<MessageId>(id) == MessageId::WifiStartResponse ? 3 : 1;
      const bool hasStatus = aawireless::decodeVarintField(payload, field, &status);
      if (hasStatus && status != 0) {
        logWarn(QStringLiteral("Phone reported %1 status %2 (e.g. -3 = wrong Wi-Fi password, "
                               "-10 = Wi-Fi off on phone, -2 = channel not usable)")
                    .arg(aawireless::messageName(id))
                    .arg(status));
        emit handshakeFailed(devicePath, QStringLiteral("phone status %1").arg(status));
      } else {
        logInfo(QStringLiteral("Phone reported %1 OK; it should now join the hotspot")
                    .arg(aawireless::messageName(id)));
      }
      emit phoneReportedStatus(devicePath, id, hasStatus ? static_cast<int>(status) : 0);
      break;
    }
    default:
      // WifiVersionRequest and anything unknown: not required by phones; ignore.
      break;
  }
}

auto WirelessAABootstrap::sendFrame(const QString& devicePath, aawireless::MessageId id,
                                    const QByteArray& payload) -> bool {
  auto it = m_aaSessions.find(devicePath);
  if (it == m_aaSessions.end()) {
    return false;
  }
  const QByteArray frame = aawireless::frameMessage(id, payload);
  qsizetype written = 0;
  while (written < frame.size()) {
    const ssize_t n = ::write(it->fd, frame.constData() + written,
                              static_cast<size_t>(frame.size() - written));
    if (n > 0) {
      written += n;
      continue;
    }
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
      pollfd pfd{it->fd, POLLOUT, 0};
      if (::poll(&pfd, 1, 1000) > 0) {
        continue;
      }
    }
    logWarn(QStringLiteral("Failed to send %1: %2")
                .arg(aawireless::messageName(static_cast<quint16>(id)),
                     QString::fromLocal8Bit(strerror(errno))));
    return false;
  }
  return true;
}

void WirelessAABootstrap::closeSession(const QString& devicePath) {
  auto it = m_aaSessions.find(devicePath);
  if (it == m_aaSessions.end()) {
    return;
  }
  if (it->notifier) {
    it->notifier->setEnabled(false);
    it->notifier->deleteLater();
  }
  if (it->fd >= 0) {
    ::close(it->fd);
  }
  m_aaSessions.erase(it);
}

void WirelessAABootstrap::closeAllSessions() {
  const QStringList paths = m_aaSessions.keys();
  for (const QString& path : paths) {
    closeSession(path);
  }
  for (int fd : std::as_const(m_hspFds)) {
    ::close(fd);
  }
  m_hspFds.clear();
}
