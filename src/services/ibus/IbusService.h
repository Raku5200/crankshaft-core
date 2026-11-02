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

#pragma once

#include <QByteArray>
#include <QElapsedTimer>
#include <QHash>
#include <QObject>
#include <QQueue>
#include <QSet>
#include <QString>
#include <QVariantMap>

#include "IbusProtocol.h"

class QSocketNotifier;
class QTimer;

namespace crankshaft::ibus {

/**
 * Talks to the car's IBus through a USB interface such as the Resler
 * I/K-Bus interface (FTDI USB serial + a TH3122-type bus transceiver, so it
 * appears as /dev/ttyUSB*). 9600 baud, 8 data bits, even parity, 1 stop bit.
 *
 * Receives: cluster data (ignition, speed/RPM, temperatures, OBC text values
 * and units) and BMBT/MFL buttons. Sends: OBC text requests (polling, as the
 * navigation computer does), OBC controls from the UI, and the Android Auto
 * line on the cluster display.
 *
 * Sending waits for a quiet bus, then checks that the interface echoes the
 * frame back (the transceiver hears its own transmission); a garbled or
 * missing echo is treated as a collision and the frame is resent. Nothing is
 * sent while the bus is asleep, so the Pi never wakes up the car.
 */
class IbusService : public QObject {
  Q_OBJECT

 public:
  explicit IbusService(QObject* parent = nullptr);
  ~IbusService() override;

  void start();
  void stop();

  [[nodiscard]] auto status() const -> QVariantMap;
  [[nodiscard]] auto vehicleState() const -> QVariantMap { return m_state.toVariantMap(); }
  [[nodiscard]] auto busAwake() const -> bool;
  /// {visible, reverse, active, frontLeft..rearRight (cm, 255 none, -1 unknown)}
  [[nodiscard]] auto parkingState() const -> QVariantMap;

  /// Android Auto key code for an input event, or -1. Exposed for tests.
  [[nodiscard]] auto keyCodeFor(const InputEvent& event) const -> int;

 public slots:
  /// The AA cluster-text map from core (keys "line", "alert").
  void setClusterText(const QVariantMap& clusterText);
  /// UI request (e.g. reset consumption): GT 0x41 to the cluster.
  void sendObcControl(int property, int control);
  /// "android-auto" (default) or "ui": where BMBT/MFL input goes.
  void setInputTarget(const QString& target);
  /// UI settings: poll the cluster for OBC values (off = only listen, e.g. when
  /// the factory nav/monitor already asks), and which car controls to use.
  /// clusterText: show the Android Auto line (song / turn) on the cluster.
  void setSettings(bool pollObc, bool bmbtInput, bool mflInput, bool clusterText = true,
                   bool parkingSensors = true);
  /// Show the Android Auto line again now, after the BC button paused it.
  void resumeClusterText();

 signals:
  void statusChanged(const QVariantMap& status);
  void vehicleStateChanged(const QVariantMap& state);
  /// Every BMBT/MFL button or dial event (for the UI).
  void inputReceived(const QVariantMap& event);
  /// Android Auto key down/up to inject (only while input goes to Android Auto).
  void androidAutoKey(int keyCode, bool down);
  /// Parking sensors changed (see parkingState()).
  void parkingChanged(const QVariantMap& parking);
  /// Android Auto rotary controller steps (+ clockwise).
  void androidAutoRotary(int steps);

 private:
  struct Outgoing {
    QByteArray frame;
    QString reason;
    int attempts = 0;
  };

  void tryOpen();
  auto resolveDevice() const -> QString;
  auto openSerial(const QString& path) -> bool;
  void closeSerial(const QString& why);
  void onReadable();
  void handleFrame(const IbusFrame& frame);
  void handleInput(const InputEvent& event);
  void enqueue(const QByteArray& frame, const QString& reason);
  void pumpTx();
  void onEchoTimeout();
  void poll();
  void publishStatus();
  void pauseClusterText(const QString& why);
  void logUnknownFrame(const IbusFrame& frame);
  void startParking(const QString& why);
  void onParkingTick();
  void updateParkingVisible();

  // config
  bool m_enabled = true;
  QString m_devicePath;  // "auto" or a path
  quint8 m_sourceAddress = kAddrGt;
  int m_pollIntervalMs = 10000;
  bool m_pollObc = false;
  bool m_bmbtInput = true;
  bool m_mflInput = true;
  bool m_clusterText = true;
  bool m_parkingSensors = true;
  int m_pdcIntervalMs = 750;
  int m_idleGapMs = 10;
  int m_maxRetries = 3;
  int m_busAwakeWindowMs = 30000;
  bool m_sendIkeText = true;
  bool m_inputToAndroidAuto = true;
  QHash<QString, int> m_keyMap;

  int m_fd = -1;
  QString m_openPath;
  QSocketNotifier* m_notifier = nullptr;
  QTimer* m_reopenTimer = nullptr;
  QTimer* m_txTimer = nullptr;
  QTimer* m_echoTimer = nullptr;
  QTimer* m_pollTimer = nullptr;
  QTimer* m_statusTimer = nullptr;
  QTimer* m_staleTimer = nullptr;
  QTimer* m_pdcTimer = nullptr;

  IbusFrameParser m_parser;
  VehicleState m_state;
  QElapsedTimer m_lastRx;
  QQueue<Outgoing> m_txQueue;
  bool m_awaitingEcho = false;
  int m_echoSeen = 0;      // echoes matched so far
  int m_echoMissing = 0;   // sends with no echo while echo was never seen
  bool m_unitsRequested = false;
  bool m_wasAwake = false;

  QString m_inputTarget = QStringLiteral("android-auto");
  QByteArray m_lastIkeLine;
  QByteArray m_lastAlertLine;
  bool m_alertShown = false;
  // The driver picked OBC values on the cluster (BC button): leave the
  // display alone until the song or the navigation instruction changes.
  bool m_clusterTextPaused = false;
  QString m_clusterContentKey;        // content of the latest AA line
  QString m_pausedClusterContentKey;  // content showing when BC was pressed
  QSet<quint32> m_loggedFrameKinds;

  // Parking sensors: polled only while reversing or while PDC is switched on.
  ParkDistances m_parking;
  bool m_parkingHeard = false;       // a 0xA0 answer arrived since this started
  bool m_parkingVisible = false;
  QElapsedTimer m_lastPdcActivity;   // activation or "active" answer
  QElapsedTimer m_lastPdcAnswer;     // any 0xA0 answer (ours or another unit's)
  int m_pdcUnanswered = 0;
  bool m_pdcGaveUp = false;          // no PDC module answered this reverse

  quint64 m_framesRx = 0;
  quint64 m_framesTx = 0;
  quint64 m_txFailures = 0;
  quint64 m_collisions = 0;
  QString m_lastError;
};

}  // namespace crankshaft::ibus
