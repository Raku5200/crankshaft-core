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
#include <QDBusContext>
#include <QDBusObjectPath>
#include <QDBusUnixFileDescriptor>
#include <QHash>
#include <QSet>
#include <QObject>
#include <QString>
#include <QVariantMap>
#include <functional>

class QSocketNotifier;
class QTimer;

/**
 * Native (phone-initiated) wireless Android Auto bootstrap over Bluetooth.
 *
 * This is how factory wireless head units start Android Auto - no developer
 * options or "head unit server" on the phone:
 *
 *  1. The head unit exposes the Android Auto wireless RFCOMM service
 *     (UUID 4de17a00-52cb-11e6-bdf4-0800200c9a66, channel 8) plus a dummy HSP
 *     headset profile so paired phones have a profile to connect.
 *  2. When the phone's Android Auto app connects to that service, the head
 *     unit sends WifiStartRequest {ip, port}; the phone asks for credentials
 *     (WifiInfoRequest) and the head unit answers with WifiInfoResponse
 *     {ssid, password, bssid, WPA2, dynamic AP}.
 *  3. The phone joins the head unit's hotspot and opens a TCP connection to
 *     ip:port, where the regular Android Auto protocol takes over.
 *
 * The RFCOMM messages are framed as [u16 BE payload length][u16 BE message id]
 * [protobuf payload]. The payloads are tiny, so they are hand-encoded here
 * (see namespace aawireless) to match what phones expect exactly.
 */
namespace aawireless {

enum class MessageId : quint16 {
  WifiStartRequest = 1,
  WifiInfoRequest = 2,
  WifiInfoResponse = 3,
  WifiVersionRequest = 4,
  WifiVersionResponse = 5,
  WifiConnectionStatus = 6,
  WifiStartResponse = 7,
};

// Values as sent by working implementations (bit-flag style; WPA2 = 8).
enum class SecurityMode : quint32 { Open = 1, Wpa2Personal = 8 };
enum class AccessPointType : quint32 { Static = 0, Dynamic = 1 };

struct WifiInfo {
  QString ssid;
  QString password;
  QString bssid;      // AP MAC, "aa:bb:cc:dd:ee:ff"
  QString ipAddress;  // head unit address on the hotspot
  quint16 port{0};    // TCP port the head unit listens on for Android Auto
};

auto messageName(quint16 id) -> QString;

// Protobuf payloads.
auto encodeWifiStartRequest(const QString& ipAddress, quint32 port) -> QByteArray;
auto encodeWifiInfoResponse(const WifiInfo& info) -> QByteArray;

// Wraps a payload in the RFCOMM frame header.
auto frameMessage(MessageId id, const QByteArray& payload) -> QByteArray;

// Reads a varint field (e.g. a Status enum) from a protobuf payload.
// Returns true and stores the value sign-extended to int if found.
auto decodeVarintField(const QByteArray& payload, quint32 fieldNumber, qint64* value) -> bool;

// Incremental frame parser for a byte stream.
class FrameParser {
 public:
  void append(const QByteArray& data) { m_buffer.append(data); }
  // Extracts the next complete frame; returns false if more data is needed.
  auto next(quint16* id, QByteArray* payload) -> bool;
  auto bufferedBytes() const -> int { return static_cast<int>(m_buffer.size()); }

 private:
  QByteArray m_buffer;
};

}  // namespace aawireless

class WirelessAABootstrap;

/** org.bluez.Profile1 implementation exported on the system bus. */
class WirelessAABluezProfile : public QObject {
  Q_OBJECT
  Q_CLASSINFO("D-Bus Interface", "org.bluez.Profile1")

 public:
  WirelessAABluezProfile(WirelessAABootstrap* owner, bool isAndroidAuto);

 public Q_SLOTS:
  void Release();
  void NewConnection(const QDBusObjectPath& device, const QDBusUnixFileDescriptor& fd,
                     const QVariantMap& properties);
  void RequestDisconnection(const QDBusObjectPath& device);

 private:
  WirelessAABootstrap* m_owner;
  bool m_isAndroidAuto;
};

/** org.bluez.Agent1 implementation: accepts pairing only inside the pairing window. */
class WirelessAABluezAgent : public QObject, protected QDBusContext {
  Q_OBJECT
  Q_CLASSINFO("D-Bus Interface", "org.bluez.Agent1")

 public:
  explicit WirelessAABluezAgent(WirelessAABootstrap* owner);

 public Q_SLOTS:
  void Release();
  QString RequestPinCode(const QDBusObjectPath& device);
  void DisplayPinCode(const QDBusObjectPath& device, const QString& pincode);
  uint RequestPasskey(const QDBusObjectPath& device);
  void DisplayPasskey(const QDBusObjectPath& device, uint passkey, ushort entered);
  void RequestConfirmation(const QDBusObjectPath& device, uint passkey);
  void RequestAuthorization(const QDBusObjectPath& device);
  void AuthorizeService(const QDBusObjectPath& device, const QString& uuid);
  void Cancel();

 private:
  auto allowNewPairing(const QDBusObjectPath& device, const char* what) -> bool;
  WirelessAABootstrap* m_owner;
};

class WirelessAABootstrap : public QObject {
  Q_OBJECT

 public:
  struct Options {
    // Seconds the adapter stays discoverable and accepts new pairings after start().
    // 0 = never discoverable (only already-paired phones can connect).
    int pairingWindowSeconds{120};
    // How often to nudge paired phones to connect while no projection is active.
    int reconnectIntervalSeconds{20};
    // Bluetooth name shown on the phone; empty keeps the current adapter alias.
    QString adapterAlias;
    // RFCOMM channel for the Android Auto wireless service.
    quint16 rfcommChannel{8};
  };

  using WifiInfoProvider = std::function<aawireless::WifiInfo()>;

  explicit WirelessAABootstrap(QObject* parent = nullptr);
  ~WirelessAABootstrap() override;

  // Registers profiles/agent with BlueZ and starts the reconnect loop.
  // The provider is called at handshake time so the hotspot IP/BSSID are current.
  auto start(WifiInfoProvider provider, const Options& options) -> bool;
  void stop();
  auto isRunning() const -> bool { return m_running; }

  // While a projection session is active, stop nudging phones to reconnect.
  void setProjectionActive(bool active);

  // Replaces the hotspot details source (also used by tests, without BlueZ).
  void setWifiInfoProvider(WifiInfoProvider provider) { m_provider = std::move(provider); }

  // Re-opens the pairing window (discoverable + agent accepts pairing).
  void openPairingWindow(int seconds);
  auto pairingWindowOpen() const -> bool;

 signals:
  void handshakeStarted(const QString& devicePath);
  void credentialsSent(const QString& devicePath);
  void phoneReportedStatus(const QString& devicePath, int messageId, int status);
  void handshakeFailed(const QString& devicePath, const QString& reason);

 private:
  friend class WirelessAABluezProfile;
  friend class WirelessAABluezAgent;

  struct RfcommSession {
    int fd{-1};
    QSocketNotifier* notifier{nullptr};
    aawireless::FrameParser parser;
    bool credentialsSent{false};
  };

  void onNewConnection(bool isAndroidAuto, const QDBusObjectPath& device,
                       const QDBusUnixFileDescriptor& fd);
  void onRequestDisconnection(bool isAndroidAuto, const QDBusObjectPath& device);
  void onSessionReadable(const QString& devicePath);
  void handleFrame(const QString& devicePath, quint16 id, const QByteArray& payload);
  auto sendFrame(const QString& devicePath, aawireless::MessageId id, const QByteArray& payload)
      -> bool;
  void closeSession(const QString& devicePath);
  void closeAllSessions();

  auto findAdapterPath() -> QString;
  auto registerProfile(const QString& objectPath, const QString& uuid, const QVariantMap& options)
      -> bool;
  void unregisterProfile(const QString& objectPath);
  auto registerAgent() -> bool;
  void unregisterAgent();
  void setAdapterProperty(const QString& name, const QVariant& value);
  void markTrusted(const QString& devicePath);
  void reconnectTick();

  WifiInfoProvider m_provider;
  Options m_options;
  bool m_running{false};
  bool m_projectionActive{false};
  bool m_aaProfileRegistered{false};
  bool m_hspProfileRegistered{false};
  bool m_agentRegistered{false};
  QString m_adapterPath;
  qint64 m_pairingWindowEndsMs{0};
  int m_reconnectCursor{0};

  WirelessAABluezProfile* m_aaProfile{nullptr};
  WirelessAABluezProfile* m_hspProfile{nullptr};
  WirelessAABluezAgent* m_agent{nullptr};
  QTimer* m_reconnectTimer{nullptr};

  QHash<QString, RfcommSession> m_aaSessions;  // keyed by device object path
  QHash<QString, int> m_hspFds;                // held open so the link stays up
  QSet<QString> m_headsetNudged;               // connected phones already asked for HSP
  QHash<QString, qint64> m_hspSinceMs;         // when each headset link came up
  int m_headsetRetries{0};                     // reconnects since start() (capped)
  void logAdapterUuids();
  QStringList adapterUuids() const;
  bool adapterAdvertisesAndroidAuto() const;
  quint16 m_rfcommChannel{0};
  void reconnectHeadset(const QString& devicePath);
};
