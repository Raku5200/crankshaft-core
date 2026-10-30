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

#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QDBusObjectPath>
#include <QDBusUnixFileDescriptor>
#include <QElapsedTimer>
#include <memory>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "../hal/wireless/WirelessAABootstrap.h"

using namespace aawireless;

namespace {
auto bytes(std::initializer_list<int> values) -> QByteArray {
  QByteArray out;
  for (int v : values) {
    out.append(static_cast<char>(v));
  }
  return out;
}
}  // namespace

TEST(WirelessAABootstrapTest, EncodesWifiStartRequest) {
  // field 1 (string) "10.42.0.1", field 2 (varint) 5288
  const QByteArray expected = bytes({0x0A, 9}) + QByteArray("10.42.0.1") + bytes({0x10, 0xA8, 0x29});
  EXPECT_EQ(encodeWifiStartRequest(QStringLiteral("10.42.0.1"), 5288), expected);
}

TEST(WirelessAABootstrapTest, EncodesWifiInfoResponseWithWpa2AndDynamicAp) {
  WifiInfo info;
  info.ssid = QStringLiteral("AA");
  info.password = QStringLiteral("secret12");
  info.bssid = QStringLiteral("b8:27:eb:00:00:01");
  const QByteArray expected = bytes({0x0A, 2}) + QByteArray("AA") +
                              bytes({0x12, 8}) + QByteArray("secret12") +
                              bytes({0x1A, 17}) + QByteArray("b8:27:eb:00:00:01") +
                              bytes({0x20, 8,     // security_mode = WPA2_PERSONAL (8)
                                     0x28, 1});   // access_point_type = DYNAMIC (1)
  EXPECT_EQ(encodeWifiInfoResponse(info), expected);
}

TEST(WirelessAABootstrapTest, OpenNetworkUsesOpenSecurityMode) {
  WifiInfo info;
  info.ssid = QStringLiteral("AA");
  info.bssid = QStringLiteral("x");
  const QByteArray encoded = encodeWifiInfoResponse(info);
  qint64 security = -1;
  ASSERT_TRUE(decodeVarintField(encoded, 4, &security));
  EXPECT_EQ(security, 1);
}

TEST(WirelessAABootstrapTest, FramesWithBigEndianLengthAndId) {
  const QByteArray frame = frameMessage(MessageId::WifiInfoResponse, QByteArray(300, 'x'));
  ASSERT_EQ(frame.size(), 304);
  EXPECT_EQ(static_cast<quint8>(frame[0]), 0x01);  // 300 = 0x012C
  EXPECT_EQ(static_cast<quint8>(frame[1]), 0x2C);
  EXPECT_EQ(static_cast<quint8>(frame[2]), 0x00);
  EXPECT_EQ(static_cast<quint8>(frame[3]), 0x03);
}

TEST(WirelessAABootstrapTest, ParserHandlesSplitAndBackToBackFrames) {
  const QByteArray stream = frameMessage(MessageId::WifiInfoRequest, {}) +
                            frameMessage(MessageId::WifiStartResponse, bytes({0x18, 0x00}));
  FrameParser parser;
  quint16 id = 0;
  QByteArray payload;

  parser.append(stream.left(3));
  EXPECT_FALSE(parser.next(&id, &payload));
  parser.append(stream.mid(3, 3));
  ASSERT_TRUE(parser.next(&id, &payload));
  EXPECT_EQ(id, static_cast<quint16>(MessageId::WifiInfoRequest));
  EXPECT_TRUE(payload.isEmpty());
  EXPECT_FALSE(parser.next(&id, &payload));

  parser.append(stream.mid(6));
  ASSERT_TRUE(parser.next(&id, &payload));
  EXPECT_EQ(id, static_cast<quint16>(MessageId::WifiStartResponse));
  EXPECT_EQ(payload, bytes({0x18, 0x00}));
  EXPECT_EQ(parser.bufferedBytes(), 0);
}

TEST(WirelessAABootstrapTest, DecodesNegativeStatus) {
  // WifiStartResponse { status (field 3) = -3 } encoded as 10-byte varint
  const QByteArray payload =
      bytes({0x18, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01});
  qint64 status = 0;
  ASSERT_TRUE(decodeVarintField(payload, 3, &status));
  EXPECT_EQ(status, -3);
}

TEST(WirelessAABootstrapTest, DecodeSkipsOtherFieldsAndReportsMissing) {
  const QByteArray payload = bytes({0x0A, 2, 'h', 'i', 0x08, 0x05});
  qint64 value = 0;
  ASSERT_TRUE(decodeVarintField(payload, 1, &value));
  EXPECT_EQ(value, 5);
  EXPECT_FALSE(decodeVarintField(payload, 3, &value));
}

// ---------------------------------------------------------------------------
// End-to-end RFCOMM handshake over a socket pair; the test plays the phone.
// ---------------------------------------------------------------------------
namespace {

void pumpEvents(int ms) {
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < ms) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
  }
}

auto phoneReadFrame(int fd, quint16* id, QByteArray* payload) -> bool {
  FrameParser parser;
  QElapsedTimer timer;
  timer.start();
  while (timer.elapsed() < 2000) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
    pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, 20) > 0) {
      char buffer[512];
      const ssize_t n = ::read(fd, buffer, sizeof(buffer));
      if (n <= 0) {
        return false;
      }
      parser.append(QByteArray(buffer, static_cast<int>(n)));
      if (parser.next(id, payload)) {
        return true;
      }
    }
  }
  return false;
}

void phoneWrite(int fd, MessageId id, const QByteArray& payload) {
  const QByteArray frame = frameMessage(id, payload);
  ASSERT_EQ(::write(fd, frame.constData(), static_cast<size_t>(frame.size())), frame.size());
}

}  // namespace

TEST(WirelessAABootstrapTest, CompletesHandshakeWithPhoneOverRfcommSocket) {
  std::unique_ptr<QCoreApplication> app;
  if (!QCoreApplication::instance()) {
    static int argc = 1;
    static char name[] = "crankshaft-core-tests";
    static char* argv[] = {name, nullptr};
    app = std::make_unique<QCoreApplication>(argc, argv);
  }

  int sv[2];
  ASSERT_EQ(::socketpair(AF_UNIX, SOCK_STREAM, 0, sv), 0);
  const int headUnitFd = sv[0];
  const int phoneFd = sv[1];

  WifiInfo info;
  info.ssid = QStringLiteral("Crankshaft-AA");
  info.password = QStringLiteral("pw-123456789");
  info.bssid = QStringLiteral("b8:27:eb:12:34:56");
  info.ipAddress = QStringLiteral("10.42.0.1");
  info.port = 5277;

  WirelessAABootstrap bootstrap;
  bootstrap.setWifiInfoProvider([info]() { return info; });

  bool credentialsSent = false;
  int reportedStatus = 99;
  QString failure;
  QObject::connect(&bootstrap, &WirelessAABootstrap::credentialsSent,
                   [&](const QString&) { credentialsSent = true; });
  QObject::connect(&bootstrap, &WirelessAABootstrap::phoneReportedStatus,
                   [&](const QString&, int, int status) { reportedStatus = status; });
  QObject::connect(&bootstrap, &WirelessAABootstrap::handshakeFailed,
                   [&](const QString&, const QString& reason) { failure = reason; });

  // BlueZ hands the connected RFCOMM socket to the profile.
  WirelessAABluezProfile profile(&bootstrap, true);
  profile.NewConnection(QDBusObjectPath(QStringLiteral("/org/bluez/hci0/dev_AA_BB_CC_DD_EE_FF")),
                        QDBusUnixFileDescriptor(headUnitFd), QVariantMap());
  ::close(headUnitFd);  // the bootstrap keeps its own duplicate

  // 1. Head unit -> phone: WifiStartRequest {ip, port}
  quint16 id = 0;
  QByteArray payload;
  ASSERT_TRUE(phoneReadFrame(phoneFd, &id, &payload));
  EXPECT_EQ(id, static_cast<quint16>(MessageId::WifiStartRequest));
  EXPECT_EQ(payload, encodeWifiStartRequest(info.ipAddress, info.port));

  // 2. Phone -> head unit: WifiInfoRequest; head unit answers with credentials.
  phoneWrite(phoneFd, MessageId::WifiInfoRequest, {});
  ASSERT_TRUE(phoneReadFrame(phoneFd, &id, &payload));
  EXPECT_EQ(id, static_cast<quint16>(MessageId::WifiInfoResponse));
  EXPECT_EQ(payload, encodeWifiInfoResponse(info));
  EXPECT_TRUE(credentialsSent);

  // 3. Phone -> head unit: WifiStartResponse {status = SUCCESS}
  phoneWrite(phoneFd, MessageId::WifiStartResponse, bytes({0x18, 0x00}));
  pumpEvents(100);
  EXPECT_EQ(reportedStatus, 0);
  EXPECT_TRUE(failure.isEmpty());

  // 4. Phone reports a wrong-password failure -> surfaced as handshakeFailed.
  phoneWrite(phoneFd, MessageId::WifiConnectionStatus,
             bytes({0x08, 0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x01}));
  pumpEvents(100);
  EXPECT_EQ(reportedStatus, -3);
  EXPECT_FALSE(failure.isEmpty());

  // 5. Phone hangs up; the session must be cleaned up without crashing.
  ::close(phoneFd);
  pumpEvents(100);
}
