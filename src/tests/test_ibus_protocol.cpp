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


#include "../services/ibus/IbusProtocol.h"
#include "../services/ibus/IbusService.h"
#include "../services/ibus/IkeText.h"

using namespace crankshaft::ibus;

namespace {
auto hex(const char* text) -> QByteArray {
  return QByteArray::fromHex(QByteArray(text).replace(' ', ""));
}

auto parseOne(const char* text) -> IbusFrame {
  IbusFrameParser parser;
  parser.feed(hex(text));
  const auto frames = parser.takeFrames();
  EXPECT_EQ(frames.size(), 1);
  return frames.isEmpty() ? IbusFrame{} : frames.first();
}

// IKE 0x24 frame with text, built with the same helper as outgoing frames.
auto obcText(quint8 property, const QByteArray& text) -> QByteArray {
  QByteArray data;
  data.append(static_cast<char>(0x24));
  data.append(static_cast<char>(property));
  data.append('\0');
  data.append(text);
  return ibusFrame(0x80, 0xFF, data);
}
}  // namespace

TEST(IbusProtocolTest, ObcRequestFramesMatchWilhelmDocs) {
  EXPECT_EQ(toHex(obcControlFrame(ObcTime, ObcRequestText)), "3B 05 80 41 01 01 FF");
  EXPECT_EQ(toHex(obcControlFrame(ObcRange, ObcRequestText)), "3B 05 80 41 06 01 F8");
  EXPECT_EQ(toHex(obcControlFrame(ObcConsumption1, ObcRecalculate)), "3B 05 80 41 04 10 EB");
  EXPECT_EQ(toHex(obcControlFrame(ObcLimit, ObcSetCurrentSpeed)), "3B 05 80 41 09 20 D6");
}

TEST(IbusProtocolTest, ParserSplitsBackToBackFrames) {
  IbusFrameParser parser;
  parser.feed(hex("80 04 BF 11 03 29 80 05 BF 18 41 19 7A"));
  const auto frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 2);
  EXPECT_EQ(frames[0].command(), 0x11);
  EXPECT_EQ(frames[1].command(), 0x18);
  EXPECT_EQ(frames[1].destination, 0xBF);
}

TEST(IbusProtocolTest, ParserResyncsAfterGarbageAndSplitReads) {
  IbusFrameParser parser;
  parser.feed(hex("00 FF 12"));         // noise
  parser.feed(hex("80 05 BF"));         // first half
  EXPECT_TRUE(parser.takeFrames().isEmpty());
  parser.feed(hex("18 41 19 7A"));      // second half
  parser.resyncStale();                 // the bus went quiet
  const auto frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_EQ(frames[0].raw, hex("80 05 BF 18 41 19 7A"));
  EXPECT_GT(parser.droppedBytes(), 0u);
}

TEST(IbusProtocolTest, ParserRejectsBadChecksum) {
  IbusFrameParser parser;
  parser.feed(hex("80 05 BF 18 41 19 7B"));
  EXPECT_TRUE(parser.takeFrames().isEmpty());
}

TEST(IbusProtocolTest, ClusterSpeedTemperatureIgnition) {
  VehicleState state;
  EXPECT_TRUE(applyClusterFrame(parseOne("80 05 BF 18 41 19 7A"), state));
  EXPECT_EQ(state.speedKmh, 130);
  EXPECT_EQ(state.rpm, 2500);
  EXPECT_TRUE(applyClusterFrame(parseOne("80 06 BF 19 0F 54 00 7B"), state));
  EXPECT_EQ(state.ambientC, 15);
  EXPECT_EQ(state.coolantC, 84);
  EXPECT_TRUE(applyClusterFrame(parseOne("80 04 BF 11 03 29"), state));
  EXPECT_EQ(state.ignition, 3);
  // Same values again: no change reported.
  EXPECT_FALSE(applyClusterFrame(parseOne("80 04 BF 11 03 29"), state));
}

TEST(IbusProtocolTest, NegativeAmbient) {
  VehicleState state;
  QByteArray data = hex("19 FB 50 00");  // -5 C
  IbusFrameParser parser;
  parser.feed(ibusFrame(0x80, 0xBF, data));
  const auto frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 1);
  applyClusterFrame(frames.first(), state);
  EXPECT_EQ(state.ambientC, -5);
}

TEST(IbusProtocolTest, UnitsFromLanguageRegion) {
  VehicleState state;
  EXPECT_TRUE(applyClusterFrame(parseOne("80 07 BF 15 01 85 60 42 8B"), state));
  EXPECT_TRUE(state.unitsKnown);
  EXPECT_TRUE(state.clock12h);
  EXPECT_FALSE(state.miles);
  EXPECT_FALSE(state.fahrenheit);
  EXPECT_EQ(state.consumption1Unit, 0);
  EXPECT_EQ(state.toVariantMap().value("distanceUnit").toString(), "km");
}

TEST(IbusProtocolTest, ObcTextValuesAreTrimmed) {
  VehicleState state;
  IbusFrameParser parser;
  parser.feed(obcText(ObcRange, " 512") + obcText(ObcConsumption1, "10.2 ") +
              obcText(ObcTime, "12:34PM") + obcText(ObcTemperature, " +21.0"));
  for (const auto& frame : parser.takeFrames()) {
    applyClusterFrame(frame, state);
  }
  EXPECT_EQ(state.range, "512");
  EXPECT_EQ(state.consumption1, "10.2");
  EXPECT_EQ(state.time, "12:34PM");
  EXPECT_EQ(state.outsideTemp, "+21.0");
}

TEST(IbusProtocolTest, OnlyClusterFramesChangeState) {
  VehicleState state;
  EXPECT_FALSE(applyClusterFrame(parseOne("3B 05 80 41 01 01 FF"), state));
}

TEST(IbusProtocolTest, BmbtDial) {
  auto left = decodeInput(parseOne("F0 04 3B 49 02 84"));
  ASSERT_TRUE(left.has_value());
  EXPECT_EQ(left->kind, InputEvent::Kind::Dial);
  EXPECT_EQ(left->steps, -2);
  auto right = decodeInput(parseOne("F0 04 3B 49 81 07"));
  ASSERT_TRUE(right.has_value());
  EXPECT_EQ(right->steps, 1);
}

TEST(IbusProtocolTest, BmbtButtonsWithState) {
  auto preset = decodeInput(parseOne("F0 04 68 48 11 C5"));
  ASSERT_TRUE(preset.has_value());
  EXPECT_EQ(preset->button, "preset_1");
  EXPECT_EQ(preset->state, InputEvent::State::Press);

  IbusFrameParser parser;
  parser.feed(ibusFrame(0xF0, 0x3B, hex("48 85")));  // confirm release
  parser.feed(ibusFrame(0xF0, 0xFF, hex("48 74")));  // menu hold
  const auto frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 2);
  auto confirm = decodeInput(frames[0]);
  ASSERT_TRUE(confirm.has_value());
  EXPECT_EQ(confirm->button, "confirm");
  EXPECT_EQ(confirm->state, InputEvent::State::Release);
  auto menu = decodeInput(frames[1]);
  ASSERT_TRUE(menu.has_value());
  EXPECT_EQ(menu->button, "menu");
  EXPECT_EQ(menu->state, InputEvent::State::Hold);
}

TEST(IbusProtocolTest, MflButtons) {
  auto next = decodeInput(parseOne("50 04 68 3B 01 06"));
  ASSERT_TRUE(next.has_value());
  EXPECT_EQ(next->source, "mfl");
  EXPECT_EQ(next->button, "next");
  EXPECT_EQ(next->state, InputEvent::State::Press);
  auto backRelease = decodeInput(parseOne("50 04 68 3B 28 2F"));
  ASSERT_TRUE(backRelease.has_value());
  EXPECT_EQ(backRelease->button, "previous");
  EXPECT_EQ(backRelease->state, InputEvent::State::Release);
  auto telHold = decodeInput(parseOne("50 04 C8 3B 80 27"));
  ASSERT_TRUE(telHold.has_value());
  EXPECT_EQ(telHold->button, "telephone");
  EXPECT_EQ(telHold->state, InputEvent::State::Hold);
  auto rt = decodeInput(parseOne("50 04 C8 3B 40 E7"));
  ASSERT_TRUE(rt.has_value());
  EXPECT_EQ(rt->button, "rt");
}

TEST(IbusProtocolTest, DefaultKeyMapping) {
  IbusService service;
  InputEvent confirm;
  confirm.source = "bmbt";
  confirm.button = "confirm";
  EXPECT_EQ(service.keyCodeFor(confirm), 23);  // DPAD_CENTER
  InputEvent next;
  next.source = "mfl";
  next.button = "next";
  EXPECT_EQ(service.keyCodeFor(next), 87);  // MEDIA_NEXT
  InputEvent menu;
  menu.source = "bmbt";
  menu.button = "menu";
  EXPECT_EQ(service.keyCodeFor(menu), -1);  // handled by the UI
}

TEST(IbusProtocolTest, NothingSentWhileNoInterface) {
  IbusService service;
  EXPECT_FALSE(service.busAwake());
  // Must not crash or queue anything without an open port.
  service.setClusterText({{"line", "> 300m Galle Rd"}, {"alert", true}});
  service.sendObcControl(ObcConsumption1, ObcRecalculate);
  EXPECT_FALSE(service.status().value("connected").toBool());
}

// ---------------------------------------------------------------------------
// End to end over a pseudo-terminal standing in for the Resler interface: the
// "bus" side echoes what the service sends (as the transceiver does) and
// answers OBC requests like the cluster.
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <QCoreApplication>
#include <QElapsedTimer>
#include <memory>

#include "../services/config/ConfigService.h"

namespace {
struct FakeBus {
  int master = -1;
  QString slavePath;
  IbusFrameParser fromService;
  QList<QByteArray> sent;  // frames the service transmitted
  int dropEchoes = 0;      // swallow the next N echoes (collision)
  bool hasPdc = false;     // a PDC module answers 0x1B requests
  int pdcRearCenter = 40;  // cm
  bool pdcOn = true;

  FakeBus() {
    int slave = -1;
    char name[128] = {};
    if (openpty(&master, &slave, name, nullptr, nullptr) == 0) {
      slavePath = QString::fromLatin1(name);
      ::close(slave);  // the service opens it by name
      fcntl(master, F_SETFL, fcntl(master, F_GETFL) | O_NONBLOCK);
    }
  }
  ~FakeBus() {
    if (master >= 0) ::close(master);
  }
  void send(const QByteArray& frame) const { (void)::write(master, frame.constData(), frame.size()); }

  // Run the Qt loop for @p ms while acting as the bus.
  void run(int ms) {
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < ms) {
      QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
      char buf[256];
      const ssize_t n = ::read(master, buf, sizeof(buf));
      if (n > 0) {
        fromService.feed(QByteArray(buf, static_cast<int>(n)));
        for (const auto& frame : fromService.takeFrames()) {
          sent.append(frame.raw);
          if (dropEchoes > 0) {
            --dropEchoes;
            continue;
          }
          send(frame.raw);  // echo
          if (frame.destination == 0x80 && frame.command() == 0x41 &&
              static_cast<quint8>(frame.data.at(1)) == ObcRange) {
            QByteArray data = QByteArray::fromHex("240600");
            data.append(" 512");
            send(ibusFrame(0x80, 0xFF, data));
          }
          if (hasPdc && frame.destination == 0x60 && frame.command() == 0x1B) {
            // A0 status rl rr rcl rcr fl fr fcl fcr flags
            QByteArray data = QByteArray::fromHex("A000");
            for (int v : {90, 255, pdcRearCenter, pdcRearCenter + 5, 255, 255, 120, 255}) {
              data.append(static_cast<char>(v));
            }
            data.append(static_cast<char>(pdcOn ? 0x01 : 0x00));
            send(ibusFrame(0x60, 0x3F, data));
          }
        }
      }
      usleep(1000);
    }
  }
};

auto ensureApp() -> QCoreApplication* {
  static int argc = 1;
  static char arg0[] = "tests";
  static char* argv[] = {arg0, nullptr};
  if (!QCoreApplication::instance()) {
    static auto* app = new QCoreApplication(argc, argv);
    return app;
  }
  return QCoreApplication::instance();
}
}  // namespace

TEST(IbusServiceTest, PollsClusterAndReadsAnswersOverSerial) {
  ensureApp();
  FakeBus bus;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  ConfigService::instance().set("core.ibus.poll_obc", true);
  IbusService service;
  QVariantMap lastState;
  QObject::connect(&service, &IbusService::vehicleStateChanged,
                   [&](const QVariantMap& state) { lastState = state; });
  service.start();
  ASSERT_TRUE(service.status().value("connected").toBool());

  // Nothing is sent while the bus is silent (asleep).
  bus.run(100);
  EXPECT_TRUE(bus.sent.isEmpty());

  bus.send(QByteArray::fromHex("8004BF11012B"));  // KL-R wakes it
  bus.run(1500);

  ASSERT_GE(bus.sent.size(), 6);
  EXPECT_EQ(toHex(bus.sent.first()), "3B 03 80 14 AC");  // units request
  EXPECT_EQ(lastState.value("ignition").toInt(), 1);
  EXPECT_EQ(lastState.value("range").toString(), "512");
  const auto status = service.status();
  EXPECT_TRUE(status.value("echo").toBool());
  EXPECT_EQ(status.value("collisions").toULongLong(), 0u);
  EXPECT_EQ(status.value("framesTx").toULongLong(), static_cast<qulonglong>(bus.sent.size()));
  ConfigService::instance().set("core.ibus.device", "auto");
  ConfigService::instance().set("core.ibus.poll_obc", false);
}

TEST(IbusServiceTest, ResendsWhenEchoIsMissing) {
  ensureApp();
  FakeBus bus;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  ConfigService::instance().set("core.ibus.poll_obc", true);
  ConfigService::instance().set("core.ibus.poll_interval_s", 3600);
  IbusService service;
  service.start();
  bus.send(QByteArray::fromHex("8004BF110329"));  // KL-15
  bus.run(600);  // first poll goes out and is echoed: echo is known to work
  const int before = bus.sent.size();
  ASSERT_GE(before, 1);

  bus.dropEchoes = 1;  // the next frame "collides"
  service.sendObcControl(ObcConsumption1, ObcRecalculate);
  bus.run(800);

  // Control sent twice (first lost) + the refresh request.
  const QByteArray control = obcControlFrame(ObcConsumption1, ObcRecalculate);
  EXPECT_EQ(bus.sent.mid(before).count(control), 2);
  EXPECT_EQ(service.status().value("collisions").toULongLong(), 1u);
  EXPECT_EQ(service.status().value("txFailures").toULongLong(), 0u);
  ConfigService::instance().set("core.ibus.device", "auto");
  ConfigService::instance().set("core.ibus.poll_obc", false);
  ConfigService::instance().set("core.ibus.poll_interval_s", 10);
}

TEST(IbusServiceTest, ClusterTextGoesToIke) {
  ensureApp();
  FakeBus bus;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  IbusService service;
  service.start();
  bus.send(QByteArray::fromHex("8004BF110329"));
  bus.run(200);
  EXPECT_TRUE(bus.sent.isEmpty());  // OBC polling is off by default

  service.setClusterText({{"line", "> 300m Galle Rd"}, {"alert", true}});
  service.setClusterText({{"line", "> 300m Galle Rd"}, {"alert", true}});  // unchanged: once
  service.setClusterText({{"line", "> 300m Galle Rd"}, {"alert", false}});
  bus.run(600);

  ASSERT_EQ(bus.sent.size(), 3);
  EXPECT_EQ(bus.sent[0], ikeTelephoneTextFrame("> 300m Galle Rd"));
  EXPECT_EQ(bus.sent[1], ikeCheckControlTextFrame("> 300m Galle Rd"));
  EXPECT_EQ(bus.sent[2], ikeCheckControlClearFrame());
  ConfigService::instance().set("core.ibus.device", "auto");
}

TEST(IbusServiceTest, BcButtonStopsScrollingUntilNewSongOrTurn) {
  ensureApp();
  FakeBus bus;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  IbusService service;
  service.start();
  bus.send(QByteArray::fromHex("8004BF110329"));  // ignition on (KL-15)
  bus.run(200);

  service.setClusterText({{"line", "Song one"}, {"contentKey", "media:Song one"}});
  bus.run(300);
  ASSERT_EQ(bus.sent.size(), 1);

  bus.send(QByteArray::fromHex("8004FF57022E"));  // BC button on the stalk
  bus.run(200);
  EXPECT_TRUE(service.status().value("clusterTextPaused").toBool());
  bus.sent.clear();
  service.setClusterText({{"line", "ong one S"}, {"contentKey", "media:Song one"}});  // scrolling
  service.setClusterText({{"line", "ng one So"}, {"contentKey", "media:Song one"}});
  bus.run(300);
  EXPECT_TRUE(bus.sent.isEmpty());
  EXPECT_TRUE(service.status().value("clusterTextPaused").toBool());

  service.setClusterText({{"line", "Song two"}, {"contentKey", "media:Song two"}});  // next song
  bus.run(300);
  ASSERT_EQ(bus.sent.size(), 1);
  EXPECT_EQ(bus.sent[0], ikeTelephoneTextFrame("Song two"));
  EXPECT_FALSE(service.status().value("clusterTextPaused").toBool());

  // Navigation: the distance counting down is not new information...
  bus.send(QByteArray::fromHex("8004FF57022E"));
  bus.run(100);
  bus.sent.clear();
  service.setClusterText({{"line", "> 300m Galle Rd"}, {"contentKey", "navigation:2:0:Galle Rd"}});
  bus.run(300);
  ASSERT_EQ(bus.sent.size(), 1);  // ...but switching from song to a turn is
  bus.send(QByteArray::fromHex("8004FF57022E"));
  bus.run(100);
  bus.sent.clear();
  service.setClusterText({{"line", "> 250m Galle Rd"}, {"contentKey", "navigation:2:0:Galle Rd"}});
  bus.run(300);
  EXPECT_TRUE(bus.sent.isEmpty());
  service.setClusterText({{"line", "< 400m Duplication"}, {"contentKey", "navigation:1:0:Duplication"}});
  bus.run(300);
  ASSERT_EQ(bus.sent.size(), 1);
  EXPECT_EQ(bus.sent[0], ikeTelephoneTextFrame("< 400m Duplication"));

  // Switched off in Settings: nothing is sent at all.
  service.setSettings(false, true, true, false);
  bus.sent.clear();
  service.setClusterText({{"line", "Song three"}});
  bus.run(300);
  EXPECT_TRUE(bus.sent.isEmpty());
  ConfigService::instance().set("core.ibus.device", "auto");
}

TEST(IbusProtocolTest, GearAndParkDistances) {
  VehicleState state;
  IbusFrame frame;
  IbusFrameParser parser;
  parser.feed(QByteArray::fromHex("800ABF130111000000004670"));  // engine running, reverse
  auto frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 1);
  EXPECT_TRUE(applyClusterFrame(frames[0], state));
  EXPECT_TRUE(state.reverse());
  EXPECT_TRUE(state.handbrake);

  QByteArray data = QByteArray::fromHex("A000");
  for (int v : {30, 255, 25, 28, 255, 200, 120, 100}) data.append(static_cast<char>(v));
  data.append('\x01');
  parser.feed(ibusFrame(0x60, 0x3F, data));
  frames = parser.takeFrames();
  ASSERT_EQ(frames.size(), 1);
  ParkDistances pdc;
  ASSERT_TRUE(decodePdcDistances(frames[0], pdc));
  EXPECT_TRUE(pdc.active);
  EXPECT_EQ(pdc.rearLeft, 30);
  EXPECT_EQ(pdc.rearRight, 255);
  EXPECT_EQ(pdc.rearCenterLeft, 25);
  EXPECT_EQ(pdc.rearCenterRight, 28);
  EXPECT_EQ(pdc.frontRight, 200);
  EXPECT_EQ(pdc.frontCenterLeft, 120);
  EXPECT_EQ(pdc.frontCenterRight, 100);
  EXPECT_EQ(pdcSensorRequestFrame(), QByteArray::fromHex("3F03601B47"));
}

TEST(IbusServiceTest, ReverseShowsParkingSensors) {
  ensureApp();
  FakeBus bus;
  bus.hasPdc = true;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  IbusService service;
  QList<QVariantMap> updates;
  QObject::connect(&service, &IbusService::parkingChanged,
                   [&](const QVariantMap& p) { updates.append(p); });
  service.start();
  bus.send(QByteArray::fromHex("8004BF110329"));
  bus.run(200);
  EXPECT_TRUE(bus.sent.isEmpty());  // nothing asked outside reverse

  bus.send(QByteArray::fromHex("800ABF130111000000004670"));  // reverse
  bus.run(1700);
  EXPECT_GE(bus.sent.count(pdcSensorRequestFrame()), 2);
  ASSERT_FALSE(updates.isEmpty());
  EXPECT_TRUE(service.parkingState().value("visible").toBool());
  EXPECT_EQ(service.parkingState().value("rearCenterLeft").toInt(), 40);

  bus.pdcRearCenter = 20;  // getting closer
  bus.run(900);
  EXPECT_EQ(service.parkingState().value("rearCenterLeft").toInt(), 20);

  // Out of reverse and the PDC switches off: screen goes, polling stops.
  bus.pdcOn = false;
  bus.send(ibusFrame(0x80, 0xBF, QByteArray::fromHex("1300810000000046")));  // drive
  bus.run(900);
  EXPECT_FALSE(service.parkingState().value("visible").toBool());
  bus.run(4000);  // PDC "on" window runs out
  bus.sent.clear();
  bus.run(2000);
  EXPECT_EQ(bus.sent.count(pdcSensorRequestFrame()), 0);

  // Switched off in Settings: reverse asks nothing.
  service.setSettings(false, true, true, true, false);
  bus.sent.clear();
  bus.send(QByteArray::fromHex("800ABF130111000000004670"));
  bus.run(1000);
  EXPECT_EQ(bus.sent.count(pdcSensorRequestFrame()), 0);
  EXPECT_FALSE(service.parkingState().value("visible").toBool());
  ConfigService::instance().set("core.ibus.device", "auto");
}

TEST(IbusServiceTest, ReverseWithoutPdcModuleGivesUp) {
  ensureApp();
  FakeBus bus;  // no PDC module
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  IbusService service;
  service.start();
  bus.send(QByteArray::fromHex("8004BF110329"));
  bus.send(QByteArray::fromHex("800ABF130111000000004670"));
  bus.run(6000);
  EXPECT_EQ(bus.sent.count(pdcSensorRequestFrame()), 4);
  EXPECT_FALSE(service.parkingState().value("visible").toBool());
  ConfigService::instance().set("core.ibus.device", "auto");
}

TEST(IbusServiceTest, SettingsTurnOffPollingAndCarControls) {
  ensureApp();
  FakeBus bus;
  ASSERT_GE(bus.master, 0);
  ConfigService::instance().set("core.ibus.device", bus.slavePath);
  IbusService service;
  QList<QVariantMap> inputs;
  QList<int> keys;
  QObject::connect(&service, &IbusService::inputReceived,
                   [&](const QVariantMap& e) { inputs.append(e); });
  QObject::connect(&service, &IbusService::androidAutoKey,
                   [&](int code, bool) { keys.append(code); });
  service.start();

  service.setSettings(true, false, true);  // poll on, BMBT off, MFL on
  bus.send(QByteArray::fromHex("8004BF110329"));
  bus.run(800);
  EXPECT_GE(bus.sent.size(), 6);  // units + OBC requests

  bus.send(ibusFrame(0xF0, 0x3B, QByteArray::fromHex("4805")));  // BMBT confirm: ignored
  bus.send(QByteArray::fromHex("5004683B0106"));                  // MFL next: used
  bus.run(200);
  ASSERT_EQ(inputs.size(), 1);
  EXPECT_EQ(inputs[0].value("source").toString(), "mfl");
  EXPECT_EQ(keys, QList<int>({87}));

  service.setSettings(false, true, false);  // poll off, BMBT on, MFL off
  bus.sent.clear();
  bus.send(QByteArray::fromHex("5004683B0106"));
  bus.run(300);
  EXPECT_EQ(inputs.size(), 1);
  EXPECT_TRUE(bus.sent.isEmpty());
  EXPECT_FALSE(service.status().value("pollObc").toBool());
  ConfigService::instance().set("core.ibus.device", "auto");
}
