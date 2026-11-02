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

#include "IbusProtocol.h"

#include "IkeText.h"

namespace crankshaft::ibus {

namespace {
auto byteAt(const QByteArray& bytes, int index) -> quint8 {
  return static_cast<quint8>(bytes.at(index));
}

template <typename T>
auto assign(T& field, const T& value) -> bool {
  if (field == value) {
    return false;
  }
  field = value;
  return true;
}

auto hexName(const char* prefix, quint8 value) -> QString {
  return QStringLiteral("%1_0x%2")
      .arg(QLatin1String(prefix))
      .arg(value, 2, 16, QLatin1Char('0'))
      .toLower();
}

auto bmbtButtonName(quint8 code) -> QString {
  switch (code) {
    case 0x00: return QStringLiteral("next");
    case 0x10: return QStringLiteral("previous");
    case 0x20: return QStringLiteral("select");
    case 0x04: return QStringLiteral("tone");
    case 0x11: return QStringLiteral("preset_1");
    case 0x01: return QStringLiteral("preset_2");
    case 0x12: return QStringLiteral("preset_3");
    case 0x02: return QStringLiteral("preset_4");
    case 0x13: return QStringLiteral("preset_5");
    case 0x03: return QStringLiteral("preset_6");
    case 0x21: return QStringLiteral("am");
    case 0x31: return QStringLiteral("fm");
    case 0x23: return QStringLiteral("mode_previous");
    case 0x33: return QStringLiteral("mode_next");
    case 0x30: return QStringLiteral("overlay");
    case 0x06: return QStringLiteral("power");
    case 0x24: return QStringLiteral("eject");
    case 0x14: return QStringLiteral("tape_side");
    case 0x32: return QStringLiteral("tp");
    case 0x22: return QStringLiteral("rds");
    case 0x08: return QStringLiteral("telephone");
    case 0x07: return QStringLiteral("clock");
    case 0x34: return QStringLiteral("menu");
    case 0x05: return QStringLiteral("confirm");
    default: return hexName("bmbt", code);
  }
}
}  // namespace

auto xorChecksum(const QByteArray& bytes) -> quint8 {
  quint8 checksum = 0;
  for (const char byte : bytes) {
    checksum ^= static_cast<quint8>(byte);
  }
  return checksum;
}

void IbusFrameParser::feed(const QByteArray& bytes) {
  m_buffer.append(bytes);
  parse();
}

auto IbusFrameParser::takeFrames() -> QList<IbusFrame> {
  QList<IbusFrame> frames;
  frames.swap(m_frames);
  return frames;
}

void IbusFrameParser::resyncStale() {
  while (!m_buffer.isEmpty()) {
    m_buffer.remove(0, 1);
    ++m_droppedBytes;
    parse();
    if (m_buffer.isEmpty()) {
      return;
    }
    // parse() stopped on an incomplete frame again; keep dropping.
  }
}

void IbusFrameParser::parse() {
  while (m_buffer.size() >= 2) {
    const int length = byteAt(m_buffer, 1);
    if (length < kMinLength || length > kMaxLength) {
      m_buffer.remove(0, 1);
      ++m_droppedBytes;
      continue;
    }
    const int total = length + 2;
    if (m_buffer.size() < total) {
      return;  // wait for the rest
    }
    const QByteArray candidate = m_buffer.left(total);
    if (xorChecksum(candidate.left(total - 1)) != byteAt(candidate, total - 1)) {
      m_buffer.remove(0, 1);
      ++m_droppedBytes;
      continue;
    }
    IbusFrame frame;
    frame.source = byteAt(candidate, 0);
    frame.destination = byteAt(candidate, 2);
    frame.data = candidate.mid(3, total - 4);
    frame.raw = candidate;
    m_frames.append(frame);
    m_buffer.remove(0, total);
  }
}

auto consumptionUnitText(int unit) -> QString {
  switch (unit) {
    case 1: return QStringLiteral("mpg");     // UK
    case 2: return QStringLiteral("mpg");     // US
    case 3: return QStringLiteral("km/L");
    default: return QStringLiteral("L/100");
  }
}

auto VehicleState::toVariantMap() const -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("ignition")] = ignition;
  map[QStringLiteral("speedKmh")] = speedKmh;
  map[QStringLiteral("rpm")] = rpm;
  map[QStringLiteral("ambientC")] = ambientC;
  map[QStringLiteral("coolantC")] = coolantC;
  map[QStringLiteral("gear")] = gear;
  map[QStringLiteral("reverse")] = reverse();
  map[QStringLiteral("handbrake")] = handbrake;
  map[QStringLiteral("time")] = time;
  map[QStringLiteral("date")] = date;
  map[QStringLiteral("outsideTemp")] = outsideTemp;
  map[QStringLiteral("consumption1")] = consumption1;
  map[QStringLiteral("consumption2")] = consumption2;
  map[QStringLiteral("range")] = range;
  map[QStringLiteral("distance")] = distance;
  map[QStringLiteral("arrival")] = arrival;
  map[QStringLiteral("limit")] = limit;
  map[QStringLiteral("avgSpeed")] = avgSpeed;
  map[QStringLiteral("timer")] = timer;
  map[QStringLiteral("unitsKnown")] = unitsKnown;
  map[QStringLiteral("clock12h")] = clock12h;
  map[QStringLiteral("temperatureUnit")] = fahrenheit ? QStringLiteral("°F") : QStringLiteral("°C");
  map[QStringLiteral("distanceUnit")] = miles ? QStringLiteral("mi") : QStringLiteral("km");
  map[QStringLiteral("speedUnit")] = avgSpeedMph ? QStringLiteral("mph") : QStringLiteral("km/h");
  map[QStringLiteral("consumption1Unit")] = consumptionUnitText(consumption1Unit);
  map[QStringLiteral("consumption2Unit")] = consumptionUnitText(consumption2Unit);
  return map;
}

auto applyClusterFrame(const IbusFrame& frame, VehicleState& state) -> bool {
  if (frame.source != kAddrIke || frame.data.isEmpty()) {
    return false;
  }
  const QByteArray& d = frame.data;
  switch (frame.command()) {
    case 0x11:
      if (d.size() >= 2) {
        return assign(state.ignition, byteAt(d, 1) & 0x07);
      }
      return false;
    case 0x15:
      if (d.size() >= 5) {
        const quint8 b2 = byteAt(d, 2);
        const quint8 b3 = byteAt(d, 3);
        bool changed = assign(state.unitsKnown, true);
        changed |= assign(state.clock12h, (b2 & 0x01) != 0);
        changed |= assign(state.fahrenheit, (b2 & 0x02) != 0);
        changed |= assign(state.avgSpeedMph, (b2 & 0x10) != 0);
        changed |= assign(state.miles, (b2 & 0x40) != 0);
        changed |= assign(state.consumption1Unit, b3 & 0x03);
        changed |= assign(state.consumption2Unit, (b3 & 0x0C) >> 2);
        return changed;
      }
      return false;
    case 0x13:
      if (d.size() >= 3) {
        bool changed = assign(state.handbrake, (byteAt(d, 1) & 0x01) != 0);
        changed |= assign(state.gear, static_cast<int>(byteAt(d, 2) >> 4));
        return changed;
      }
      return false;
    case 0x18:
      if (d.size() >= 3) {
        const quint8 speed = byteAt(d, 1);
        bool changed = assign(state.speedKmh, speed == 0xFF ? -1 : speed * 2);
        changed |= assign(state.rpm, byteAt(d, 2) * 100);
        return changed;
      }
      return false;
    case 0x19:
      if (d.size() >= 3) {
        bool changed = assign(state.ambientC, static_cast<int>(static_cast<qint8>(d.at(1))));
        changed |= assign(state.coolantC, static_cast<int>(static_cast<qint8>(d.at(2))));
        return changed;
      }
      return false;
    case 0x24: {
      if (d.size() < 3) {
        return false;
      }
      const quint8 property = byteAt(d, 1) & 0x3F;
      const QString text = QString::fromLatin1(d.mid(3)).trimmed();
      switch (property) {
        case ObcTime: return assign(state.time, text);
        case ObcDate: return assign(state.date, text);
        case ObcTemperature: return assign(state.outsideTemp, text);
        case ObcConsumption1: return assign(state.consumption1, text);
        case ObcConsumption2: return assign(state.consumption2, text);
        case ObcRange: return assign(state.range, text);
        case ObcDistance: return assign(state.distance, text);
        case ObcArrival: return assign(state.arrival, text);
        case ObcLimit: return assign(state.limit, text);
        case ObcAvgSpeed: return assign(state.avgSpeed, text);
        case ObcTimer: return assign(state.timer, text);
        default: return false;
      }
    }
    default:
      return false;
  }
}

auto ParkDistances::toVariantMap() const -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("active")] = active;
  map[QStringLiteral("frontLeft")] = frontLeft;
  map[QStringLiteral("frontCenterLeft")] = frontCenterLeft;
  map[QStringLiteral("frontCenterRight")] = frontCenterRight;
  map[QStringLiteral("frontRight")] = frontRight;
  map[QStringLiteral("rearLeft")] = rearLeft;
  map[QStringLiteral("rearCenterLeft")] = rearCenterLeft;
  map[QStringLiteral("rearCenterRight")] = rearCenterRight;
  map[QStringLiteral("rearRight")] = rearRight;
  return map;
}

auto pdcSensorRequestFrame() -> QByteArray {
  return ibusFrame(kAddrDiagnostics, kAddrPdc, QByteArray(1, static_cast<char>(0x1B)));
}

auto decodePdcDistances(const IbusFrame& frame, ParkDistances& out) -> bool {
  // 60 LL 3F A0 <rl> <rr> <rcl> <rcr> <fl> <fr> <fcl> <fcr> <status> ...
  // (byte order as decoded by BlueBus; data[0] is the command, data[1] a
  // status byte, data[10] bit 0 = PDC switched on).
  const QByteArray& d = frame.data;
  if (frame.source != kAddrPdc || frame.command() != 0xA0 || d.size() < 11) {
    return false;
  }
  out.rearLeft = byteAt(d, 2);
  out.rearRight = byteAt(d, 3);
  out.rearCenterLeft = byteAt(d, 4);
  out.rearCenterRight = byteAt(d, 5);
  out.frontLeft = byteAt(d, 6);
  out.frontRight = byteAt(d, 7);
  out.frontCenterLeft = byteAt(d, 8);
  out.frontCenterRight = byteAt(d, 9);
  out.active = (byteAt(d, 10) & 0x01) != 0;
  return true;
}

auto isPdcActivation(const IbusFrame& frame) -> bool {
  return frame.source == kAddrPdc &&
         (frame.command() == 0x07 || (frame.destination == 0xD0 && frame.command() == 0x5A));
}

auto obcControlFrame(quint8 property, quint8 control, quint8 source) -> QByteArray {
  QByteArray data;
  data.append(static_cast<char>(0x41));
  data.append(static_cast<char>(property));
  data.append(static_cast<char>(control));
  return ibusFrame(source, kAddrIke, data);
}

auto unitsRequestFrame(quint8 source) -> QByteArray {
  return ibusFrame(source, kAddrIke, QByteArray(1, static_cast<char>(0x14)));
}

auto stateName(InputEvent::State state) -> QString {
  switch (state) {
    case InputEvent::State::Hold: return QStringLiteral("hold");
    case InputEvent::State::Release: return QStringLiteral("release");
    case InputEvent::State::Press:
    default: return QStringLiteral("press");
  }
}

auto InputEvent::toVariantMap() const -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("kind")] = kind == Kind::Dial ? QStringLiteral("dial") : QStringLiteral("button");
  map[QStringLiteral("source")] = source;
  map[QStringLiteral("button")] = button;
  map[QStringLiteral("state")] = stateName(state);
  map[QStringLiteral("steps")] = steps;
  return map;
}

auto decodeInput(const IbusFrame& frame) -> std::optional<InputEvent> {
  if (frame.data.size() < 2) {
    return std::nullopt;
  }
  const quint8 value = byteAt(frame.data, 1);

  if (frame.source == kAddrBmbt && frame.command() == 0x49) {
    InputEvent event;
    event.kind = InputEvent::Kind::Dial;
    event.source = QStringLiteral("bmbt");
    event.button = QStringLiteral("dial");
    const int steps = value & 0x1F;
    event.steps = (value & 0x80) ? steps : -steps;
    return event;
  }

  if (frame.source == kAddrBmbt && frame.command() == 0x48) {
    InputEvent event;
    event.source = QStringLiteral("bmbt");
    event.button = bmbtButtonName(value & 0x3F);
    const quint8 state = value & 0xC0;
    event.state = state == 0x40   ? InputEvent::State::Hold
                  : state == 0x80 ? InputEvent::State::Release
                                  : InputEvent::State::Press;
    return event;
  }

  if (frame.source == kAddrMfl && frame.command() == 0x3B) {
    InputEvent event;
    event.source = QStringLiteral("mfl");
    if (value & 0x01) {
      event.button = QStringLiteral("next");
    } else if (value & 0x08) {
      event.button = QStringLiteral("previous");
    } else if (value & 0x40) {
      event.button = QStringLiteral("rt");
      return event;  // a toggle, no hold/release
    } else {
      // Answer/dial/hang-up: press 0x00, hold 0x80, release 0xA0.
      event.button = QStringLiteral("telephone");
      event.state = value == 0x80   ? InputEvent::State::Hold
                    : value == 0xA0 ? InputEvent::State::Release
                                    : InputEvent::State::Press;
      return event;
    }
    event.state = (value & 0x10)   ? InputEvent::State::Hold
                  : (value & 0x20) ? InputEvent::State::Release
                                   : InputEvent::State::Press;
    return event;
  }

  return std::nullopt;
}

}  // namespace crankshaft::ibus
