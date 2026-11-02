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

// IBus framing and the E38/E39/E53 messages Crankshaft reads and sends.
// Pure functions and value types only (no I/O), so they are unit-tested.
// Message formats follow wilhelm-docs (piersholt.github.io/wilhelm-docs):
//   frame          : SRC LEN DST DATA... XOR   (LEN = bytes after LEN)
//   IKE 0x11       : 80 04 BF 11 <ignition>              KL-30 00, KL-R 01, KL-15 03, KL-50 07
//   IKE 0x15       : 80 07 BF 15 <b1> <b2> <b3> <b4>     language and units
//   IKE 0x18       : 80 05 BF 18 <speed/2> <rpm/100>
//   IKE 0x19       : 80 06 BF 19 <ambient> <coolant> <ext>
//   IKE 0x24       : 80 LL xx 24 <property> <options> <text>   OBC values as text
//   GT  0x41       : 3B 05 80 41 <property> <control>    OBC control / text request
//   BMBT 0x48      : F0 04 xx 48 <button|state>
//   BMBT 0x49      : F0 04 3B 49 <dir|steps>
//   MFL 0x3B       : 50 04 xx 3B <button|state>

#include <QByteArray>
#include <QList>
#include <QString>
#include <QVariantMap>
#include <optional>

namespace crankshaft::ibus {

constexpr quint8 kAddrGt = 0x3B;
constexpr quint8 kAddrBmbt = 0xF0;
constexpr quint8 kAddrMfl = 0x50;
constexpr quint8 kAddrRadio = 0x68;
constexpr quint8 kAddrBroadcast = 0xBF;

/// Shortest and longest LEN byte accepted. LEN counts DST + data + checksum.
constexpr int kMinLength = 3;
constexpr int kMaxLength = 0x40;

struct IbusFrame {
  quint8 source = 0;
  quint8 destination = 0;
  QByteArray data;  // command byte first
  QByteArray raw;   // the whole frame including checksum

  [[nodiscard]] auto command() const -> int {
    return data.isEmpty() ? -1 : static_cast<quint8>(data.at(0));
  }
};

/// XOR of all bytes.
auto xorChecksum(const QByteArray& bytes) -> quint8;

/// Splits a byte stream into checksum-valid frames. On a bad checksum or an
/// impossible length one byte is dropped and parsing resumes from the next,
/// so it re-synchronises by itself after noise or a mid-frame start.
class IbusFrameParser {
 public:
  void feed(const QByteArray& bytes);
  /// Frames completed so far (cleared by this call).
  auto takeFrames() -> QList<IbusFrame>;
  /// Drop a partial frame.
  void reset() { m_buffer.clear(); }
  /// Call after the bus has been quiet for a while with a partial frame
  /// buffered: a noise byte with a plausible length can make the parser wait
  /// for bytes that never come. Recover any complete frames behind it and
  /// drop the rest.
  void resyncStale();
  [[nodiscard]] auto hasPartialFrame() const -> bool { return !m_buffer.isEmpty(); }
  [[nodiscard]] auto droppedBytes() const -> quint64 { return m_droppedBytes; }

 private:
  void parse();
  QByteArray m_buffer;
  QList<IbusFrame> m_frames;
  quint64 m_droppedBytes = 0;
};

/// OBC properties (IKE 0x24 / GT 0x41).
enum ObcProperty : quint8 {
  ObcTime = 0x01,
  ObcDate = 0x02,
  ObcTemperature = 0x03,
  ObcConsumption1 = 0x04,
  ObcConsumption2 = 0x05,
  ObcRange = 0x06,
  ObcDistance = 0x07,
  ObcArrival = 0x08,
  ObcLimit = 0x09,
  ObcAvgSpeed = 0x0A,
  ObcTimer = 0x0E,
};

enum ObcControl : quint8 {
  ObcRequestText = 0x01,
  ObcRequestStatus = 0x02,
  ObcActivate = 0x04,
  ObcDeactivate = 0x08,
  ObcRecalculate = 0x10,
  ObcSetCurrentSpeed = 0x20,
};

/// Everything the cluster tells us. Empty strings / -1 mean "not heard yet".
struct VehicleState {
  int ignition = -1;  // 0 KL-30, 1 KL-R, 3 KL-15, 7 KL-50
  int speedKmh = -1;
  int rpm = -1;
  int ambientC = -999;
  int coolantC = -999;

  QString time;
  QString date;
  QString outsideTemp;
  QString consumption1;
  QString consumption2;
  QString range;
  QString distance;
  QString arrival;
  QString limit;
  QString avgSpeed;
  QString timer;

  // From IKE 0x15
  bool unitsKnown = false;
  bool clock12h = false;
  bool fahrenheit = false;
  bool miles = false;
  bool avgSpeedMph = false;
  int consumption1Unit = 0;  // 0 L/100km, 1 mpg (UK), 2 mpg (US), 3 km/L
  int consumption2Unit = 0;

  [[nodiscard]] auto toVariantMap() const -> QVariantMap;
};

/// Apply a frame from the cluster to @p state. Returns true when something changed.
auto applyClusterFrame(const IbusFrame& frame, VehicleState& state) -> bool;

/// Text shown for a consumption unit code from 0x15.
auto consumptionUnitText(int unit) -> QString;

/// "3B 05 80 41 <property> <control> XX" (source is normally the GT).
auto obcControlFrame(quint8 property, quint8 control, quint8 source = kAddrGt) -> QByteArray;
/// "3B 03 80 14 XX": ask the cluster for 0x15 language and units.
auto unitsRequestFrame(quint8 source = kAddrGt) -> QByteArray;

/// A button or dial on the BMBT (on-board monitor) or MFL (steering wheel).
struct InputEvent {
  enum class Kind { Button, Dial };
  enum class State { Press, Hold, Release };

  Kind kind = Kind::Button;
  QString source;  // "bmbt" or "mfl"
  /// "next", "previous", "confirm", "menu", "telephone", "voice", "mode", ...
  /// Unknown buttons are "bmbt_0xNN" / "mfl_0xNN".
  QString button;
  State state = State::Press;
  int steps = 0;  // dial: + right, - left

  [[nodiscard]] auto toVariantMap() const -> QVariantMap;
};

/// Decode a BMBT/MFL input frame; nullopt for anything else.
auto decodeInput(const IbusFrame& frame) -> std::optional<InputEvent>;

auto stateName(InputEvent::State state) -> QString;

}  // namespace crankshaft::ibus
