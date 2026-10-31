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

// Text for the E38/E39/E53 high-OBC instrument cluster (IKE) 20-character
// display, and the IBus frames that put it there. Message formats follow
// BlueBus (github.com/tedsalmon/BlueBus) and wilhelm-docs:
//   TEL -> IKE  0x23 title text  : C8 LL 80 23 42 32 <text>        steady line
//   PDC -> IKE  0x1A check ctrl  : 60 LL 80 1A 36 00 <20 chars>    pop-up line
//   PDC -> IKE  0x1A clear       : 60 05 80 1A 30 00 XX
// Pure functions only; the serial IbusService that sends them comes later.

#include <QByteArray>
#include <QString>

#include "../android_auto/ProjectionInfo.h"

namespace crankshaft::ibus {

constexpr int kIkeTextWidth = 20;

// IBus device addresses used here.
constexpr quint8 kAddrIke = 0x80;
constexpr quint8 kAddrTelephone = 0xC8;
constexpr quint8 kAddrPdc = 0x60;

/// Symbols drawn in front of a turn. Plain ASCII by default so it works on
/// every cluster; each can be swapped for one of the cluster's own glyphs
/// (e.g. its arrows or triangles above 0x80) via config, once confirmed on
/// the car. Values are raw bytes in the cluster character set.
struct IkeGlyphs {
  QByteArray left = "<";
  QByteArray right = ">";
  QByteArray slightLeft = "\\";
  QByteArray slightRight = "/";
  QByteArray sharpLeft = "<<";
  QByteArray sharpRight = ">>";
  QByteArray straight = "^";
  QByteArray uTurn = "U";
  QByteArray keepLeft = "<|";
  QByteArray keepRight = "|>";
  QByteArray fork = "Y";
  QByteArray merge = "M";
  QByteArray roundabout = "O";
  QByteArray destination = "*";
  QByteArray ferry = "~";
  QByteArray music = "";  // optional prefix for the media line
};

/// Parse a glyph spec from config: literal text with optional \xNN escapes,
/// e.g. "<", "\\xCA", "\\xAD\\xAD".
auto parseGlyphSpec(const QString& spec) -> QByteArray;

/// Bytes the cluster can show: printable ASCII kept, accents folded
/// (e.g. "Café" -> "Cafe"), anything else dropped. Glyph bytes are passed
/// through untouched by the callers, not through this.
auto toIkeCharset(const QString& text) -> QByteArray;

/// Shorten common road words to fit 20 characters ("Galle Road" -> "Galle Rd",
/// "Baseline Mawatha" -> "Baseline Mw").
auto abbreviateRoad(const QString& road) -> QString;

/// Compact distance for the cluster: "300 m" -> "300m", "1.2 km" -> "1.2km".
auto compactDistance(const QString& distanceText) -> QString;

auto maneuverGlyph(projection::Maneuver maneuver, int roundaboutExit, const IkeGlyphs& glyphs)
    -> QByteArray;

/// The fixed part ("> 300m ") and the road name of a navigation line.
/// Both empty when not guiding; while rerouting the road is "Rerouting...".
struct NavigationLineParts {
  QByteArray prefix;
  QByteArray road;
  /// Characters left for the road after the prefix.
  [[nodiscard]] int roadWidth() const { return kIkeTextWidth - static_cast<int>(prefix.size()); }
  [[nodiscard]] bool roadNeedsScroll() const { return road.size() > roadWidth(); }
};
auto formatNavigationParts(const projection::NavigationGuidance& guidance,
                           const IkeGlyphs& glyphs = {}) -> NavigationLineParts;

/// "<glyph> <distance> <road>" fitted to 20 bytes, e.g. "> 300m Galle Rd".
/// The glyph and distance stay put; a road name too long for the space left
/// scrolls with @p scrollStep. Empty when not guiding.
auto formatNavigationLine(const projection::NavigationGuidance& guidance,
                          const IkeGlyphs& glyphs = {}, int scrollStep = 0) -> QByteArray;

/// "Artist - Song" (or whichever part exists). May be longer than 20; use
/// scrollWindow() to show it. Empty when nothing is playing.
auto formatMediaLine(const projection::NowPlaying& nowPlaying, const IkeGlyphs& glyphs = {})
    -> QByteArray;

/// 20-byte window of @p text for scroll step @p step (one byte per step).
/// Text that fits is returned unchanged; longer text loops with a 3-space gap.
auto scrollWindow(const QByteArray& text, int step, int width = kIkeTextWidth) -> QByteArray;

/// Generic IBus frame: SRC LEN DST DATA... XOR
auto ibusFrame(quint8 source, quint8 destination, const QByteArray& data) -> QByteArray;

/// Steady cluster text line (as a telephone title, like BlueBus' CD53 mode).
auto ikeTelephoneTextFrame(const QByteArray& text) -> QByteArray;

/// Check Control style text (20 chars, padded), persists until cleared.
auto ikeCheckControlTextFrame(const QByteArray& text) -> QByteArray;
auto ikeCheckControlClearFrame() -> QByteArray;

/// Hex dump "C8 07 80 23 ..." for logs / the UI preview.
auto toHex(const QByteArray& frame) -> QString;

}  // namespace crankshaft::ibus
