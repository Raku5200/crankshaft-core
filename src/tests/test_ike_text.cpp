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

#include "../services/android_auto/ProjectionInfo.h"
#include "../services/ibus/IkeText.h"

using namespace crankshaft::ibus;
using namespace crankshaft::projection;

namespace {
auto guiding(Maneuver m, const QString& distance, const QString& road, int exit = 0)
    -> NavigationGuidance {
  NavigationGuidance g;
  g.status = NavigationGuidance::Status::Active;
  g.maneuver = m;
  g.distanceText = distance;
  g.road = road;
  g.roundaboutExit = exit;
  return g;
}
}  // namespace

TEST(IkeTextTest, ChecksumMatchesRealCheckControlFrame) {
  // From wilhelm-docs lcm/1a: "  WASHER FLUID LOW  " with display 0x36, options 0x03
  QByteArray data("\x1A\x36\x03", 3);
  data.append("  WASHER FLUID LOW  ");
  const QByteArray frame = ibusFrame(0x30, 0x80, data);
  EXPECT_EQ(toHex(frame),
            "30 19 80 1A 36 03 20 20 57 41 53 48 45 52 20 46 4C 55 49 44 20 4C 4F 57 20 20 9A");
}

TEST(IkeTextTest, CheckControlClearFrame) {
  EXPECT_EQ(toHex(ikeCheckControlClearFrame()), "60 05 80 1A 30 00 CF");
}

TEST(IkeTextTest, CheckControlTextIsPaddedTo20) {
  const QByteArray frame = ikeCheckControlTextFrame("> 300m Galle Rd");
  ASSERT_EQ(frame.size(), 4 + 3 + 20);
  EXPECT_EQ(static_cast<quint8>(frame[1]), 0x19);
  EXPECT_EQ(frame.mid(6, 20), QByteArray("> 300m Galle Rd     "));
}

TEST(IkeTextTest, TelephoneTextFrameLayout) {
  const QByteArray frame = ikeTelephoneTextFrame("Hello");
  EXPECT_EQ(toHex(frame.left(6)), "C8 0A 80 23 42 32");
  EXPECT_EQ(frame.mid(6, 5), QByteArray("Hello"));
}

TEST(IkeTextTest, NavigationLineRightTurn) {
  EXPECT_EQ(formatNavigationLine(guiding(Maneuver::Right, "300 m", "Galle Road")),
            QByteArray("> 300m Galle Rd"));
}

TEST(IkeTextTest, NavigationLineRoundaboutWithExitAndTruncation) {
  const QByteArray line =
      formatNavigationLine(guiding(Maneuver::Roundabout, "1.2 km", "Sri Jayawardenepura Mawatha", 2));
  EXPECT_EQ(line.size(), 20);
  EXPECT_TRUE(line.startsWith("O2 1.2km Sri Jaya"));
}

TEST(IkeTextTest, NavigationLineUsesConfiguredGlyph) {
  IkeGlyphs glyphs;
  glyphs.left = parseGlyphSpec(QStringLiteral("\\xCA"));
  const QByteArray line = formatNavigationLine(guiding(Maneuver::Left, "50 m", "Duplication Rd"), glyphs);
  EXPECT_EQ(static_cast<quint8>(line[0]), 0xCA);
  EXPECT_EQ(line.mid(1), QByteArray(" 50m Duplication Rd"));
}

TEST(IkeTextTest, NavigationLineEmptyWhenNotGuidingAndReroutingText) {
  NavigationGuidance g = guiding(Maneuver::Left, "50 m", "X");
  g.status = NavigationGuidance::Status::Inactive;
  EXPECT_TRUE(formatNavigationLine(g).isEmpty());
  g.status = NavigationGuidance::Status::Rerouting;
  EXPECT_EQ(formatNavigationLine(g), QByteArray("Rerouting..."));
}

TEST(IkeTextTest, MediaLineFoldsAccentsAndScrolls) {
  NowPlaying np;
  np.artist = QStringLiteral("Beyoncé");
  np.song = QStringLiteral("Café “Night” Drive");
  const QByteArray line = formatMediaLine(np);
  EXPECT_EQ(line, QByteArray("Beyonce - Cafe \"Night\" Drive"));
  EXPECT_EQ(scrollWindow(line, 0), line.left(20));
  EXPECT_EQ(scrollWindow(line, 10), QByteArray("Cafe \"Night\" Drive  "));
  EXPECT_EQ(scrollWindow(QByteArray("short"), 7), QByteArray("short"));
}

TEST(IkeTextTest, GlyphSpecParsing) {
  EXPECT_EQ(parseGlyphSpec(QStringLiteral("<")), QByteArray("<"));
  EXPECT_EQ(parseGlyphSpec(QStringLiteral("\\xAD\\xAD")), QByteArray("\xAD\xAD"));
  EXPECT_EQ(parseGlyphSpec(QStringLiteral("a\\x4")), QByteArray("a\\x4"));
}

TEST(ProjectionInfoTest, ManeuverMapping) {
  EXPECT_EQ(maneuverFromNavigationType(8), Maneuver::Right);
  EXPECT_EQ(maneuverFromNavigationType(34), Maneuver::Roundabout);
  EXPECT_EQ(maneuverFromNavigationType(41), Maneuver::DestinationLeft);
  EXPECT_EQ(maneuverFromTurnEvent(4, 1), Maneuver::Left);
  EXPECT_EQ(maneuverFromTurnEvent(3, 2), Maneuver::SlightRight);
  EXPECT_EQ(maneuverFromTurnEvent(13, 3), Maneuver::Roundabout);
}

TEST(ProjectionInfoTest, DistanceFormatting) {
  EXPECT_EQ(formatDistance(300, 300000, 1), QStringLiteral("300 m"));
  EXPECT_EQ(formatDistance(1234, 1200, 3), QStringLiteral("1.2 km"));
  EXPECT_EQ(formatDistance(16000, 16000, 2), QStringLiteral("16 km"));
  EXPECT_EQ(formatDistance(847, 0, 0), QStringLiteral("850 m"));
  EXPECT_EQ(formatDistanceText(QStringLiteral("0.5"), 5, 800), QStringLiteral("0.5 mi"));
  EXPECT_EQ(compactDistance(QStringLiteral("1.2 km")), QStringLiteral("1.2km"));
}

TEST(ProjectionInfoTest, RoadAbbreviation) {
  EXPECT_EQ(abbreviateRoad(QStringLiteral("Baseline Mawatha")), QStringLiteral("Baseline Mw"));
  EXPECT_EQ(abbreviateRoad(QStringLiteral("Southern Expressway")), QStringLiteral("Southern Expy"));
  EXPECT_EQ(abbreviateRoad(QStringLiteral("Roadside Lane")), QStringLiteral("Roadside Ln"));
}
