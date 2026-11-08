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

#include "IkeText.h"

#include <QRegularExpression>
#include <QStringList>

namespace crankshaft::ibus {

using projection::Maneuver;

auto parseGlyphSpec(const QString& spec) -> QByteArray {
  QByteArray out;
  for (int i = 0; i < spec.size(); ++i) {
    if (spec[i] == QLatin1Char('\\') && i + 3 < spec.size() &&
        (spec[i + 1] == QLatin1Char('x') || spec[i + 1] == QLatin1Char('X'))) {
      bool ok = false;
      const int value = spec.mid(i + 2, 2).toInt(&ok, 16);
      if (ok) {
        out.append(static_cast<char>(value));
        i += 3;
        continue;
      }
    }
    const QChar c = spec[i];
    if (c.unicode() < 0x80) {
      out.append(static_cast<char>(c.unicode()));
    }
  }
  return out;
}

auto toIkeCharset(const QString& text) -> QByteArray {
  const QString decomposed = text.normalized(QString::NormalizationForm_KD);
  QByteArray out;
  out.reserve(decomposed.size());
  for (const QChar c : decomposed) {
    const char16_t u = c.unicode();
    if (u >= 0x20 && u < 0x7F) {
      out.append(static_cast<char>(u));
    } else if (u == 0x2018 || u == 0x2019) {
      out.append('\'');
    } else if (u == 0x201C || u == 0x201D) {
      out.append('"');
    } else if (u == 0x2013 || u == 0x2014) {
      out.append('-');
    } else if (u == 0x00DF) {
      out.append("ss");
    } else if (c.isSpace()) {
      out.append(' ');
    }
    // combining marks (from the decomposition) and other symbols are dropped
  }
  return out.simplified();
}

auto abbreviateRoad(const QString& road) -> QString {
  static const QList<QPair<QString, QString>> kWords = {
      {QStringLiteral("Road"), QStringLiteral("Rd")},
      {QStringLiteral("Street"), QStringLiteral("St")},
      {QStringLiteral("Avenue"), QStringLiteral("Ave")},
      {QStringLiteral("Highway"), QStringLiteral("Hwy")},
      {QStringLiteral("Expressway"), QStringLiteral("Expy")},
      {QStringLiteral("Lane"), QStringLiteral("Ln")},
      {QStringLiteral("Drive"), QStringLiteral("Dr")},
      {QStringLiteral("Place"), QStringLiteral("Pl")},
      {QStringLiteral("Boulevard"), QStringLiteral("Blvd")},
      {QStringLiteral("Mawatha"), QStringLiteral("Mw")},
      {QStringLiteral("Junction"), QStringLiteral("Jn")},
      {QStringLiteral("North"), QStringLiteral("N")},
      {QStringLiteral("South"), QStringLiteral("S")},
      {QStringLiteral("East"), QStringLiteral("E")},
      {QStringLiteral("West"), QStringLiteral("W")},
  };
  QString result = road.simplified();
  for (const auto& [word, abbreviation] : kWords) {
    result.replace(QRegularExpression(QStringLiteral("\\b%1\\b").arg(word),
                                      QRegularExpression::CaseInsensitiveOption),
                   abbreviation);
  }
  return result;
}

auto compactDistance(const QString& distanceText) -> QString {
  QString compact = distanceText.simplified();
  compact.remove(QLatin1Char(' '));
  return compact;
}

auto maneuverGlyph(Maneuver maneuver, int roundaboutExit, const IkeGlyphs& g) -> QByteArray {
  switch (maneuver) {
    case Maneuver::Left:
    case Maneuver::RampLeft:
    case Maneuver::ExitLeft:
      return g.left;
    case Maneuver::Right:
    case Maneuver::RampRight:
    case Maneuver::ExitRight:
      return g.right;
    case Maneuver::SlightLeft: return g.slightLeft;
    case Maneuver::SlightRight: return g.slightRight;
    case Maneuver::SharpLeft: return g.sharpLeft;
    case Maneuver::SharpRight: return g.sharpRight;
    case Maneuver::KeepLeft: return g.keepLeft;
    case Maneuver::KeepRight: return g.keepRight;
    case Maneuver::ForkLeft: return g.fork + g.left;
    case Maneuver::ForkRight: return g.fork + g.right;
    case Maneuver::MergeLeft: return g.merge + g.left;
    case Maneuver::MergeRight: return g.merge + g.right;
    case Maneuver::Merge: return g.merge;
    case Maneuver::UTurnLeft:
    case Maneuver::UTurnRight:
      return g.uTurn;
    case Maneuver::RoundaboutEnter:
    case Maneuver::RoundaboutExit:
    case Maneuver::Roundabout:
      return roundaboutExit > 0 ? g.roundabout + QByteArray::number(roundaboutExit) : g.roundabout;
    case Maneuver::Destination: return g.destination;
    case Maneuver::DestinationLeft: return g.destination + g.left;
    case Maneuver::DestinationRight: return g.destination + g.right;
    case Maneuver::Ferry: return g.ferry;
    case Maneuver::Straight:
    case Maneuver::Depart:
    case Maneuver::NameChange:
      return g.straight;
    case Maneuver::Unknown:
      break;
  }
  return {};
}

auto formatNavigationParts(const projection::NavigationGuidance& guidance,
                           const IkeGlyphs& glyphs) -> NavigationLineParts {
  using Status = projection::NavigationGuidance::Status;
  NavigationLineParts parts;
  if (guidance.status == Status::Rerouting) {
    parts.road = "Rerouting...";
    return parts;
  }
  if (guidance.status != Status::Active) {
    return parts;
  }

  parts.prefix = maneuverGlyph(guidance.maneuver, guidance.roundaboutExit, glyphs);
  const QByteArray distance = toIkeCharset(compactDistance(guidance.distanceText));
  if (!distance.isEmpty()) {
    if (!parts.prefix.isEmpty()) parts.prefix.append(' ');
    parts.prefix.append(distance);
  }
  parts.prefix = parts.prefix.left(kIkeTextWidth);

  const QString roadSource = guidance.road.isEmpty() ? guidance.currentRoad : guidance.road;
  parts.road = toIkeCharset(abbreviateRoad(roadSource));
  if (!parts.road.isEmpty() && !parts.prefix.isEmpty() && parts.prefix.size() < kIkeTextWidth) {
    parts.prefix.append(' ');
  }
  return parts;
}

auto formatNavigationLine(const projection::NavigationGuidance& guidance, const IkeGlyphs& glyphs,
                          int scrollStep) -> QByteArray {
  const NavigationLineParts parts = formatNavigationParts(guidance, glyphs);
  QByteArray line = parts.prefix;
  if (!parts.road.isEmpty() && parts.roadWidth() > 0) {
    line.append(scrollWindow(parts.road, scrollStep, parts.roadWidth()));
  }
  // Left-aligned and always exactly 20 characters: a shorter line would be
  // centred by the cluster, and the arrow and distance would jump around
  // whenever the scrolling road name passes its gap.
  line = line.left(kIkeTextWidth);
  if (line.trimmed().isEmpty()) {
    return {};
  }
  return line.leftJustified(kIkeTextWidth, ' ');
}

auto formatMediaLine(const projection::NowPlaying& nowPlaying, const IkeGlyphs& glyphs)
    -> QByteArray {
  if (!nowPlaying.hasTrack()) {
    return {};
  }
  QByteArray line = glyphs.music;
  if (!line.isEmpty()) line.append(' ');
  // Song first: it is what changes, and it is what shows before the cluster
  // starts scrolling the rest.
  const QByteArray artist = toIkeCharset(nowPlaying.artist);
  const QByteArray song = toIkeCharset(nowPlaying.song);
  if (!artist.isEmpty() && !song.isEmpty()) {
    line.append(song + " - " + artist);
  } else {
    line.append(song.isEmpty() ? artist : song);
  }
  return line;
}

auto scrollWindow(const QByteArray& text, int step, int width) -> QByteArray {
  if (text.size() <= width) {
    return text;
  }
  const QByteArray loop = text + QByteArray("   ");
  const int offset = ((step % loop.size()) + loop.size()) % loop.size();
  const QByteArray doubled = loop + loop;
  return doubled.mid(offset, width);
}

auto ibusFrame(quint8 source, quint8 destination, const QByteArray& data) -> QByteArray {
  QByteArray frame;
  frame.reserve(data.size() + 4);
  frame.append(static_cast<char>(source));
  frame.append(static_cast<char>(data.size() + 2));  // destination + data + checksum
  frame.append(static_cast<char>(destination));
  frame.append(data);
  quint8 checksum = 0;
  for (const char byte : frame) {
    checksum ^= static_cast<quint8>(byte);
  }
  frame.append(static_cast<char>(checksum));
  return frame;
}

auto ikeTelephoneTextFrame(const QByteArray& text) -> QByteArray {
  QByteArray data("\x23\x42\x32", 3);
  data.append(text.left(kIkeTextWidth));
  return ibusFrame(kAddrTelephone, kAddrIke, data);
}

auto ikeCheckControlTextFrame(const QByteArray& text) -> QByteArray {
  QByteArray data("\x1A\x36\x00", 3);
  QByteArray padded = text.left(kIkeTextWidth);
  padded.append(QByteArray(kIkeTextWidth - padded.size(), ' '));
  data.append(padded);
  return ibusFrame(kAddrPdc, kAddrIke, data);
}

auto ikeCheckControlClearFrame() -> QByteArray {
  return ibusFrame(kAddrPdc, kAddrIke, QByteArray("\x1A\x30\x00", 3));
}

auto toHex(const QByteArray& frame) -> QString {
  return QString::fromLatin1(frame.toHex(' ')).toUpper();
}

}  // namespace crankshaft::ibus
