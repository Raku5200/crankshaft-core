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

#include "ProjectionInfo.h"

namespace crankshaft::projection {

auto maneuverFromNavigationType(int type) -> Maneuver {
  switch (type) {
    case 1: return Maneuver::Depart;
    case 2: return Maneuver::NameChange;
    case 3: return Maneuver::KeepLeft;
    case 4: return Maneuver::KeepRight;
    case 5: return Maneuver::SlightLeft;
    case 6: return Maneuver::SlightRight;
    case 7: return Maneuver::Left;
    case 8: return Maneuver::Right;
    case 9: return Maneuver::SharpLeft;
    case 10: return Maneuver::SharpRight;
    case 11: return Maneuver::UTurnLeft;
    case 12: return Maneuver::UTurnRight;
    case 13: case 15: case 17: case 19: return Maneuver::RampLeft;
    case 14: case 16: case 18: case 20: return Maneuver::RampRight;
    case 21: case 23: return Maneuver::ExitLeft;
    case 22: case 24: return Maneuver::ExitRight;
    case 25: return Maneuver::ForkLeft;
    case 26: return Maneuver::ForkRight;
    case 27: return Maneuver::MergeLeft;
    case 28: return Maneuver::MergeRight;
    case 29: return Maneuver::Merge;
    case 30: return Maneuver::RoundaboutEnter;
    case 31: return Maneuver::RoundaboutExit;
    case 32: case 33: case 34: case 35: return Maneuver::Roundabout;
    case 36: return Maneuver::Straight;
    case 37: case 38: return Maneuver::Ferry;
    case 39: case 40: return Maneuver::Destination;
    case 41: return Maneuver::DestinationLeft;
    case 42: return Maneuver::DestinationRight;
    default: return Maneuver::Unknown;
  }
}

auto maneuverFromTurnEvent(int nextTurnEnum, int turnSide) -> Maneuver {
  const bool left = turnSide == 1;
  const bool right = turnSide == 2;
  auto sided = [&](Maneuver l, Maneuver r, Maneuver none) {
    return left ? l : (right ? r : none);
  };

  switch (nextTurnEnum) {
    case 1: return Maneuver::Depart;
    case 2: return Maneuver::NameChange;
    case 3: return sided(Maneuver::SlightLeft, Maneuver::SlightRight, Maneuver::Straight);
    case 4: return sided(Maneuver::Left, Maneuver::Right, Maneuver::Unknown);
    case 5: return sided(Maneuver::SharpLeft, Maneuver::SharpRight, Maneuver::Unknown);
    case 6: return right ? Maneuver::UTurnRight : Maneuver::UTurnLeft;
    case 7: return sided(Maneuver::RampLeft, Maneuver::RampRight, Maneuver::RampRight);
    case 8: return sided(Maneuver::ExitLeft, Maneuver::ExitRight, Maneuver::ExitRight);
    case 9: return sided(Maneuver::ForkLeft, Maneuver::ForkRight, Maneuver::Unknown);
    case 10: return sided(Maneuver::MergeLeft, Maneuver::MergeRight, Maneuver::Merge);
    case 11: return Maneuver::RoundaboutEnter;
    case 12: return Maneuver::RoundaboutExit;
    case 13: return Maneuver::Roundabout;
    case 14: return Maneuver::Straight;
    case 16: case 17: return Maneuver::Ferry;
    case 19: return sided(Maneuver::DestinationLeft, Maneuver::DestinationRight,
                          Maneuver::Destination);
    default: return Maneuver::Unknown;
  }
}

namespace {

auto unitSuffix(int unit) -> QString {
  switch (unit) {
    case 1: return QStringLiteral("m");
    case 2: case 3: return QStringLiteral("km");
    case 4: case 5: return QStringLiteral("mi");
    case 6: return QStringLiteral("ft");
    case 7: return QStringLiteral("yd");
    default: return {};
  }
}

auto metersFallback(int meters) -> QString {
  if (meters < 0) {
    return {};
  }
  if (meters < 1000) {
    // Round like the phone does: 10 m steps below 1 km.
    const int rounded = meters < 100 ? meters : (meters + 5) / 10 * 10;
    return QStringLiteral("%1 m").arg(rounded);
  }
  if (meters < 10000) {
    return QStringLiteral("%1 km").arg(QString::number(meters / 1000.0, 'f', 1));
  }
  return QStringLiteral("%1 km").arg(qRound(meters / 1000.0));
}

}  // namespace

auto formatDistance(int meters, int displayE3, int unit) -> QString {
  const QString suffix = unitSuffix(unit);
  if (displayE3 <= 0 || suffix.isEmpty()) {
    return metersFallback(meters);
  }
  const double value = displayE3 / 1000.0;
  const bool oneDecimal = unit == 3 || unit == 5 || (value < 10.0 && value != qRound(value));
  const QString number = oneDecimal ? QString::number(value, 'f', 1) : QString::number(qRound(value));
  return QStringLiteral("%1 %2").arg(number, suffix);
}

auto formatDistanceText(const QString& displayValue, int unit, int meters) -> QString {
  const QString suffix = unitSuffix(unit);
  if (displayValue.trimmed().isEmpty() || suffix.isEmpty()) {
    return metersFallback(meters);
  }
  return QStringLiteral("%1 %2").arg(displayValue.trimmed(), suffix);
}

auto maneuverName(Maneuver maneuver) -> QString {
  switch (maneuver) {
    case Maneuver::Unknown: return QStringLiteral("unknown");
    case Maneuver::Depart: return QStringLiteral("depart");
    case Maneuver::NameChange: return QStringLiteral("name-change");
    case Maneuver::Straight: return QStringLiteral("straight");
    case Maneuver::SlightLeft: return QStringLiteral("slight-left");
    case Maneuver::SlightRight: return QStringLiteral("slight-right");
    case Maneuver::Left: return QStringLiteral("left");
    case Maneuver::Right: return QStringLiteral("right");
    case Maneuver::SharpLeft: return QStringLiteral("sharp-left");
    case Maneuver::SharpRight: return QStringLiteral("sharp-right");
    case Maneuver::UTurnLeft: return QStringLiteral("u-turn-left");
    case Maneuver::UTurnRight: return QStringLiteral("u-turn-right");
    case Maneuver::KeepLeft: return QStringLiteral("keep-left");
    case Maneuver::KeepRight: return QStringLiteral("keep-right");
    case Maneuver::RampLeft: return QStringLiteral("ramp-left");
    case Maneuver::RampRight: return QStringLiteral("ramp-right");
    case Maneuver::ExitLeft: return QStringLiteral("exit-left");
    case Maneuver::ExitRight: return QStringLiteral("exit-right");
    case Maneuver::ForkLeft: return QStringLiteral("fork-left");
    case Maneuver::ForkRight: return QStringLiteral("fork-right");
    case Maneuver::MergeLeft: return QStringLiteral("merge-left");
    case Maneuver::MergeRight: return QStringLiteral("merge-right");
    case Maneuver::Merge: return QStringLiteral("merge");
    case Maneuver::RoundaboutEnter: return QStringLiteral("roundabout-enter");
    case Maneuver::RoundaboutExit: return QStringLiteral("roundabout-exit");
    case Maneuver::Roundabout: return QStringLiteral("roundabout");
    case Maneuver::Ferry: return QStringLiteral("ferry");
    case Maneuver::Destination: return QStringLiteral("destination");
    case Maneuver::DestinationLeft: return QStringLiteral("destination-left");
    case Maneuver::DestinationRight: return QStringLiteral("destination-right");
  }
  return QStringLiteral("unknown");
}

auto statusName(NavigationGuidance::Status status) -> QString {
  switch (status) {
    case NavigationGuidance::Status::Active: return QStringLiteral("active");
    case NavigationGuidance::Status::Inactive: return QStringLiteral("inactive");
    case NavigationGuidance::Status::Rerouting: return QStringLiteral("rerouting");
    case NavigationGuidance::Status::Unavailable: break;
  }
  return QStringLiteral("unavailable");
}

auto toVariantMap(const NowPlaying& nowPlaying) -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("song")] = nowPlaying.song;
  map[QStringLiteral("artist")] = nowPlaying.artist;
  map[QStringLiteral("album")] = nowPlaying.album;
  map[QStringLiteral("playing")] = nowPlaying.playing;
  map[QStringLiteral("positionSeconds")] = nowPlaying.positionSeconds;
  map[QStringLiteral("durationSeconds")] = nowPlaying.durationSeconds;
  map[QStringLiteral("hasTrack")] = nowPlaying.hasTrack();
  return map;
}

auto toVariantMap(const NavigationGuidance& guidance) -> QVariantMap {
  QVariantMap map;
  map[QStringLiteral("status")] = statusName(guidance.status);
  map[QStringLiteral("guiding")] = guidance.isGuiding();
  map[QStringLiteral("maneuver")] = maneuverName(guidance.maneuver);
  map[QStringLiteral("roundaboutExit")] = guidance.roundaboutExit;
  map[QStringLiteral("road")] = guidance.road;
  map[QStringLiteral("distanceMeters")] = guidance.distanceMeters;
  map[QStringLiteral("distanceText")] = guidance.distanceText;
  map[QStringLiteral("timeToTurnSeconds")] = guidance.timeToTurnSeconds;
  map[QStringLiteral("currentRoad")] = guidance.currentRoad;
  map[QStringLiteral("eta")] = guidance.eta;
  map[QStringLiteral("destinationDistanceText")] = guidance.destinationDistanceText;
  map[QStringLiteral("timeToArrivalSeconds")] = guidance.timeToArrivalSeconds;
  return map;
}

}  // namespace crankshaft::projection
