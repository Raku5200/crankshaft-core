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

// Plain data describing what the phone reports over the Android Auto
// "media playback status" and "navigation status" (instrument cluster)
// channels. No aasdk/protobuf types here so it can be unit tested and used by
// the IBus cluster formatter.

#include <QString>
#include <QVariantMap>

namespace crankshaft::projection {

enum class Maneuver {
  Unknown,
  Depart,
  NameChange,
  Straight,
  SlightLeft,
  SlightRight,
  Left,
  Right,
  SharpLeft,
  SharpRight,
  UTurnLeft,
  UTurnRight,
  KeepLeft,
  KeepRight,
  RampLeft,
  RampRight,
  ExitLeft,
  ExitRight,
  ForkLeft,
  ForkRight,
  MergeLeft,
  MergeRight,
  Merge,
  RoundaboutEnter,
  RoundaboutExit,
  Roundabout,  // enter and take exit N
  Ferry,
  Destination,
  DestinationLeft,
  DestinationRight,
};

struct NowPlaying {
  QString song;
  QString artist;
  QString album;
  bool playing = false;
  int positionSeconds = -1;
  int durationSeconds = 0;

  [[nodiscard]] auto hasTrack() const -> bool { return !song.isEmpty() || !artist.isEmpty(); }
};

struct NavigationGuidance {
  enum class Status { Unavailable, Active, Inactive, Rerouting };

  Status status = Status::Unavailable;
  Maneuver maneuver = Maneuver::Unknown;
  int roundaboutExit = 0;
  QString road;               // road of the next maneuver
  int distanceMeters = -1;    // to the next maneuver
  QString distanceText;       // as the phone wants it shown, e.g. "300 m", "1.2 km"
  int timeToTurnSeconds = -1;
  QString currentRoad;
  QString eta;                // e.g. "4:05 PM" (as sent by the phone)
  QString destinationDistanceText;
  int timeToArrivalSeconds = -1;

  [[nodiscard]] auto isGuiding() const -> bool {
    return status == Status::Active || status == Status::Rerouting;
  }
};

/// aap NavigationManeuver.NavigationType (NavigationState protocol) -> Maneuver.
auto maneuverFromNavigationType(int navigationType) -> Maneuver;

/// Deprecated NavigationNextTurnEvent (NextTurnEnum + TurnSide) -> Maneuver.
/// turnSide: 1 = LEFT, 2 = RIGHT, 3 = UNSPECIFIED.
auto maneuverFromTurnEvent(int nextTurnEnum, int turnSide) -> Maneuver;

/// Human readable distance from the phone's display hints.
/// unit follows aap DistanceUnits: 1 m, 2 km, 3 km (1 dp), 4 mi, 5 mi (1 dp), 6 ft, 7 yd.
/// displayE3 is the display value * 1000; when <= 0 the meters value is used.
auto formatDistance(int meters, int displayE3, int unit) -> QString;

/// Same, for NavigationDistance which already carries the number as text.
auto formatDistanceText(const QString& displayValue, int unit, int meters) -> QString;

auto maneuverName(Maneuver maneuver) -> QString;
auto statusName(NavigationGuidance::Status status) -> QString;

auto toVariantMap(const NowPlaying& nowPlaying) -> QVariantMap;
auto toVariantMap(const NavigationGuidance& guidance) -> QVariantMap;

}  // namespace crankshaft::projection
