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

// Android Auto "navigation status" (instrument cluster) channel.
//
// aasdk ships a NavigationStatusService, but it only understands the
// deprecated turn/distance events and drops the newer NavigationState and
// NavigationCurrentPosition messages (road names, maneuvers, ETA) that
// current Google Maps sends. This channel handles all of them.

#include <aap_protobuf/service/control/message/ChannelOpenRequest.pb.h>
#include <aap_protobuf/service/control/message/ChannelOpenResponse.pb.h>
#include <aap_protobuf/service/navigationstatus/message/NavigationNextTurnDistanceEvent.pb.h>
#include <aap_protobuf/service/navigationstatus/message/NavigationNextTurnEvent.pb.h>
#include <aap_protobuf/service/navigationstatus/message/NavigationStatus.pb.h>

#include <aasdk/Channel/Channel.hpp>
#include <aasdk/Channel/Promise.hpp>
#include <aasdk/Error/Error.hpp>
#include <aasdk/Messenger/Message.hpp>
#include <memory>

#if __has_include(<aap_protobuf/service/navigationstatus/message/NavigationState.pb.h>) && \
    __has_include(<aap_protobuf/service/navigationstatus/message/NavigationCurrentPosition.pb.h>)
#include <aap_protobuf/service/navigationstatus/message/NavigationCurrentPosition.pb.h>
#include <aap_protobuf/service/navigationstatus/message/NavigationState.pb.h>
#define CRANKSHAFT_AA_NAVIGATION_STATE 1
#else
#define CRANKSHAFT_AA_NAVIGATION_STATE 0
#endif

namespace crankshaft::android_auto {

class INavigationClusterEventHandler {
 public:
  using Pointer = std::shared_ptr<INavigationClusterEventHandler>;
  virtual ~INavigationClusterEventHandler() = default;

  virtual void onChannelOpenRequest(
      const aap_protobuf::service::control::message::ChannelOpenRequest& request) = 0;
  virtual void onChannelError(const aasdk::error::Error& error) = 0;
  virtual void onStatusUpdate(
      const aap_protobuf::service::navigationstatus::message::NavigationStatus& status) = 0;
  virtual void onTurnEvent(
      const aap_protobuf::service::navigationstatus::message::NavigationNextTurnEvent& event) = 0;
  virtual void onDistanceEvent(
      const aap_protobuf::service::navigationstatus::message::NavigationNextTurnDistanceEvent&
          event) = 0;
#if CRANKSHAFT_AA_NAVIGATION_STATE
  virtual void onNavigationState(
      const aap_protobuf::service::navigationstatus::message::NavigationState& state) = 0;
  virtual void onCurrentPosition(
      const aap_protobuf::service::navigationstatus::message::NavigationCurrentPosition&
          position) = 0;
#endif
  /// Any message that was not understood; the channel is re-armed by the caller.
  virtual void onUnhandledMessage(int messageId, int payloadSize) = 0;
};

class NavigationClusterChannel : public aasdk::channel::Channel,
                                 public std::enable_shared_from_this<NavigationClusterChannel> {
 public:
  using Pointer = std::shared_ptr<NavigationClusterChannel>;

  NavigationClusterChannel(boost::asio::io_service::strand& strand,
                           aasdk::messenger::IMessenger::Pointer messenger);

  void receive(INavigationClusterEventHandler::Pointer eventHandler);
  void sendChannelOpenResponse(
      const aap_protobuf::service::control::message::ChannelOpenResponse& response,
      aasdk::channel::SendPromise::Pointer promise);

 private:
  void messageHandler(aasdk::messenger::Message::Pointer message,
                      INavigationClusterEventHandler::Pointer eventHandler);
};

}  // namespace crankshaft::android_auto
