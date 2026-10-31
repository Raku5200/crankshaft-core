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

#include "NavigationClusterChannel.h"

#include <aap_protobuf/service/control/ControlMessageType.pb.h>
#include <aasdk/Common/Data.hpp>
#include <aasdk/Messenger/MessageId.hpp>
#include <aasdk/Messenger/Promise.hpp>

namespace crankshaft::android_auto {

namespace {
// aap_protobuf.service.navigationstatus.NavigationStatusMessageId
constexpr int kNavigationStatus = 32771;
constexpr int kTurnEvent = 32772;
constexpr int kDistanceEvent = 32773;
[[maybe_unused]] constexpr int kNavigationState = 32774;
[[maybe_unused]] constexpr int kCurrentPosition = 32775;
}  // namespace

NavigationClusterChannel::NavigationClusterChannel(boost::asio::io_service::strand& strand,
                                                   aasdk::messenger::IMessenger::Pointer messenger)
    : Channel(strand, std::move(messenger), aasdk::messenger::ChannelId::NAVIGATION_STATUS) {}

void NavigationClusterChannel::receive(INavigationClusterEventHandler::Pointer eventHandler) {
  auto receivePromise = aasdk::messenger::ReceivePromise::defer(strand_);
  receivePromise->then(
      std::bind(&NavigationClusterChannel::messageHandler, this->shared_from_this(),
                std::placeholders::_1, eventHandler),
      std::bind(&INavigationClusterEventHandler::onChannelError, eventHandler,
                std::placeholders::_1));
  messenger_->enqueueReceive(channelId_, std::move(receivePromise));
}

void NavigationClusterChannel::sendChannelOpenResponse(
    const aap_protobuf::service::control::message::ChannelOpenResponse& response,
    aasdk::channel::SendPromise::Pointer promise) {
  auto message = std::make_shared<aasdk::messenger::Message>(
      channelId_, aasdk::messenger::EncryptionType::ENCRYPTED, aasdk::messenger::MessageType::CONTROL);
  message->insertPayload(
      aasdk::messenger::MessageId(
          aap_protobuf::service::control::message::ControlMessageType::MESSAGE_CHANNEL_OPEN_RESPONSE)
          .getData());
  message->insertPayload(response);
  this->send(std::move(message), std::move(promise));
}

void NavigationClusterChannel::messageHandler(aasdk::messenger::Message::Pointer message,
                                              INavigationClusterEventHandler::Pointer eventHandler) {
  aasdk::messenger::MessageId messageId(message->getPayload());
  aasdk::common::DataConstBuffer payload(message->getPayload(), messageId.getSizeOf());
  const int id = static_cast<int>(messageId.getId());

  // Parse failures are reported as "unhandled" rather than as a channel
  // error: a malformed cluster hint must never take projection down.
  auto parseAnd = [&](auto&& proto, auto&& deliver) {
    if (proto.ParseFromArray(payload.cdata, static_cast<int>(payload.size))) {
      deliver(proto);
    } else {
      eventHandler->onUnhandledMessage(id, static_cast<int>(payload.size));
    }
  };

  if (id == aap_protobuf::service::control::message::ControlMessageType::MESSAGE_CHANNEL_OPEN_REQUEST) {
    aap_protobuf::service::control::message::ChannelOpenRequest request;
    if (request.ParseFromArray(payload.cdata, static_cast<int>(payload.size))) {
      eventHandler->onChannelOpenRequest(request);
    } else {
      eventHandler->onChannelError(aasdk::error::Error(aasdk::error::ErrorCode::PARSE_PAYLOAD));
    }
    return;  // the handler re-arms after answering
  }

  switch (id) {
    case kNavigationStatus:
      parseAnd(aap_protobuf::service::navigationstatus::message::NavigationStatus{},
               [&](const auto& m) { eventHandler->onStatusUpdate(m); });
      break;
    case kTurnEvent:
      parseAnd(aap_protobuf::service::navigationstatus::message::NavigationNextTurnEvent{},
               [&](const auto& m) { eventHandler->onTurnEvent(m); });
      break;
    case kDistanceEvent:
      parseAnd(aap_protobuf::service::navigationstatus::message::NavigationNextTurnDistanceEvent{},
               [&](const auto& m) { eventHandler->onDistanceEvent(m); });
      break;
#if CRANKSHAFT_AA_NAVIGATION_STATE
    case kNavigationState:
      parseAnd(aap_protobuf::service::navigationstatus::message::NavigationState{},
               [&](const auto& m) { eventHandler->onNavigationState(m); });
      break;
    case kCurrentPosition:
      parseAnd(aap_protobuf::service::navigationstatus::message::NavigationCurrentPosition{},
               [&](const auto& m) { eventHandler->onCurrentPosition(m); });
      break;
#endif
    default:
      eventHandler->onUnhandledMessage(id, static_cast<int>(payload.size));
      break;
  }
  this->receive(std::move(eventHandler));
}

}  // namespace crankshaft::android_auto
