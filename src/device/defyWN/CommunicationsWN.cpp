/*
 * Copyright (C) 2024  Dygma Lab S.L.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Gustavo Gomez Lopez @Noteolvides
 *
 */

#ifdef ARDUINO_ARCH_RP2040

#include "Communications.h"
#include "SPISlave.h"

constexpr static uint32_t timeout = 600;
struct SideInfo {
  SideInfo(Devices _devices)
    : device(_devices) {}
  Devices device;
  bool online{false};
  uint32_t lastCommunication{0};
  bool port{false};
};

static SideInfo left{Communications_protocol::KEYSCANNER_DEFY_LEFT};
static SideInfo right{Communications_protocol::KEYSCANNER_DEFY_RIGHT};

void checkActive(SideInfo &side);

void Communications::init() {
  port0.init();
  port1.init();

  callbacks.bind(CONNECTED, [this](Packet p) {
    p.header.size    = 0;
    p.header.device  = p.header.device;
    p.header.command = CONNECTED;
    sendPacket(p);
  });
}

void Communications::run() {


  if (!queue_is_empty(&port0.rx_messages_)) {
    Packet packet;
    queue_remove_blocking(&port0.rx_messages_, &packet);
    SideInfo &side         = packet.header.device == KEYSCANNER_DEFY_LEFT ? left : right;
    side.lastCommunication = millis();
    side.online            = true;
    side.port              = false;
    callbacks.call(packet.header.command, packet);
  }

  if (!queue_is_empty(&port1.rx_messages_)) {
    Packet packet;
    queue_remove_blocking(&port1.rx_messages_, &packet);
    SideInfo &side         = packet.header.device == KEYSCANNER_DEFY_LEFT ? left : right;
    side.lastCommunication = millis();
    side.online            = true;
    side.port              = true;
    callbacks.call(packet.header.command, packet);
  }

  checkActive(left);
  checkActive(right);
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  packet.header.device   = Communications_protocol::WIRED_NEURON_DEFY;
  if (device_to_send == Communications_protocol::UNKNOWN) {
    if (port0.device != UNKNOWN)
      queue_add_blocking(&port0.tx_messages_, &packet);
    if (port1.device != UNKNOWN)
      queue_add_blocking(&port1.tx_messages_, &packet);
  }
  if (device_to_send == port1.device)
    queue_add_blocking(&port1.tx_messages_, &packet);
  if (device_to_send == port0.device)
    queue_add_blocking(&port0.tx_messages_, &packet);

  return true;
}

void checkActive(SideInfo &side) {
  if (!side.online) return;
  const bool now_active = millis() - side.lastCommunication <= timeout;

  if (side.online && !now_active) {
    side.online   = now_active;
    SPISlave &spi = side.port ? port1 : port0;
    spi.device    = Communications_protocol::UNKNOWN;
    //Clear the packets as now the channel is no longer active
    Packet packet;
    while (!queue_is_empty(&spi.rx_messages_)) {
      queue_remove_blocking(&spi.rx_messages_, &packet);
    }
    while (!queue_is_empty(&spi.tx_messages_)) {
      queue_remove_blocking(&spi.tx_messages_, &packet);
    }
    return;
  }
}


class Communications Communications;

#endif
