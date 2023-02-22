#ifdef ARDUINO_ARCH_RP2040

#include "Communications.h"
#include "SPISlave.h"

constexpr static uint32_t timeout = 200;
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

  auto update_last_communication = [this](Packet p) {
    if (p.header.device == KEYSCANNER_DEFY_RIGHT) {
      right.lastCommunication = millis();
    }
    if (p.header.device == KEYSCANNER_DEFY_LEFT) {
      left.lastCommunication = millis();
    }
  };

  callbacks.bind(IS_ALIVE, update_last_communication);
  callbacks.bind(HAS_KEYS, update_last_communication);

  callbacks.bind(CONNECTED, [this](Packet p) {
    if (p.header.device == KEYSCANNER_DEFY_RIGHT) {
      right.lastCommunication = millis();
      right.online            = true;
      if (port0.device == KEYSCANNER_DEFY_RIGHT) right.port = false;
      if (port1.device == KEYSCANNER_DEFY_RIGHT) right.port = true;
    }
    if (p.header.device == KEYSCANNER_DEFY_LEFT) {
      left.lastCommunication = millis();
      left.online            = true;
      if (port0.device == KEYSCANNER_DEFY_LEFT) left.port = false;
      if (port1.device == KEYSCANNER_DEFY_LEFT) left.port = true;
    }
  });
}

void Communications::run() {


  if (!queue_is_empty(&port0.rx_messages_)) {
    Packet packet;
    queue_remove_blocking(&port0.rx_messages_, &packet);
    callbacks.call(packet.header.command, packet);
  }

  if (!queue_is_empty(&port1.rx_messages_)) {
    Packet packet;
    queue_remove_blocking(&port1.rx_messages_, &packet);
    callbacks.call(packet.header.command, packet);
  }

  checkActive(left);
  checkActive(right);
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  packet.header.device   = Communications_protocol::NEURON_DEFY_WIRED;
  if (device_to_send == Communications_protocol::UNKNOWN) {
    if (port0.device != UNKNOWN)
      queue_add_blocking(&port0.tx_messages_, &packet);
    if (port1.device != UNKNOWN)
      queue_add_blocking(&port1.tx_messages_, &packet);
  }
  if (device_to_send == Communications_protocol::KEYSCANNER_DEFY_LEFT) {
    if (left.port) {
      if (port1.device != UNKNOWN)
        queue_add_blocking(&port1.tx_messages_, &packet);
    } else {
      if (port0.device != UNKNOWN)
        queue_add_blocking(&port0.tx_messages_, &packet);
    }
  }
  if (device_to_send == Communications_protocol::KEYSCANNER_DEFY_RIGHT) {
    if (right.port) {
      if (port1.device != UNKNOWN)
        queue_add_blocking(&port1.tx_messages_, &packet);
    } else {
      if (port0.device != UNKNOWN)
        queue_add_blocking(&port0.tx_messages_, &packet);
    }
  }
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