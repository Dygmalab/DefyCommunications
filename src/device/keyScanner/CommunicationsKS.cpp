#ifdef KEYSCANNER
#include <string.h>
#include "SPI.hpp"
#include <Keyscanner.hpp>
#include <IS31FL3743B.hpp>
#include <LEDManagement.hpp>
#include "Communications.h"
#include "pico/util/queue.h"

constexpr uint8_t SIDE_ID = 25;
queue_t txMessages;
queue_t rxMessages;
Communications_protocol::Packet tx_message;
Communications_protocol::Packet rx_message;
Communications_protocol::Devices device;
bool need_polling;
uint32_t last_time_communication_with_neuron;
uint16_t keep_alive_timeout_neuron = 100;

class Communications Communications;

void Communications::run() {

  if (to_ms_since_boot(get_absolute_time()) - last_time_communication_with_neuron > keep_alive_timeout_neuron || need_polling || KeyScanner.newKey()) {
    last_time_communication_with_neuron = to_ms_since_boot(get_absolute_time());
    Packet packet{};
    packet.header.command = IS_ALIVE;
    if (KeyScanner.newKey()) {
      KeyScanner.keyState(false);
      packet.header.command = Communications_protocol::HAS_KEYS;
      packet.header.size    = KeyScanner.readMatrix(packet.data);
    }
    sendPacket(packet);
  }

  if (!queue_is_empty(&txMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);
    SPI::read_write_buffer(SPI::CSList::CSN2, tx_message.buf, rx_message.buf, sizeof(Packet));
    need_polling = rx_message.header.has_more_packets;
    callbacks.call(rx_message.header.command, rx_message);
  }
}

void Communications::init() {
  if (gpio_get(SIDE_ID)) {
    device = Communications_protocol::KEYSCANNER_DEFY_RIGHT;
  } else {
    device = Communications_protocol::KEYSCANNER_DEFY_LEFT;
  }

  auto empty_func = [](Packet p) {};
  callbacks.bind(IS_DEAD, empty_func);

  //TODO: Is alive
  callbacks.bind(IS_ALIVE, empty_func);

  callbacks.bind(SLEEP, [](Packet p) {
    IS31FL3743B::setEnabled(false);
    LEDManagement::set_enable_underGlow(false);
  });

  callbacks.bind(WAKE_UP, [](Packet p) {
    IS31FL3743B::setEnabled(true);
    LEDManagement::set_enable_underGlow(true);
    LEDManagement::set_updated(true);
  });

  //TODO: Get version
  callbacks.bind(GET_VERSION, empty_func);

  //TODO: Alive inverval
  callbacks.bind(SET_ALIVE_INTERVAL, empty_func);

  //TODO: Has Keys loopback
  callbacks.bind(HAS_KEYS, [this](Packet p) {
    p.header.device = device;
    sendPacket(p);
  });

  callbacks.bind(SET_KEYSCAN_INTERVAL, [](Packet p) {
    KeyScanner.keyScanInterval(p.data[0]);
  });

  callbacks.bind(GET_SHORT_LED, [this](Packet p) {
    p.header.device = device;
    p.header.size   = IS31FL3743B::get_short_leds(p.data);
    sendPacket(p);
  });

  callbacks.bind(GET_OPEN_LED, [this](Packet p) {
    p.header.device = device;
    p.header.size   = IS31FL3743B::get_open_leds(p.data);
    sendPacket(p);
  });

  callbacks.bind(SET_BRIGHTNESS, [](Packet p) {
    LEDManagement::setMaxBrightness(p.data[0]);
  });

  callbacks.bind(SET_MODE_LED, [](Packet p) {
    LEDManagement::set_led_mode(p.data);
  });

  //TODO: SET_LED
  callbacks.bind(SET_LED, empty_func);

  callbacks.bind(SET_PALETTE_COLORS, [](Packet p) {
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size);
  });

  callbacks.bind(SET_LAYER_KEYMAP_COLORS, [](Packet p) {
    uint8_t layerIndex = p.data[0];
    if (layerIndex < LEDManagement::layers.size()) {
      LEDManagement::layers.push_back({});
    }

    LEDManagement::Layer &layer = LEDManagement::layers.at(layerIndex);
    memcpy(layer.keyMap_leds, &p.data[1], p.header.size - 1);
  });

  callbacks.bind(SET_LAYER_UNDERGLOW_COLORS, [](Packet p) {
    uint8_t layerIndex = p.data[0];
    if (layerIndex < LEDManagement::layers.size()) {
      LEDManagement::layers.push_back({});
    }
    LEDManagement::Layer &layer = LEDManagement::layers.at(layerIndex);
    memcpy(layer.underGlow_leds, &p.data[1], p.header.size - 1);
  });

  queue_init(&txMessages, sizeof(Communications_protocol::Packet), 20);
  queue_init(&rxMessages, sizeof(Communications_protocol::Packet), 20);
  Packet packet{};
  packet.header.device  = device;
  packet.header.command = Communications_protocol::CONNECTED;
  sendPacket(packet);
}

bool Communications::sendPacket(Packet packet) {
  packet.header.device = device;
  queue_add_blocking(&txMessages, &packet);
  return true;
}
#endif
