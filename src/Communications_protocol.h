#ifndef KEYSCANNER_COMMUNICATIONS_PROTOCOL_H_
#define KEYSCANNER_COMMUNICATIONS_PROTOCOL_H_
#include "stdio.h"

namespace Communications_protocol {

enum Devices : uint8_t {
  UNKNOWN = 0,
  KEYSCANNER_DEFY_LEFT,
  KEYSCANNER_DEFY_RIGHT,
  RF_DEFY_LEFT,
  RF_DEFY_RIGHT,
  NEURON_DEFY,
  RF_NEURON_DEFY,
  WIRED_NEURON_DEFY,
};

static_assert(sizeof(Devices) == sizeof(uint8_t));

enum Commands : uint8_t {
  IS_DEAD = 0,
  IS_ALIVE,
  CONNECTED,
  DISCONNECTED,
  SLEEP,
  WAKE_UP,
  VERSION,
  SET_ALIVE_INTERVAL,
  //Keys
  HAS_KEYS = 10,
  SET_KEYSCAN_INTERVAL,
  //LEDS
  SET_BRIGHTNESS = 20,
  SET_MODE_LED,
  SET_LED,
  SET_PALETTE_COLORS,
  SET_LAYER_KEYMAP_COLORS,
  SET_LAYER_UNDERGLOW_COLORS,
  GET_OPEN_LED,
  GET_SHORT_LED,
  //Config
  SET_SPI_SPEED = 100,
  SET_CLOCK_SPEED,
  SET_LED_DRIVER_PULLUP,
  SET_ENABLE_UNDERGLOW,
  SET_ENABLE_LED_DRIVER,
};

static_assert(sizeof(Commands) == sizeof(uint8_t));

struct Header {
  Commands command;
  Devices device;
  struct {
    uint8_t size : 7;
    bool has_more_packets : 1;
  };
  uint8_t crc;
};

static_assert(sizeof(Header) == (sizeof(uint8_t) * 4));

static const constexpr uint8_t MAX_TRANSFER_SIZE = 32;

union Packet {
  struct {
    Header header;
    uint8_t data[MAX_TRANSFER_SIZE - sizeof(Header)];
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};
static_assert(sizeof(Packet) == MAX_TRANSFER_SIZE);

}  // namespace Communications_protocol
#endif
