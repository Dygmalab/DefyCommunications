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

#ifndef KEYSCANNER_COMMUNICATIONS_PROTOCOL_H_
#define KEYSCANNER_COMMUNICATIONS_PROTOCOL_H_


#include <cstdint>


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
  BLE_NEURON_2_DEFY,
  BLE_DEFY_LEFT,
  BLE_DEFY_RIGHT,
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
  ALIVE_INTERVAL,
  //Keys
  HAS_KEYS = 10,
  KEYSCAN_INTERVAL,
  //LEDS
  BRIGHTNESS = 20,
  MODE_LED,
  LED,
  PALETTE_COLORS,
  LAYER_KEYMAP_COLORS,
  LAYER_UNDERGLOW_COLORS,
  GET_OPEN_LED,
  GET_SHORT_LED,
  RETRY_LAYERS,
  //Battery
  BATTERY_LEVEL = 40,
  BATTERY_STATUS,
  BATTERY_SAVING,
  //Config
  RF_ADDRESS = 50,
  CONFIGURATION,
  //Host connection
  HOST_CONNECTION,
  HOST_CONNECTION_STATUS,
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
static const constexpr uint8_t MAX_DATA_SIZE     = MAX_TRANSFER_SIZE - sizeof(Header);

union Packet {
  struct {
    Header header;
    uint8_t data[MAX_DATA_SIZE];
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};

static_assert(sizeof(Packet) == MAX_TRANSFER_SIZE);


}  // namespace Communications_protocol


#endif  // KEYSCANNER_COMMUNICATIONS_PROTOCOL_H_
