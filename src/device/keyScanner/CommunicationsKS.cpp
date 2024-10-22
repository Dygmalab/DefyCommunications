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

#ifdef KEYSCANNER
#include "Communications.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include "debug_print.h"
#include "SPI.hpp"
#include "CRC.h"
#include "pico/util/queue.h"
#include "LEDManagement.hpp"
#include "RFGW_communications.h"
#include "WiredCommunication.hpp"
#include "BatteryManagement.hpp"
#include "Keyscanner.hpp"

constexpr uint8_t SIDE_ID = 25;
Communications_protocol::Devices device;
using led_type_t = LEDManagement::LedBrightnessControlEffect;
class Communications Communications;


void goToSleep() {
  LEDManagement::turnPowerOff();
  RFGWCommunication::communicationType = RFGWCommunication::CommunicationType::DISABLED;
  RFGateway::rf_disable();
  for (int i = 0; i < 100; ++i) {
    RFGateway::run();
  }
  BatteryManagement::goToSleep();
}

void check_if_keyboard_is_wired_wireless(){
  static uint8_t counter = 0;
  static bool configuration_set = false;

  if (WiredCommunication::connectionEstablished && !RFGateway::module_is_connected() && !configuration_set){
    //We are on a wired keyboard.
    const constexpr uint32_t timeout = 500;
    uint32_t ms_since_enter                        = to_ms_since_boot(get_absolute_time());
    static uint32_t last_time                       = ms_since_enter;

    if (ms_since_enter - last_time >= timeout) {
      last_time = ms_since_enter;
      counter++;

      if (counter > 3) {
        KeyScanner.specifications.conection = Pins::Device::Wired;
        counter = 0;
        configuration_set = true;
      }
    }
  }
  else if (RFGateway::module_is_connected() && !configuration_set)
  {
    uint32_t ms_since_enter                        = to_ms_since_boot(get_absolute_time());
    const constexpr uint32_t timeout = 2000;
    static uint32_t last_time                       = ms_since_enter;

    if (ms_since_enter - last_time >= timeout) {
      last_time = ms_since_enter;
      configuration_set = true;
      KeyScanner.specifications.conection = Pins::Device::Wireless;
      //debug message
      /*DBG_PRINTF_TRACE("keyboard connection wireless" );
      DBG_PRINTF_TRACE("keyboard configuration %i", KeyScanner.specifications.configuration );
      DBG_PRINTF_TRACE("keyboard name %i", KeyScanner.specifications.device_name );
      DBG_PRINTF_TRACE("Chip id: ");
      DBG_PRINTF_TRACE("%s", KeyScanner.specifications.chip_id);
      DBG_PRINTF_TRACE("rf_gatewar_chip_id: %lu",  KeyScanner.specifications.rf_gateway_chip_id);*/
    }
  }

  if (configuration_set && KeyScanner.get_information_asked()){
      KeyScanner.send_configuration_package();
      KeyScanner.information_asked(false);
  }
}

void Communications::run() {
  WiredCommunication::run();
  RFGWCommunication::run();
  if (!WiredCommunication::connectionEstablished && !RFGWCommunication::connectionEstablished) {
    //TODO: be careful this is not going to break in the upgrade procedure.
    const constexpr uint32_t timeout_no_connection = 40000;
    uint32_t ms_since_enter                        = to_ms_since_boot(get_absolute_time());
    if (ms_since_enter - KeyScanner.get_ms_since_last_key_sent() >= timeout_no_connection) {
      goToSleep();
    }
  }

  check_if_keyboard_is_wired_wireless();
}

void Communications::init()
{
  if (gpio_get(SIDE_ID)) {
    device = Communications_protocol::KEYSCANNER_DEFY_RIGHT;
  } else {
    device = Communications_protocol::KEYSCANNER_DEFY_LEFT;
  }

  auto empty_func = [](Packet const &) {};

  callbacks.bind(SLEEP, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received SLEEP from %i", p.header.device);
    goToSleep();
  });

  callbacks.bind(WAKE_UP, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received WAKE_UP from %i and now is deprecated", p.header.device);
  });

  callbacks.bind(VERSION, [this](const Packet &p) {
    DBG_PRINTF_TRACE("Received VERSION from %i", p.header.device);
    uint32_t version = FMW_VERSION;
    Packet packet{};
    memcpy(packet.data, &version, sizeof(version));
    packet.header.size = sizeof(version);
    sendPacket(packet);
  });

  callbacks.bind(HAS_KEYS, [this](Packet const &p) {
    DBG_PRINTF_ERROR("Why this is has keys %i", p.header.device);
    for (int i = 0; i < MAX_TRANSFER_SIZE; i++) {
      DBG_PRINTF_ERROR("%i", p.buf[i]);
    }
    sendPacket(p);
  });

  callbacks.bind(GET_SHORT_LED, [this](Packet p) {
/*    DBG_PRINTF_TRACE("Received GET_SHORT_LED from %i ", p.header.device);
    Packet packet{};
    packet.header.size = IS31FL3743B::get_short_leds(packet.data);
    sendPacket(packet);*/
  });

  callbacks.bind(GET_OPEN_LED, [this](Packet p) {
/*    DBG_PRINTF_TRACE("Received GET_OPEN_LED from %i ", p.header.device);
    Packet packet{};
    packet.header.size = IS31FL3743B::get_open_leds(packet.data);
    sendPacket(packet);*/
  });

  callbacks.bind(BRIGHTNESS, [](Packet const &p) {

    LEDManagement::layer_config_received.brightness = true;

      DBG_PRINTF_TRACE("RECEIVED BRIGHTNESS from %i ", p.header.device);
    /* p.data[0] led driver brightness
     * p.data[1] underglow brightness
     * p.data[2] LED effect id
     * p.data[3] take brightness handler?
     */
    if (p.data[3] == false) {
//        DBG_PRINTF_TRACE("Calling onDismount");
        LEDManagement::onDismount(static_cast<led_type_t>(p.data[2]));
    } else {
//        DBG_PRINTF_TRACE("Calling onMount");
        LEDManagement::onMount(static_cast<led_type_t>(p.data[2]), p.data[0], p.data[1]);
    }
  });

  callbacks.bind(MODE_LED, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received MODE_LED from %i ", p.header.device);
    LEDManagement::layer_config_received.led_mode = true;
    LEDManagement::set_led_mode(p.data);
  });

  //TODO: SET_LED
  callbacks.bind(LED, empty_func);

  callbacks.bind(PALETTE_COLORS, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received PALETTE_COLORS from %i ", p.header.device);
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size - 1);
  });

  callbacks.bind(LAYER_KEYMAP_COLORS, [](Packet const &p) {

      DBG_PRINTF_TRACE("Received LAYER_KEYMAP_COLORS from %i ", p.header.device);
    LEDManagement::layer_config_received.bl_layer = true;

    uint8_t layerIndex = p.data[0];
   // DBG_PRINTF_TRACE("Received LAYER_KEYMAP_COLORS from %i %i ", p.header.device, layerIndex);
    if (layerIndex < LEDManagement::layers.size()) {
      LEDManagement::layers.emplace_back();
    }
    union PaletteJoiner {
      struct {
        uint8_t firstColor : 4;
        uint8_t secondColor : 4;
      };
      uint8_t paletteColor;
    };
    LEDManagement::Layer &layer = LEDManagement::layers.at(layerIndex);
    PaletteJoiner message[p.header.size - 1];
    memcpy(message, &p.data[1], p.header.size - 1);
    uint8_t k{};
    bool swap = true;
    for (uint8_t j = 0; j < sizeof(layer.keyMap_leds); ++j) {
      if (swap) {
        layer.keyMap_leds[j] = message[k].firstColor;
      } else {
        layer.keyMap_leds[j] = message[k++].secondColor;
      }
      swap = !swap;
    }
  });

  callbacks.bind(LAYER_UNDERGLOW_COLORS, [this](Packet p) {

    uint8_t layerIndex = p.data[0];
    DBG_PRINTF_TRACE("Received LAYER_UNDERGLOW_COLORS from %i %i", p.header.device, layerIndex);
    if (layerIndex < LEDManagement::layers.size()) {
      LEDManagement::layers.emplace_back();
    }
    union PaletteJoiner {
      struct {
        uint8_t firstColor : 4;
        uint8_t secondColor : 4;
      };
      uint8_t paletteColor;
    };
    LEDManagement::Layer &layer = LEDManagement::layers.at(layerIndex);
    PaletteJoiner message[p.header.size - 1];
    memcpy(message, &p.data[1], p.header.size - 1);
    uint8_t k{};
    bool swap = true;
    for (uint8_t j = 0; j < sizeof(layer.underGlow_leds); ++j) {
      if (swap) {
        layer.underGlow_leds[j] = message[k].firstColor;
      } else {
        layer.underGlow_leds[j] = message[k++].secondColor;
      }
      swap = !swap;
    }

      if (layerIndex == 9)
      {
          LEDManagement::layer_config_received.ug_layer = true;
      }
  });


  callbacks.bind(CONFIGURATION, [](Packet const &p) {
      KeyScanner.information_asked(true);
    DBG_PRINTF_TRACE("Received CONFIGURATION from %i ", p.header.device);
  });


  //Battery
  callbacks.bind(BATTERY_SAVING, [](Packet const &p) {
   // DBG_PRINTF_TRACE("Received BATTERY_SAVING from %i with value %i", p.header.device, p.data[0]);
    BatteryManagement::set_battery_saving(p.data[0]);
  });

  WiredCommunication::wcom_config_t config = {.device = device};
  WiredCommunication::init( &config );
  RFGWCommunication::init();
}

bool Communications::sendPacket(Packet packet) {
  if (WiredCommunication::connectionEstablished)
    return WiredCommunication::sendPacket(packet);
  if (RFGWCommunication::connectionEstablished)
    return RFGWCommunication::sendPacket(packet);
  return false;
}

void Communications::request_keyscanner_layers()
{
    Communications_protocol::Packet p{};
    p.header.command = Communications_protocol::RETRY_LAYERS;
    sendPacket(p);
}

#endif
