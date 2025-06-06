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
#include "debug_print.h"


#include "LEDManagement.hpp"
#include "RFGW_communications.h"
#include "WiredCommunication.hpp"
#include "BatteryManagement.hpp"
#include "Keyscanner.hpp"
#include "hal_mcu_systim.h"

enum class Host_status
{
    CONNECTED,
    DISCONNECTED,
    UNKNOWN
};

struct host_connection_t
{
    Host_status connection = Host_status::UNKNOWN;
    Host_status previous_conn = Host_status::DISCONNECTED;
    bool sleep_enabled;
    bool shut_down_leds = false; // This is used to know if we should turn off the LEDs when the host is disconnected.
                                                    // And avoid the disconnection LED effect.
};
host_connection_t host_status;

systim_timer_t *host_disconnected_timer;

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

void check_if_keyboard_is_wired_wireless()
{
  static bool configuration_set = false;

  if (WiredCommunication::connectionEstablished && !RFGateway::module_is_connected() && !configuration_set)
  {
      configuration_set = true;
      KeyScanner.specifications.conection = KsConfig::Device::Wired;
      //debug message
      /*DBG_PRINTF_TRACE("keyboard connection wireless" );
      DBG_PRINTF_TRACE("keyboard configuration %i", KeyScanner.specifications.configuration );
      DBG_PRINTF_TRACE("keyboard name %i", KeyScanner.specifications.device_name );
      DBG_PRINTF_TRACE("Chip id: ");
      DBG_PRINTF_TRACE("%s", KeyScanner.specifications.chip_id);
      DBG_PRINTF_TRACE("rf_gatewar_chip_id: %lu",  KeyScanner.specifications.rf_gateway_chip_id);*/
  }
  else if (RFGateway::module_is_connected() && !configuration_set)
  {
      configuration_set = true;
      KeyScanner.specifications.conection = KsConfig::Device::Wireless;
      //debug message
      /*DBG_PRINTF_TRACE("keyboard connection wireless" );
      DBG_PRINTF_TRACE("keyboard configuration %i", KeyScanner.specifications.configuration );
      DBG_PRINTF_TRACE("keyboard name %i", KeyScanner.specifications.device_name );
      DBG_PRINTF_TRACE("Chip id: ");
      DBG_PRINTF_TRACE("%s", KeyScanner.specifications.chip_id);
      DBG_PRINTF_TRACE("rf_gatewar_chip_id: %lu",  KeyScanner.specifications.rf_gateway_chip_id);*/
  }

  if (configuration_set && KeyScanner.get_information_asked())
  {
      KeyScanner.send_configuration_package();
      KeyScanner.information_asked(false);

      Packet host_status_packet{};
      host_status_packet.header.command = HOST_CONNECTION_STATUS;
      host_status_packet.header.size = 1;
      Communications.sendPacket(host_status_packet);
  }
}

bool Communications::is_host_connected()
{
    if (host_status.connection == Host_status::CONNECTED)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void Communications::run()
{
  WiredCommunication::run();
  RFGWCommunication::run();

  if (!WiredCommunication::connectionEstablished && !RFGWCommunication::connectionEstablished)
  {
    if (KeyScanner.key_timer_expired())
    {
      goToSleep();
    }
  }
  else if (host_status.connection == Host_status::DISCONNECTED && systim_timer_check(host_disconnected_timer))
  {
      if (host_status.sleep_enabled )
      {
          goToSleep();
      }
      else
      {
          // This case is when the host is disconnected and we are not in sleep mode.
          if(KeyScanner::new_keypress()) // If we detect some key press we want to turn on the LEDs, this is to have a consistent behavior.
          {
              LEDManagement::force_bl_shutdown_state(false);
              LEDManagement::force_ug_shutdown_state(false);
              LEDManagement::turnPowerOn(true);
              systim_timer_set_ms(host_disconnected_timer, KsConfig::TIMEOUT_NO_CONNECTION); // Reset the timer to avoid the disconnection LED effect.
          }
          else
          {
              LEDManagement::turnPowerOff();
              LEDManagement::force_bl_shutdown_state(true);
              LEDManagement::force_ug_shutdown_state(true);
          }
      }
  }
  KeyScanner::set_key_press_event(false); // Reset the key press event
  check_if_keyboard_is_wired_wireless();
}

void Communications::init()
{
  systim_timer_init(&host_disconnected_timer);
  systim_timer_set_ms(host_disconnected_timer, KsConfig::TIMEOUT_NO_CONNECTION);
  if (KsConfig::get_side() == KsConfig::Side::RIGHT) {
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
    //DBG_PRINTF_TRACE("Received VERSION from %i", p.header.device);
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

      //DBG_PRINTF_TRACE("RECEIVED BRIGHTNESS from %i ", p.header.device);
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

  callbacks.bind(MODE_LED, [this](Packet const &p)
  {
    LEDManagement::layer_config_received.led_mode = true;
    DBG_PRINTF_TRACE("Receive MODE_LED from %i ", p.header.device);

    //If we have received the configuration, we can set the LED mode.
    //If not, we will reset the flag. And we will wait for the next configuration.
    if (LEDManagement::config_received())
    {
        if (host_status.connection != Host_status::UNKNOWN)
        {
            LEDManagement::set_led_mode(p.data);
        }
    }
    else
    {
        LEDManagement::layer_config_received.led_mode = false;
        request_keyscanner_layers();
    }
  });

  //TODO: SET_LED
  callbacks.bind(LED, empty_func);

  callbacks.bind(PALETTE_COLORS, [](Packet const &p)
  {
    DBG_PRINTF_TRACE("Received PALETTE_COLORS from %i ", p.header.device);
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size - 1);
    LEDManagement::layer_config_received.palette = true;
  });

  callbacks.bind(LAYER_KEYMAP_COLORS, [](Packet const &p) {

    DBG_PRINTF_TRACE("Received LAYER_KEYMAP_COLORS from %i ", p.header.device);

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

    if (layerIndex == 9)
    {
        LEDManagement::layer_config_received.bl_layer = true;
    }
  });

  callbacks.bind(LAYER_UNDERGLOW_COLORS, [this](Packet p) {

    uint8_t layerIndex = p.data[0];
    //DBG_PRINTF_TRACE("Received LAYER_UNDERGLOW_COLORS from %i %i", p.header.device, layerIndex);
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

  callbacks.bind(HOST_CONNECTION, [this](Packet const &p)
  {
    DBG_PRINTF_TRACE("Received HOST_CONNECTION from %i ", p.header.device);
    //Check if we need to show the disconnection LED effect, or we should turn off the LEDs directly.
    host_status.shut_down_leds = p.data[3] == 1;

    if (p.data[0] == 1)
    {
        DBG_PRINTF_TRACE("HOST CONNECTED ");
        host_status.connection = Host_status::CONNECTED;

        if(host_status.previous_conn != host_status.connection)
        {
            host_status.previous_conn = host_status.connection;
            Packet mode_led_packet{};
            mode_led_packet.header.command = Communications_protocol::MODE_LED;
            mode_led_packet.header.size = 1;
            sendPacket(mode_led_packet);

            LEDManagement::force_bl_shutdown_state(false);
            LEDManagement::force_ug_shutdown_state(false);
            LEDManagement::turnPowerOn(true);
        }
    }
    else
    {
        DBG_PRINTF_TRACE("HOST DISCONNECTED ");
        host_status.connection = Host_status::DISCONNECTED;

        systim_timer_set_ms(host_disconnected_timer, KsConfig::TIMEOUT_NO_CONNECTION);

        KeyScanner::set_key_press_event(false); // Reset the key press event.

        if(host_status.previous_conn != host_status.connection && p.data[1] == 0)
        {
            host_status.previous_conn = host_status.connection;
            if ( host_status.shut_down_leds )
            {
                LEDManagement::force_bl_shutdown_state(true);
                LEDManagement::force_ug_shutdown_state(true);
            }
            else
            {
                LEDManagement::set_mode_disconnected();
            }
        }
    }
    DBG_PRINTF_TRACE("sleep enabled %i", p.data[2]);
    host_status.sleep_enabled = p.data[2] != 1;
  });

  callbacks.bind(CONFIGURATION, [](Packet const &p) {
      KeyScanner.information_asked(true);
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
