#ifdef KEYSCANNER
#include <string.h>
#include "SPI.hpp"
#include "Config.hpp"
#include <Keyscanner.hpp>
#include <IS31FL3743B.hpp>
#include <LEDManagement.hpp>
#include "Communications.h"
#include "pico/util/queue.h"
#include "hardware/clocks.h"
#include "RFGW_communications.h"
#include "CRC.h"
#include "debug_print.h"
#include "BatteryManagement.hpp"

constexpr uint8_t SIDE_ID = 25;
static uint32_t TIMEOUT   = 900;
queue_t txMessages;
queue_t rxMessages;
Communications_protocol::Packet tx_message;
Communications_protocol::Packet rx_message;
Communications_protocol::Devices device;
bool need_polling{false};
uint32_t last_time_communication;
uint16_t keep_alive_timeout = 600;

//TODO: Create enum state for connections
uint8_t has_neuron_connection           = 0;
uint32_t last_time_communication_neuron = 0;
uint8_t has_rf_connection               = 0;
bool just_connected                     = false;

class Communications Communications;

void cleanQueues() {
  while (!queue_is_empty(&txMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);
  }
  while (!queue_is_empty(&rxMessages)) {
    queue_remove_blocking(&rxMessages, &rx_message);
  }
}


void neuronDisconnection() {
  DBG_PRINTF_TRACE("Neuron disconnected\n");
  has_neuron_connection = false;
  LEDManagement::set_mode_disconnected();
  //Clean queue
  cleanQueues();
  keep_alive_timeout = 600;
  just_connected     = false;
}

void rfDisconnection(bool cleanRf = true) {
  DBG_PRINTF_TRACE("Neuron rf disconnected\n");
  has_rf_connection = false;
  LEDManagement::set_mode_disconnected();
  //    Clean queues
  cleanQueues();
  if (cleanRf)
    RFGWCommunication::cleanMessages();
  keep_alive_timeout = 600;
  just_connected     = false;
}

void connectionStateMachine() {
  if (!just_connected) return;
  static uint32_t last_wait_time = 0;
  if (uint32_t ms_since_enter = to_ms_since_boot(get_absolute_time()); has_rf_connection && ms_since_enter - last_wait_time < 20) {
    last_wait_time = ms_since_enter;
    return;
  }

  enum class ConnectionState : uint8_t {
    BRIGHTNESS,
    BATTERY_LEVEL,
    BATTERY_STATUS,
    BATTERY_SAVING,
    PALETTE,
    FIRST_LAYER_KEYMAP_COLOR,
    FIRST_LAYER_UNDERGLOW_CONNECTION,
    LED_MODE,
    NEXT_LAYER,
    NEXT_UNDERGLOW,
  };
  static ConnectionState connectionState;
  static uint8_t layer = 0;

  Packet p{};
  switch (connectionState) {
  case ConnectionState::BRIGHTNESS:
    p.header.command = Communications_protocol::BRIGHTNESS;
    Communications.sendPacket(p);
    connectionState = ConnectionState::BATTERY_LEVEL;
    break;
  case ConnectionState::BATTERY_LEVEL: {
    p.header.command   = Communications_protocol::BATTERY_LEVEL;
    auto battery_level = RFGWCommunication::getBatteryLevel();
    p.header.size      = sizeof(battery_level);
    memcpy(p.data, &battery_level, p.header.size);
    Communications.sendPacket(p);
    connectionState = ConnectionState::BATTERY_STATUS;
  } break;
  case ConnectionState::BATTERY_STATUS: {
    p.header.command    = Communications_protocol::BATTERY_STATUS;
    auto battery_status = RFGWCommunication::getBatteryStatus();
    p.header.size       = sizeof(battery_status);
    memcpy(p.data, &battery_status, p.header.size);
    Communications.sendPacket(p);
    connectionState = ConnectionState::BATTERY_SAVING;
  } break;
  case ConnectionState::BATTERY_SAVING: {
    p.header.command = Communications_protocol::BATTERY_SAVING;
    Communications.sendPacket(p);
    connectionState = ConnectionState::PALETTE;
  } break;
  case ConnectionState::PALETTE:
    p.header.command = Communications_protocol::PALETTE_COLORS;
    Communications.sendPacket(p);
    connectionState = ConnectionState::FIRST_LAYER_KEYMAP_COLOR;
    break;
  case ConnectionState::FIRST_LAYER_KEYMAP_COLOR:
    p.header.size    = 1;
    p.data[0]        = layer;
    p.header.command = Communications_protocol::LAYER_KEYMAP_COLORS;
    Communications.sendPacket(p);
    connectionState = ConnectionState::FIRST_LAYER_UNDERGLOW_CONNECTION;
    break;
  case ConnectionState::FIRST_LAYER_UNDERGLOW_CONNECTION:
    p.header.size    = 1;
    p.data[0]        = layer++;
    p.header.command = Communications_protocol::LAYER_UNDERGLOW_COLORS;
    Communications.sendPacket(p);
    connectionState = ConnectionState::LED_MODE;
    break;
  case ConnectionState::LED_MODE:
    p.header.command = Communications_protocol::MODE_LED;
    Communications.sendPacket(p);
    connectionState = ConnectionState::NEXT_LAYER;
    break;
  case ConnectionState::NEXT_LAYER:
    p.header.size    = 1;
    p.data[0]        = layer;
    p.header.command = Communications_protocol::LAYER_KEYMAP_COLORS;
    Communications.sendPacket(p);
    connectionState = ConnectionState::NEXT_UNDERGLOW;
    break;
  case ConnectionState::NEXT_UNDERGLOW:
    p.header.size    = 1;
    p.data[0]        = layer++;
    p.header.command = Communications_protocol::LAYER_UNDERGLOW_COLORS;
    Communications.sendPacket(p);
    if (layer == 10) {
      just_connected  = false;
      layer           = 0;
      connectionState = ConnectionState::BRIGHTNESS;
    } else {
      connectionState = ConnectionState::NEXT_LAYER;
    }
    break;
  }
}
void Communications::run() {
  connectionStateMachine();
  uint32_t ms_since_enter = to_ms_since_boot(get_absolute_time());
  if (ms_since_enter - last_time_communication > keep_alive_timeout || need_polling || KeyScanner.newKey()) {
    last_time_communication = ms_since_enter;
    Packet packet{};
    if (KeyScanner.newKey() && !(!has_neuron_connection && !has_rf_connection)) {
      KeyScanner.keyState(false);
      packet.header.command = Communications_protocol::HAS_KEYS;
      packet.header.size    = KeyScanner.readMatrix(packet.data);
    } else {
      packet.header.command = IS_ALIVE;
      packet.header.size    = 0;
    }
    sendPacket(packet);
  }

  if (!queue_is_empty(&txMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);

    //Wired mode has more priority
    SPI::read_write_buffer(SPI::CSList::CSN2, tx_message.buf, rx_message.buf, sizeof(Packet));
    //If we have a response then update the time.
    uint8_t rx_crc        = rx_message.header.crc;
    rx_message.header.crc = 0;
    uint8_t crc_8         = crc8(rx_message.buf, sizeof(Header) + rx_message.header.size);
    if (rx_message.header.command != IS_DEAD && crc_8 == rx_crc) {
      last_time_communication_neuron = ms_since_enter;
      need_polling                   = rx_message.header.has_more_packets;
      callbacks.call(rx_message.header.command, rx_message);
      return;
    }

    //If the communication with the neuron could not be established, try to send to the message to the RF
    if (RFGWCommunication::isEnabled()) {
      RFGWCommunication::sendPacket(tx_message);
    }
  }

  if (has_neuron_connection && ms_since_enter - last_time_communication_neuron > TIMEOUT) {
    neuronDisconnection();
  }

  if (has_rf_connection && ms_since_enter - RFGWCommunication::last_time_communication_rf > TIMEOUT) {
    rfDisconnection();
  }
}

void Communications::init() {
  if (gpio_get(SIDE_ID)) {
    device = Communications_protocol::KEYSCANNER_DEFY_RIGHT;
  } else {
    device = Communications_protocol::KEYSCANNER_DEFY_LEFT;
  }

  auto empty_func = [](Packet const &) {};

  callbacks.bind(IS_ALIVE, [this](Packet p) {
    if ((!has_neuron_connection && !has_rf_connection)) {
      if (p.header.device == Communications_protocol::WIRED_NEURON_DEFY) {
        has_neuron_connection = 1;
        DBG_PRINTF_TRACE("Wired Neuron is available to connect");
      }
      if (p.header.device == Communications_protocol::RF_NEURON_DEFY) {
        has_rf_connection = 1;
        DBG_PRINTF_TRACE("RF Neuron is available to connect");
      }
      if (p.header.device == Communications_protocol::NEURON_DEFY) {
        has_neuron_connection = 1;
        DBG_PRINTF_TRACE("Neuron is available to connect");
      }
      p.header.device  = device;
      p.header.command = Communications_protocol::CONNECTED;
      uint32_t version = FMW_VERSION;
      memcpy(p.data, &version, sizeof(version));
      p.header.size = sizeof(version);
      sendPacket(p);
    }
  });

  callbacks.bind(CONNECTED, [](Packet const &p) {
    has_neuron_connection = false;
    has_rf_connection     = false;

    if (p.header.device == Communications_protocol::RF_NEURON_DEFY) {
      has_rf_connection  = 2;
      keep_alive_timeout = 1000;
      TIMEOUT            = 2000;
      DBG_PRINTF_TRACE("RF connected");
    }
    if (p.header.device == Communications_protocol::NEURON_DEFY) {
      has_neuron_connection = 2;
      keep_alive_timeout    = 100;
      TIMEOUT               = 400;
      DBG_PRINTF_TRACE("Neuron connected");
    }
    if (p.header.device == Communications_protocol::WIRED_NEURON_DEFY) {
      has_neuron_connection = 2;
      keep_alive_timeout    = 100;
      TIMEOUT               = 400;
      DBG_PRINTF_TRACE("Wired Neuron connected");
    }
    just_connected = true;
  });

  callbacks.bind(DISCONNECTED, [](Packet const &) {
    rfDisconnection(false);
    neuronDisconnection();
    sleep_ms(2000);
  });

  callbacks.bind(SLEEP, [](Packet const &) {
    IS31FL3743B::set_enable(false);
    LEDManagement::set_enable_underGlow(false);
  });

  callbacks.bind(WAKE_UP, [](Packet const &) {
    IS31FL3743B::set_enable(true);
    LEDManagement::set_enable_underGlow(true);
    LEDManagement::set_updated(true);
  });

  callbacks.bind(VERSION, [this](Packet p) {
    uint32_t version = FMW_VERSION;
    memcpy(p.data, &version, sizeof(version));
    p.header.size = sizeof(version);
    sendPacket(p);
  });

  callbacks.bind(HAS_KEYS, [this](Packet const &) {
    DBG_PRINTF_TRACE("Warning why has enter here!");
    //    sendPacket(p);
  });

  callbacks.bind(KEYSCAN_INTERVAL, [](Packet const &p) {
    KeyScanner.keyScanInterval(p.data[0]);
  });

  callbacks.bind(GET_SHORT_LED, [this](Packet p) {
    p.header.size = IS31FL3743B::get_short_leds(p.data);
    sendPacket(p);
  });

  callbacks.bind(GET_OPEN_LED, [this](Packet p) {
    p.header.size = IS31FL3743B::get_open_leds(p.data);
    sendPacket(p);
  });

  callbacks.bind(BRIGHTNESS, [](Packet const &p) {
    LEDManagement::setMaxBrightness(p.data[0]);
  });

  callbacks.bind(MODE_LED, [](Packet const &p) {
    LEDManagement::set_led_mode(p.data);
  });

  //TODO: SET_LED
  callbacks.bind(LED, empty_func);

  callbacks.bind(PALETTE_COLORS, [](Packet const &p) {
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size);
  });

  callbacks.bind(LAYER_KEYMAP_COLORS, [](Packet const &p) {
    uint8_t layerIndex = p.data[0];
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
  });

  //Battery
  callbacks.bind(BATTERY_SAVING, [](Packet const &p) {
    DBG_PRINTF_TRACE("Battery Saving is %i", p.data[0]);
    BatteryManagement::set_battery_saving(p.data[0]);
  });

  //Config
  callbacks.bind(SET_ENABLE_LED_DRIVER, [](Packet const &) {
    uint8_t enable;
    memcpy(&enable, &rx_message.data[0], sizeof(uint8_t));
    IS31FL3743B::set_enable(enable);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.led_driver_enabled     = enable;
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_ENABLE_UNDERGLOW, [](Packet const &) {
    uint8_t enable;
    memcpy(&enable, &rx_message.data[0], sizeof(uint8_t));
    gpio_put(UG_EN, enable);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.underGlow_enabled      = enable;
    Configuration::set_configuration(configuration);
  });

  callbacks.bind(ALIVE_INTERVAL, [](Packet const &) {
    uint32_t pooling_rate_base;
    uint32_t pooling_rate_variation;
    memcpy(&pooling_rate_base, &rx_message.data[0], sizeof(uint32_t));
    memcpy(&pooling_rate_variation, &rx_message.data[sizeof(uint32_t)], sizeof(uint32_t));
    keep_alive_timeout                              = pooling_rate_base;
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.pooling_rate_base      = pooling_rate_base;
    configuration.start_info.pooling_rate_variation = pooling_rate_variation;
    DBG_PRINTF_TRACE("Sending alive interval base %lu and variation %lu", pooling_rate_base, pooling_rate_variation);
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_SPI_SPEED, [](Packet const &) {
    uint32_t spi_speed_base;
    uint32_t spi_speed_variation;
    memcpy(&spi_speed_base, &rx_message.data[0], sizeof(uint32_t));
    memcpy(&spi_speed_variation, &rx_message.data[sizeof(uint32_t)], sizeof(uint32_t));
    SPI::set_baudrate(spi_speed_base);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.spi_speed_base         = spi_speed_base;
    configuration.start_info.spi_speed_variation    = spi_speed_variation;
    DBG_PRINTF_TRACE("Sending spi speed base %lu and variation %lu", spi_speed_base, spi_speed_variation);
    Configuration::set_configuration(configuration);
  });

  callbacks.bind(SET_CLOCK_SPEED, [](Packet const &) {
    uint32_t cpu_speed;
    memcpy(&cpu_speed, &rx_message.data[0], sizeof(uint32_t));
    set_sys_clock_khz(cpu_speed, true);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.cpu_speed              = cpu_speed;
    DBG_PRINTF_TRACE("Setting cpuSpeed to %lu", cpu_speed);
    Configuration::set_configuration(configuration);
  });

  callbacks.bind(SET_LED_DRIVER_PULLUP, [](Packet const &) {
    uint8_t led_driver_pull_up;
    memcpy(&led_driver_pull_up, &rx_message.data[0], sizeof(uint8_t));
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.pull_up_config         = led_driver_pull_up;
    DBG_PRINTF_TRACE("Setting ledDriver in left side to %i", led_driver_pull_up);
    Configuration::set_configuration(configuration);
    IS31FL3743B::setPullUpRegister(led_driver_pull_up);
  });

  queue_init(&txMessages, sizeof(Communications_protocol::Packet), 100);
  queue_init(&rxMessages, sizeof(Communications_protocol::Packet), 100);
}

bool Communications::sendPacket(Packet packet) {
  packet.header.device = device;
  packet.header.crc    = 0;
  packet.header.crc    = crc8(packet.buf, sizeof(Header) + packet.header.size);
  queue_add_blocking(&txMessages, &packet);
  return true;
}
#endif
