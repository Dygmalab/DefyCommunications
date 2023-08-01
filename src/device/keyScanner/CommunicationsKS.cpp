#ifdef KEYSCANNER
#include <string.h>
#include "SPI.hpp"
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
static uint32_t TIMEOUT   = 400;
constexpr static uint32_t TIMEOUT_DISCONECTION   = 10000;
queue_t txMessages;
queue_t rxMessages;
Communications_protocol::Packet tx_message;
Communications_protocol::Packet rx_message;
Communications_protocol::Devices device;
Communications_protocol::Devices connectedTo=UNKNOWN;
bool need_polling{false};
uint32_t last_time_communication;
uint32_t last_time_disconection;
uint16_t keep_alive_timeout = 100;

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
  DBG_PRINTF_TRACE("Wired Neuron disconnected\n");
  if (RFGWCommunication::relay_host) {
    device                        = Communications_protocol::KEYSCANNER_DEFY_LEFT;
    RFGWCommunication::relay_host = false;
    RFGateway::rf_disable();
  } else {
    has_neuron_connection = false;
  }
  LEDManagement::set_mode_disconnected();
  //Clean queue
  cleanQueues();
  keep_alive_timeout = 150;
  connectedTo = UNKNOWN;
  last_time_disconection = to_ms_since_boot(get_absolute_time());
  just_connected     = false;
}

void rfDisconnection(bool cleanRf = true) {
  DBG_PRINTF_TRACE("Neuron 2 rf disconnected\n");
  has_rf_connection = false;
  LEDManagement::set_mode_disconnected();
  //    Clean queues
  cleanQueues();
  if (cleanRf)
    RFGWCommunication::cleanMessages();
  keep_alive_timeout = 150;
  connectedTo = UNKNOWN;
  last_time_disconection = to_ms_since_boot(get_absolute_time());
  just_connected     = false;
}

void connectionStateMachine() {
  enum class ConnectionState : uint8_t {
    BRIGHTNESS,
    BATTERY_STATUS,
    BATTERY_LEVEL,
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
  if (!just_connected) {
    connectionState = ConnectionState::BRIGHTNESS;
    layer           = 0;
    return;
  }
  static uint32_t last_wait_time = 0;
  uint32_t ms_since_enter        = to_ms_since_boot(get_absolute_time());
  if (ms_since_enter - last_wait_time > 50) {
    last_wait_time = ms_since_enter;
    Packet p{};
    DBG_PRINTF_TRACE("Connection state is asking for %i", connectionState);
    switch (connectionState) {
    case ConnectionState::BRIGHTNESS:
      p.header.command = Communications_protocol::BRIGHTNESS;
      Communications.sendPacket(p);
      if (connectedTo == Communications_protocol::WIRED_NEURON_DEFY)
        connectionState = ConnectionState::PALETTE;
      else
        connectionState = ConnectionState::BATTERY_STATUS;
      break;
    case ConnectionState::BATTERY_STATUS:
      RFGateway::chg_status_get();
      connectionState = ConnectionState::BATTERY_LEVEL;
      break;
    case ConnectionState::BATTERY_LEVEL:
      RFGateway::bat_level_get();
      connectionState = ConnectionState::BATTERY_SAVING;
      break;
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
}


void Communications::run() {
  connectionStateMachine();
  uint32_t ms_since_enter = to_ms_since_boot(get_absolute_time());
  if(connectedTo == UNKNOWN && ms_since_enter - last_time_disconection > TIMEOUT_DISCONECTION){
    LEDManagement::turnPowerOff();
    if (KeyScanner.newKey()){
      last_time_disconection = ms_since_enter;
      LEDManagement::turnPowerOn();
    }
  }
  if (ms_since_enter - last_time_communication > keep_alive_timeout || need_polling || KeyScanner.newKey()) {
    Packet packet{};
    if (KeyScanner.newKey()) {
      DBG_PRINTF_TRACE("New key detected");
      KeyScanner.keyState(false);
      packet.header.command = Communications_protocol::HAS_KEYS;
      packet.header.size    = KeyScanner.readMatrix(packet.data);
      DBG_PRINTF_TRACE("Got key state %i %i %i %i %i %i", packet.data[0], packet.data[1], packet.data[2], packet.data[3], packet.data[4], ms_since_enter - last_time_communication);
    } else {
      DBG_PRINTF_TRACE("Adding is alive %i", ms_since_enter - last_time_communication);
      packet.header.command = IS_ALIVE;
      packet.data[0]        = HAS_KEYS;
      packet.header.size    = KeyScanner.readMatrix(&packet.data[1]) + 1;
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
      DBG_PRINTF_TRACE("Received wired Command %i from %i", rx_message.header.command, rx_message.header.device);
      //If its for the right just relay the info
      if (rx_message.header.device == Communications_protocol::BLE_DEFY_RIGHT) {
        RFGWCommunication::sendPacket(rx_message);
      } else if (rx_message.header.device == Communications_protocol::BLE_NEURON_2_DEFY) {
        if (has_neuron_connection == 3 && !gpio_get(SIDE_ID))
          RFGWCommunication::sendPacket(rx_message);
        callbacks.call(rx_message.header.command, rx_message);
      } else {
        callbacks.call(rx_message.header.command, rx_message);
      }
      return;
    }

    //If the communication with the neuron could not be established, try to send to the message to the RF only in the case it is not ble
    if (RFGWCommunication::isEnabled() && has_neuron_connection != 3) {
      DBG_PRINTF_TRACE("Sending via rf Command %i from %i", tx_message.header.command, tx_message.header.device);
      RFGWCommunication::sendPacket(tx_message);
    }
  }

  if (has_neuron_connection && ms_since_enter - last_time_communication_neuron > TIMEOUT) {
    neuronDisconnection();
  }

  if (has_rf_connection && ms_since_enter - RFGWCommunication::last_time_communication_rf > TIMEOUT) {
    DBG_PRINTF_TRACE("RF disconnected %i %i %i %i", to_ms_since_boot(get_absolute_time()), ms_since_enter, RFGWCommunication::last_time_communication_rf, ms_since_enter - RFGWCommunication::last_time_communication_rf);
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
        p.header.device       = device;
        has_neuron_connection = 1;
        DBG_PRINTF_TRACE("Wired Neuron is available to connect");
      }
      if (p.header.device == Communications_protocol::RF_NEURON_DEFY) {
        p.header.device   = device;
        has_rf_connection = 1;
        DBG_PRINTF_TRACE("RF Neuron 2 is available to connect");
      }
      if (p.header.device == Communications_protocol::NEURON_DEFY) {
        p.header.device       = device;
        has_neuron_connection = 1;
        DBG_PRINTF_TRACE("Neuron 2 is available to connect");
      }

      if (p.header.device == Communications_protocol::BLE_NEURON_2_DEFY) {
        if (!gpio_get(SIDE_ID)) {
          device = Communications_protocol::BLE_DEFY_LEFT;
        }
        has_neuron_connection = 1;
        DBG_PRINTF_TRACE("Ble Neuron 2 is available to connect");
      }
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
      keep_alive_timeout = 150;
      TIMEOUT            = 1500;
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
    if (p.header.device == Communications_protocol::BLE_NEURON_2_DEFY) {
      if (!gpio_get(SIDE_ID)) {
        has_neuron_connection         = 3;
        keep_alive_timeout            = 200;
        TIMEOUT                       = 1500;
        RFGWCommunication::relay_host = true;
        RFGateway::rf_disable();
      } else {
        has_neuron_connection = 2;
        keep_alive_timeout    = 100;
        TIMEOUT               = 400;
      }
      DBG_PRINTF_TRACE("Ble Neuron 2 Neuron connected");
    }
    connectedTo    = p.header.device;
    just_connected = true;
  });

  callbacks.bind(DISCONNECTED, [this](Packet const &p) {
    DBG_PRINTF_TRACE("Received disconnected from %i", p.header.device);
    if (has_rf_connection)
      rfDisconnection(false);
    if (has_neuron_connection)
      neuronDisconnection();
    sleep_ms(4000);
  });

  callbacks.bind(SLEEP, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received SLEEP from %i", p.header.device);
    LEDManagement::turnPowerOff();
  });

  callbacks.bind(WAKE_UP, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received WAKE_UP from %i", p.header.device);
    LEDManagement::turnPowerOn();
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

  callbacks.bind(KEYSCAN_INTERVAL, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received VERSION from %i", p.header.device);
    KeyScanner.keyScanInterval(p.data[0]);
  });

  callbacks.bind(GET_SHORT_LED, [this](Packet p) {
    DBG_PRINTF_TRACE("Received GET_SHORT_LED from %i ", p.header.device);
    Packet packet{};
    packet.header.size = IS31FL3743B::get_short_leds(packet.data);
    sendPacket(packet);
  });

  callbacks.bind(GET_OPEN_LED, [this](Packet p) {
    DBG_PRINTF_TRACE("Received GET_OPEN_LED from %i ", p.header.device);
    Packet packet{};
    packet.header.size = IS31FL3743B::get_open_leds(packet.data);
    sendPacket(packet);
  });

  callbacks.bind(BRIGHTNESS, [](Packet const &p) {
    float driver_brightness     = BatteryManagement::mapRange(p.data[0], 0, 255, 0,  LEDManagement::get_max_ledDriver_brightness());
    float under_glow_brightness = BatteryManagement::mapRange(p.data[1], 0, 255, 0,  LEDManagement::get_max_underglow_brightness());
    DBG_PRINTF_TRACE("Received BRIGHTNESS from %i values %i %f %i %f", p.header.device, p.data[0], driver_brightness, p.data[1], under_glow_brightness);
    LEDManagement::set_ledDriver_brightness(driver_brightness);
    LEDManagement::set_underglow_brightness(under_glow_brightness);
    LEDManagement::set_updated(true);
  });

  callbacks.bind(MODE_LED, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received MODE_LED from %i ", p.header.device);
    LEDManagement::set_led_mode(p.data);
  });

  //TODO: SET_LED
  callbacks.bind(LED, empty_func);

  callbacks.bind(PALETTE_COLORS, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received PALETTE_COLORS from %i ", p.header.device);
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size);
  });

  callbacks.bind(LAYER_KEYMAP_COLORS, [](Packet const &p) {
    uint8_t layerIndex = p.data[0];
    DBG_PRINTF_TRACE("Received LAYER_KEYMAP_COLORS from %i %i ", p.header.device, layerIndex);
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
  });

  //Battery
  callbacks.bind(BATTERY_SAVING, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received BATTERY_SAVING from %i with value %i", p.header.device, p.data[0]);
    BatteryManagement::set_battery_saving(p.data[0]);
  });

  //RF MAC
  callbacks.bind(RF_ADDRESS, [](Packet const &p) {
    uint32_t rf_address{};
    memcpy(&rf_address, p.data, sizeof(rf_address));
    DBG_PRINTF_TRACE("Received RF_MAC_ADDRESS from %i with value %i", p.header.device, rf_address);
    if (rf_address != 0) {
      RFGWCommunication::setAddress(rf_address);
    };
  });

  queue_init(&txMessages, sizeof(Communications_protocol::Packet), 100);
  queue_init(&rxMessages, sizeof(Communications_protocol::Packet), 100);
}


bool Communications::sendPacket(Packet packet) {
  last_time_communication = to_ms_since_boot(get_absolute_time());
  if (packet.header.device == Communications_protocol::UNKNOWN) {
    packet.header.device = device;
  }
  packet.header.crc = 0;
  packet.header.crc = crc8(packet.buf, sizeof(Header) + packet.header.size);
  queue_add_blocking(&txMessages, &packet);
  return true;
}
#endif
