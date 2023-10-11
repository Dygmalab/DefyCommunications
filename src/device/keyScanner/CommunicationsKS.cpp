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
#include "BatteryManagement.hpp"
#include "IS31FL3743B.hpp"
#include "Keyscanner.hpp"

constexpr uint8_t SIDE_ID = 25;
Communications_protocol::Devices device;

class Communications Communications;


bool verifyCrc(Packet &packet) {
  uint8_t rx_crc    = packet.header.crc;
  packet.header.crc = 0;
  uint8_t crc_8     = crc8(packet.buf, sizeof(Header) + packet.header.size);
  packet.header.crc = rx_crc;
  return crc_8 == rx_crc;
}

void calculateCRC(Packet &packet) {
  packet.header.crc = 0;
  packet.header.crc = crc8(packet.buf, sizeof(Header) + packet.header.size);
}

class WiredCommunication {
 public:
  static void init() {
    Communications.callbacks.bind(IS_ALIVE, [](Packet p) {
      if (!WiredCommunication::connected) return;
      if (WiredCommunication::connectionEstablished) return;
      if (p.header.device == Communications_protocol::WIRED_NEURON_DEFY || p.header.device == Communications_protocol::NEURON_DEFY) {
        p.header.device  = device;
        p.header.command = Communications_protocol::CONNECTED;
        p.header.size    = 0;
        WiredCommunication::sendPacket(p);
        DBG_PRINTF_TRACE("Wired Neuron is available to connect");
      }
    });

    Communications.callbacks.bind(CONNECTED, [](Packet const &p) {
      if (!WiredCommunication::connected) return;
      if (WiredCommunication::connectionEstablished) return;
      if (p.header.device == Communications_protocol::NEURON_DEFY || p.header.device == Communications_protocol::WIRED_NEURON_DEFY) {
        DBG_PRINTF_TRACE("Neuron wired connected");
        WiredCommunication::connectionEstablished = true;
        if (RFGWCommunication::isEnabled())
          RFGateway::rf_disable();
      }
    });
  }
  static void run() {
    checkConnection();
    pollConnection();
  }

  static void checkConnection() {
    if (connected) return;
    const constexpr uint16_t keep_alive_timeout = 500;
    uint32_t ms_since_enter                     = to_ms_since_boot(get_absolute_time());
    if (ms_since_enter - last_time_communication > keep_alive_timeout) {
      last_time_communication = ms_since_enter;
      Packet sending{};
      sending.header.command = IS_ALIVE;
      sending.header.device  = device;
      Packet response{};
      SPI::read_write_buffer(SPI::CSList::CSN2, sending.buf, response.buf, sizeof(Packet));
      if (response.header.command != IS_DEAD && verifyCrc(response)) {
        connected = true;
        Communications.callbacks.call(response.header.command, response);
      }
    }
  }

  static void pollConnection() {
    if (!connectionEstablished) return;
    const constexpr uint16_t keep_alive_timeout = 100;
    uint32_t ms_since_enter                     = to_ms_since_boot(get_absolute_time());
    if (keepPooling || ms_since_enter - last_time_communication > keep_alive_timeout) {
      last_time_communication = ms_since_enter;
      Packet sending{};
      sending.header.command = IS_ALIVE;
      sendPacket(sending);
    }
  }


  static bool sendPacket(Packet &sending) {
    last_time_communication = to_ms_since_boot(get_absolute_time());

    Packet response{};
    sending.header.device = device;
    calculateCRC(sending);
    SPI::read_write_buffer(SPI::CSList::CSN2, sending.buf, response.buf, sizeof(Packet));

    //This should only happen if there is a disconnection
    if (response.header.command == IS_DEAD) {
      connected             = false;
      connectionEstablished = false;
      DBG_PRINTF_TRACE("Wired disconnected");
      if (!RFGWCommunication::isEnabled())
        RFGateway::rf_device_type_set(RFGateway::RFGW_RF_DEV_TYPE_DEVICE);
      LEDManagement::set_mode_disconnected();
      return false;
    }

    WiredCommunication::keepPooling = response.header.has_more_packets;

    if (verifyCrc(response)) {
      Communications.callbacks.call(response.header.command, response);
      return true;
    }

    return false;
  }

  inline static auto connected                   = false;
  inline static auto connectionEstablished       = false;
  inline static bool keepPooling                 = false;
  inline static uint32_t last_time_communication = 0;
};

void Communications::run() {
  WiredCommunication::run();
  RFGWCommunication::run();
}


void Communications::init() {

  if (gpio_get(SIDE_ID)) {
    device = Communications_protocol::KEYSCANNER_DEFY_RIGHT;
  } else {
    device = Communications_protocol::KEYSCANNER_DEFY_LEFT;
  }

  auto empty_func = [](Packet const &) {};


  callbacks.bind(SLEEP, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received SLEEP from %i", p.header.device);
    LEDManagement::turnPowerOff();
    BatteryManagement::goToSleep();
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
    float driver_brightness     = BatteryManagement::mapRange(p.data[0], 0, 255, 0, 0.8);
    float under_glow_brightness = BatteryManagement::mapRange(p.data[1], 0, 255, 0, 0.3);
    LEDManagement::set_max_ledDriver_brightness(driver_brightness);
    LEDManagement::set_max_underglow_brightness(under_glow_brightness);
    //TODO: if saving mode is on dont set the led brightness.
    LEDManagement::set_ledDriver_brightness(driver_brightness);
    LEDManagement::set_underglow_brightness(under_glow_brightness);
    LEDManagement::set_updated(true);
    DBG_PRINTF_TRACE("Received BRIGHTNESS from %i values %i %f %i %f", p.header.device, p.data[0], driver_brightness, p.data[1], under_glow_brightness);
    DBG_PRINTF_TRACE("BRIGHTNESS after setting it: %f", driver_brightness);
  });

  callbacks.bind(MODE_LED, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received MODE_LED from %i ", p.header.device);
    LEDManagement::set_led_mode(p.data);
  });

  //TODO: SET_LED
  callbacks.bind(LED, empty_func);

  callbacks.bind(PALETTE_COLORS, [](Packet const &p) {
    DBG_PRINTF_TRACE("Received PALETTE_COLORS from %i ", p.header.device);
    memcpy(&LEDManagement::palette[p.data[0]], &p.data[1], p.header.size - 1);
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

  WiredCommunication::init();
  RFGWCommunication::init();
}


bool Communications::sendPacket(Packet packet) {
  if (WiredCommunication::connectionEstablished)
    return WiredCommunication::sendPacket(packet);
  if (RFGWCommunication::connectionEstablished)
    return RFGWCommunication::sendPacket(packet);
  return false;
}
#endif
