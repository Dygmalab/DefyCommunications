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
#include "RFLinkLayer.hpp"

constexpr uint8_t SIDE_ID = 25;
queue_t txMessages;
queue_t rxMessages;
Communications_protocol::Packet tx_message;
Communications_protocol::Packet rx_message;
Communications_protocol::Devices device;
bool need_polling;
uint32_t last_time_communication;
uint16_t keep_alive_timeout = 100;

//TODO: Create enum state for connections
uint8_t has_neuron_connection           = 0;
uint32_t last_time_communication_neuron = 0;
uint8_t has_rf_connection               = 0;
uint32_t last_time_communication_rf     = 0;

//TODO: Move crc8 to CRH file
uint8_t const crc_table[256] = {0x0, 0x7, 0xE, 0x9, 0x1C, 0x1B, 0x12, 0x15, 0x38, 0x3F, 0x36, 0x31, 0x24, 0x23, 0x2A, 0x2D, 0x70, 0x77, 0x7E, 0x79, 0x6C, 0x6B, 0x62, 0x65, 0x48, 0x4F, 0x46, 0x41, 0x54, 0x53, 0x5A, 0x5D, 0xE0, 0xE7, 0xEE, 0xE9, 0xFC, 0xFB, 0xF2, 0xF5, 0xD8, 0xDF, 0xD6, 0xD1, 0xC4, 0xC3, 0xCA, 0xCD, 0x90, 0x97, 0x9E, 0x99, 0x8C, 0x8B, 0x82, 0x85, 0xA8, 0xAF, 0xA6, 0xA1, 0xB4, 0xB3, 0xBA, 0xBD, 0xC7, 0xC0, 0xC9, 0xCE, 0xDB, 0xDC, 0xD5, 0xD2, 0xFF, 0xF8, 0xF1, 0xF6, 0xE3, 0xE4, 0xED, 0xEA, 0xB7, 0xB0, 0xB9, 0xBE, 0xAB, 0xAC, 0xA5, 0xA2, 0x8F, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9D, 0x9A, 0x27, 0x20, 0x29, 0x2E, 0x3B, 0x3C, 0x35, 0x32, 0x1F, 0x18, 0x11, 0x16, 0x3, 0x4, 0xD, 0xA, 0x57, 0x50, 0x59, 0x5E, 0x4B, 0x4C, 0x45, 0x42, 0x6F, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7D, 0x7A, 0x89, 0x8E, 0x87, 0x80, 0x95, 0x92, 0x9B, 0x9C, 0xB1, 0xB6, 0xBF, 0xB8, 0xAD, 0xAA, 0xA3, 0xA4, 0xF9, 0xFE, 0xF7, 0xF0, 0xE5, 0xE2, 0xEB, 0xEC, 0xC1, 0xC6, 0xCF, 0xC8, 0xDD, 0xDA, 0xD3, 0xD4, 0x69, 0x6E, 0x67, 0x60, 0x75, 0x72, 0x7B, 0x7C, 0x51, 0x56, 0x5F, 0x58, 0x4D, 0x4A, 0x43, 0x44, 0x19, 0x1E, 0x17, 0x10, 0x5, 0x2, 0xB, 0xC, 0x21, 0x26, 0x2F, 0x28, 0x3D, 0x3A, 0x33, 0x34, 0x4E, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5C, 0x5B, 0x76, 0x71, 0x78, 0x7F, 0x6A, 0x6D, 0x64, 0x63, 0x3E, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2C, 0x2B, 0x6, 0x1, 0x8, 0xF, 0x1A, 0x1D, 0x14, 0x13, 0xAE, 0xA9, 0xA0, 0xA7, 0xB2, 0xB5, 0xBC, 0xBB, 0x96, 0x91, 0x98, 0x9F, 0x8A, 0x8D, 0x84, 0x83, 0xDE, 0xD9, 0xD0, 0xD7, 0xC2, 0xC5, 0xCC, 0xCB, 0xE6, 0xE1, 0xE8, 0xEF, 0xFA, 0xFD, 0xF4, 0xF3};

uint8_t crc8(uint8_t const msg[], uint32_t nBytes) {
  uint8_t temp;
  temp = 0;
  for (int i = 0; i < nBytes; i++) {
    temp = crc_table[temp ^ msg[i]];
  }
  return temp;
}

class Communications Communications;

void cleanQueues() {
  while (!queue_is_empty(&txMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);
  }
  while (!queue_is_empty(&rxMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);
  }
}

void Communications::run() {
  uint32_t ms_since_enter = to_ms_since_boot(get_absolute_time());
  if (ms_since_enter - last_time_communication > keep_alive_timeout || need_polling || KeyScanner.newKey()) {
    last_time_communication = ms_since_enter;
    Packet packet{};
    if (KeyScanner.newKey() && !(!has_neuron_connection && !has_rf_connection)) {
      KeyScanner.keyState(false);
      packet.header.command = Communications_protocol::HAS_KEYS;
      packet.header.size    = KeyScanner.readMatrix(packet.data);
    } else {
      packet.header.command                  = IS_ALIVE;
      Configuration::StartInfo configuration = Configuration::get_configuration().start_info;
      configuration.spi_speed_base           = SPI::get_baudrate();
      configuration.cpu_speed                = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
      packet.header.size                     = sizeof(Configuration::StartInfo);
      memcpy(packet.data, &configuration, sizeof(configuration));
    }
    sendPacket(packet);
  }

  if (!queue_is_empty(&txMessages)) {
    queue_remove_blocking(&txMessages, &tx_message);

    if ((!has_neuron_connection && !has_rf_connection) || has_neuron_connection) {
      //Wired mode has priority
      SPI::read_write_buffer(SPI::CSList::CSN2, tx_message.buf, rx_message.buf, sizeof(Packet));
      //If we have a response then update the time.
      uint8_t rx_crc        = rx_message.header.crc;
      rx_message.header.crc = 0;
      uint8_t crc_8         = crc8(rx_message.buf, sizeof(Packet));
      if (rx_message.header.command != IS_DEAD && crc_8 == rx_crc) {
        last_time_communication_neuron = ms_since_enter;
        need_polling                   = rx_message.header.has_more_packets;
        callbacks.call(rx_message.header.command, rx_message);
        return;
      }
    }

    //If the communication with the neuron could not be established, try to send to the message to the RF
    //TODO: send message to RF
    if (RFLinkLayer::isRfConnected()) {
      //Send data to RF
    }
  }

  if (has_neuron_connection && ms_since_enter - last_time_communication_neuron > 900) {
    printf("Neuron disconnected\n");
    has_neuron_connection = false;
    LEDManagement::set_mode_disconnected();
    //Clean queue
    cleanQueues();
  }
  if (has_rf_connection && ms_since_enter - last_time_communication_rf > 900) {
    has_rf_connection = false;
    LEDManagement::set_mode_disconnected();
    //Clean queue
    cleanQueues();
  }
}

void Communications::init() {
  if (gpio_get(SIDE_ID)) {
    device = Communications_protocol::KEYSCANNER_DEFY_RIGHT;
  } else {
    device = Communications_protocol::KEYSCANNER_DEFY_LEFT;
  }

  auto empty_func = [](Packet p) {};

  callbacks.bind(IS_ALIVE, [this](Packet p) {
    if ((!has_neuron_connection && !has_rf_connection)) {
      if (p.header.device == Communications_protocol::WIRED_NEURON_DEFY) {
        has_neuron_connection = 1;
        printf("Wired Neuron is available to connect\n");
      }
      if (p.header.device == Communications_protocol::RF_NEURON_DEFY) {
        has_rf_connection = 1;
        printf("RF Neuron is available to connect\n");
      }
      if (p.header.device == Communications_protocol::NEURON_DEFY) {
        has_neuron_connection = 1;
        printf("Neuron is available to connect\n");
      }
      p.header.device  = device;
      p.header.command = Communications_protocol::CONNECTED;
      sendPacket(p);
    }
  });

  callbacks.bind(CONNECTED, [](Packet p) {
    has_neuron_connection = false;
    has_rf_connection     = false;

    if (p.header.device == Communications_protocol::RF_NEURON_DEFY) {
      has_rf_connection = 2;
      printf("RF connected\n");
      return;
    }
    if (p.header.device == Communications_protocol::NEURON_DEFY) {
      has_neuron_connection = 2;
      printf("Neuron connected\n");
      return;
    }
    if (p.header.device == Communications_protocol::WIRED_NEURON_DEFY) {
      has_neuron_connection = 2;
      printf("Wired Neuron connected\n");
      return;
    }
  });

  callbacks.bind(SLEEP, [](Packet p) {
    IS31FL3743B::set_enable(false);
    LEDManagement::set_enable_underGlow(false);
  });

  callbacks.bind(WAKE_UP, [](Packet p) {
    IS31FL3743B::set_enable(true);
    LEDManagement::set_enable_underGlow(true);
    LEDManagement::set_updated(true);
  });

  callbacks.bind(VERSION, [this](Packet p) {
    p.data[0]     = 1;
    p.header.size = 1;
    sendPacket(p);
  });

  //TODO: Has Keys loopback
  callbacks.bind(HAS_KEYS, [this](Packet p) {
    sendPacket(p);
  });

  callbacks.bind(SET_KEYSCAN_INTERVAL, [](Packet p) {
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

  //Config
  callbacks.bind(SET_ENABLE_LED_DRIVER, [](Packet p) {
    uint8_t enable;
    memcpy(&enable, &rx_message.data[0], sizeof(uint8_t));
    IS31FL3743B::set_enable(enable);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.led_driver_enabled     = enable;
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_ENABLE_UNDERGLOW, [](Packet p) {
    uint8_t enable;
    memcpy(&enable, &rx_message.data[0], sizeof(uint8_t));
    gpio_put(UG_EN, enable);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.underGlow_enabled      = enable;
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_ALIVE_INTERVAL, [](Packet p) {
    uint32_t pooling_rate_base;
    uint32_t pooling_rate_variation;
    memcpy(&pooling_rate_base, &rx_message.data[0], sizeof(uint32_t));
    memcpy(&pooling_rate_variation, &rx_message.data[sizeof(uint32_t)], sizeof(uint32_t));
    keep_alive_timeout                              = pooling_rate_base;
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.pooling_rate_base      = pooling_rate_base;
    configuration.start_info.pooling_rate_variation = pooling_rate_variation;
    printf("Sending alive interval base %lu and variation %lu\n", pooling_rate_base, pooling_rate_variation);
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_SPI_SPEED, [](Packet p) {
    uint32_t spi_speed_base;
    uint32_t spi_speed_variation;
    memcpy(&spi_speed_base, &rx_message.data[0], sizeof(uint32_t));
    memcpy(&spi_speed_variation, &rx_message.data[sizeof(uint32_t)], sizeof(uint32_t));
    SPI::set_baudrate(spi_speed_base);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.spi_speed_base         = spi_speed_base;
    configuration.start_info.spi_speed_variation    = spi_speed_variation;
    printf("Sending spi speed base %lu and variation %lu\n", spi_speed_base, spi_speed_variation);
    Configuration::set_configuration(configuration);
  });

  callbacks.bind(SET_CLOCK_SPEED, [](Packet p) {
    uint32_t cpu_speed;
    memcpy(&cpu_speed, &rx_message.data[0], sizeof(uint32_t));
    set_sys_clock_khz(cpu_speed, true);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.cpu_speed              = cpu_speed;
    printf("Setting cpuSpeed to %lu\n", cpu_speed);
    Configuration::set_configuration(configuration);
  });

  callbacks.bind(SET_LED_DRIVER_PULLUP, [](Packet p) {
    uint8_t led_driver_pull_up;
    memcpy(&led_driver_pull_up, &rx_message.data[0], sizeof(uint8_t));
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.pull_up_config         = led_driver_pull_up;
    printf("Setting ledDriver in left side to %i\n", led_driver_pull_up);
    Configuration::set_configuration(configuration);
    IS31FL3743B::setPullUpRegister(led_driver_pull_up);
  });

  queue_init(&txMessages, sizeof(Communications_protocol::Packet), 20);
  queue_init(&rxMessages, sizeof(Communications_protocol::Packet), 20);
}
bool Communications::sendPacket(Packet packet) {
  packet.header.device = device;
  packet.header.crc    = 0;
  packet.header.crc    = crc8(packet.buf, sizeof(Packet));
  queue_add_blocking(&txMessages, &packet);
  return true;
}
#endif
