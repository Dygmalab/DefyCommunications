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
#include "RFGW_communications.h"
#include "CRC.h"

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
      //Wired mode has more priority
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
    if (RFGWCommunication::isEnabled()) {
      RFGWCommunication::sendPacket(tx_message);
    }
  }

  if (RFGWCommunication::isEnabled()) {
    if (RFGWCommunication::hasPackets()) {
      rx_message            = RFGWCommunication::getPacket();
      uint8_t rx_crc        = rx_message.header.crc;
      rx_message.header.crc = 0;
      uint8_t crc_8         = crc8(rx_message.buf, sizeof(Packet));
      if (rx_message.header.command != IS_DEAD && crc_8 == rx_crc) {
        last_time_communication_rf = ms_since_enter;
        need_polling               = rx_message.header.has_more_packets;
        callbacks.call(rx_message.header.command, rx_message);
        return;
      }
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
    RFGWCommunication::cleanMessages();
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
      uint32_t version = FMW_VERSION;
      memcpy(p.data, &version, sizeof(version));
      p.header.size = sizeof(version);
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
    uint32_t version = FMW_VERSION;
    memcpy(p.data, &version, sizeof(version));
    p.header.size = sizeof(version);
    sendPacket(p);
  });

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
