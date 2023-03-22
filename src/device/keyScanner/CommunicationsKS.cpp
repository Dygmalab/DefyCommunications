#ifdef KEYSCANNER
#include <string.h>
#include "SPI.hpp"
#include "Config.hpp"
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
    IS31FL3743B::set_enable(false);
    LEDManagement::set_enable_underGlow(false);
  });

  callbacks.bind(WAKE_UP, [](Packet p) {
    IS31FL3743B::set_enable(true);
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

  //Config
  callbacks.bind(SET_ENABLE_LED_DRIVER, [](Packet p) {
    uint8_t enable;
    memcpy(&enable, &rx_message.data[0], sizeof(uint8_t));
    IS31FL3743B::set_enable(enable);
    Configuration::StartConfiguration configuration = Configuration::get_configuration();
    configuration.start_info.led_driver_enabled     = enable;
    Configuration::set_configuration(configuration);
  });
  callbacks.bind(SET_LAYER_UNDERGLOW_COLORS, [](Packet p) {
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
    keep_alive_timeout_neuron                       = pooling_rate_base;
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
