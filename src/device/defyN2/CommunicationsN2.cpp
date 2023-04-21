#ifdef NRF52_ARCH
#include "Communications.h"
#include "SpiPort.h"
#include "Time_counter.h"
#include "Usb_serial.h"
#include "rfgw_config_app.h"
#include "rf_gateway.h"
#include "CRC_wrapper.h"


extern Time_counter time_counter;

#if COMPILE_SPI0_SUPPORT
static SpiPort spiPort0(0);
Devices spiPort0Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort0LastCommunication{0};
#endif

#if COMPILE_SPI1_SUPPORT
static SpiPort spiPort1(1);
static Devices spiPort1Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort1LastCommunication{0};
#endif

#if COMPILE_SPI2_SUPPORT
static SpiPort spiPort2(2);
static Devices spiPort2Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort2LastCommunication{0};
#endif

constexpr static uint32_t timeout = 400;

void checkActive();

class RFGW_parser {
 public:
  static void init() {
    /* Initialize the RF Gateway */
    rfgw_init();

    /* Enable the RF Gateway */
    rfgw_enable();

    /* Open the Keyscanner pipes */
    rfgw_pipe_open(RFGW_PIPE_ID_KEYSCANNER_LEFT);
    rfgw_pipe_open(RFGW_PIPE_ID_KEYSCANNER_RIGHT);
  }


  static void run() {
    rfgw_poll();
    uint16_t pipe_send_loadsize = 0;
    uint16_t pipe_recv_loadsize = 0;

    rfgw_pipe_get_recv_loadsize(RFGW_PIPE_ID_KEYSCANNER_LEFT, &pipe_recv_loadsize);
    rfgw_pipe_get_send_freesize(RFGW_PIPE_ID_KEYSCANNER_LEFT, &pipe_send_loadsize);

    if (pipe_recv_loadsize) {
      left.connected             = true;
      left.lastTimeCommunication = time_counter.get_millis();
      Packet packet{};
      rfgw_pipe_recv(RFGW_PIPE_ID_KEYSCANNER_LEFT, packet.buf, pipe_recv_loadsize);
      Communications.callbacks.call(packet.header.command, packet);
      if (left.tx_messages.empty()) {
        packet                = {};
        packet.header.device  = Communications_protocol::RF_DEFY_LEFT;
        packet.header.command = Communications_protocol::IS_ALIVE;
        packet.header.size    = 0;
        packet.header.crc     = 0;
        left.tx_messages.push(packet);
      }
    }

    if (!left.tx_messages.empty()) {
      Communications_protocol::Packet &packet = left.tx_messages.front();
      if (pipe_send_loadsize>=sizeof(Header)+packet.header.size) {
        rfgw_pipe_send(RFGW_PIPE_ID_KEYSCANNER_LEFT, (uint8_t *)packet.buf, sizeof(Header)+packet.header.size);
        left.tx_messages.pop();
      }
    }
  }


  struct Side {
    Packet packet{};
    bool connected = false;
    uint32_t lastTimeCommunication{0};
    std::queue<Packet> tx_messages;
    void sendPacket(Packet &packet) {
      packet.header.device = Communications_protocol::RF_NEURON_DEFY;
      tx_messages.push(packet);
    };
  };
  static Side left;
};

RFGW_parser::Side RFGW_parser::left;


void Communications::init() {
#if COMPILE_SPI0_SUPPORT
  spiPort0.init();
#endif

#if COMPILE_SPI1_SUPPORT
  spiPort1.init();
#endif

#if COMPILE_SPI2_SUPPORT
  spiPort2.init();
#endif
  RFGW_parser::init();
}

void Communications::run() {
  Packet packet{};
#if COMPILE_SPI0_SUPPORT
  if (spiPort0.readPacket(packet)) {
    spiPort0LastCommunication = time_counter.get_millis();
    spiPort0Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort1.readPacket(packet)) {
    spiPort1Device            = packet.header.device;
    spiPort1LastCommunication = time_counter.get_millis();
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort2.readPacket(packet)) {
    spiPort2LastCommunication = time_counter.get_millis();
    spiPort2Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif
  RFGW_parser::run();
  checkActive();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  packet.header.device   = Communications_protocol::NEURON_DEFY;
  if (device_to_send == UNKNOWN) {
#if COMPILE_SPI0_SUPPORT
    if (spiPort0Device != UNKNOWN)
      spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
    if (spiPort1Device != UNKNOWN)
      spiPort1.sendPacket(packet);
#endif
#if COMPILE_SPI2_SUPPORT
    if (spiPort2Device != UNKNOWN)
      spiPort2.sendPacket(packet);
#endif
    if (RFGW_parser::left.connected)
      RFGW_parser::left.sendPacket(packet);
  }

#if COMPILE_SPI0_SUPPORT
  if (spiPort0Device == device_to_send)
    spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
  if (spiPort1Device == device_to_send)
    spiPort1.sendPacket(packet);
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device == device_to_send)
    spiPort2.sendPacket(packet);
#endif
  if (RFGW_parser::left.connected && device_to_send == Communications_protocol::RF_DEFY_LEFT)
    RFGW_parser::left.sendPacket(packet);

  return true;
}

void checkActive() {
  bool now_active;
  Packet packet;

#if COMPILE_SPI0_SUPPORT
  if (spiPort0Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort0LastCommunication <= timeout;
  if (!now_active) {
    spiPort0Device = UNKNOWN;
    while (spiPort0.readPacket(packet)) {}
    return;
  }
#endif

#if COMPILE_SPI1_SUPPORT
  if (spiPort1Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort1LastCommunication <= timeout;
  if (!now_active) {
    spiPort1Device = UNKNOWN;
    while (spiPort2.readPacket(packet)) {}
    return;
  }
#endif

#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort2LastCommunication <= timeout;
  if (!now_active) {
    spiPort2Device = UNKNOWN;
    while (spiPort2.readPacket(packet)) {}
    return;
  }
#endif
  now_active = time_counter.get_millis() - RFGW_parser::left.lastTimeCommunication <= timeout;
  if (!now_active) {
    RFGW_parser::left.connected = false;
    while (!RFGW_parser::left.tx_messages.empty()) {
      RFGW_parser::left.tx_messages.pop();
    }
  }
}


class Communications Communications;
#endif