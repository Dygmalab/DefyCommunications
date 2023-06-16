#ifdef NRF52_ARCH
#include "Communications.h"
#include "Communications_protocol_rf.h"
#include "SpiPort.h"
#include "Time_counter.h"
#include "Usb_serial.h"
#include "rfgw_config_app.h"
#include "rf_gateway.h"
#include "CRC_wrapper.h"

extern Usb_serial usb_serial;  // It is declared in main.cpp

extern Time_counter time_counter;

constexpr static uint8_t DELIMITER{0b10101010};

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

static uint32_t TIMEOUT = 1300;

void checkActive();

class RFGW_parser {
 public:
  static void run() {
    if (!usbd_ready()) return;
    rfgw_poll();
    left.run();
    right.run();
  }


  struct Side {
    explicit Side(rfgw_pipe_id_t pipe)
      : pipe_id(pipe) {}

    void run() {
      uint16_t pipe_send_loadsize = 0;
      uint16_t pipe_recv_loadsize = 0;

      rfgw_pipe_get_recv_loadsize(pipe_id, &pipe_recv_loadsize);
      rfgw_pipe_get_send_freesize(pipe_id, &pipe_send_loadsize);

      if (pipe_recv_loadsize) {
        connected             = true;
        lastTimeCommunication = time_counter.get_millis();
        rfgw_pipe_recv(pipe_id, &rx_buffer[rx_buffer_last_index], pipe_recv_loadsize);
        rx_buffer_last_index += pipe_recv_loadsize;
        checkBuffer();
        if (tx_messages.empty()) {
          Packet packet{};
          packet.header.command = Communications_protocol::IS_ALIVE;
          sendPacket(packet);
        }
      }

      if (!tx_messages.empty()) {
        Communications_protocol_rf::WrapperPacket &packet = tx_messages.front();
        uint16_t size_to_transfer                         = packet.getSize();
        if (pipe_send_loadsize >= size_to_transfer) {
          rfgw_pipe_send(pipe_id, packet.buf, size_to_transfer);
          tx_messages.pop();
        }
      }
    }

    void checkBuffer() {
      uint32_t i = 0;
      do {
        if (rx_buffer[i++] == DELIMITER) {
          Packet packet{};
          //TODO: refactor the check of the max transfer size
          if (i - 1 > MAX_TRANSFER_SIZE) {
            //Something happend in the transfer left clear the buffer
            memset(rx_buffer, 0, i);
            rx_buffer_last_index -= i;
            memmove(rx_buffer, &rx_buffer[i], rx_buffer_last_index);
            memset(&rx_buffer[rx_buffer_last_index], 0, rx_buffer_last_index + i);
            i = 0;
            continue;
          }
          memcpy(packet.buf, rx_buffer, i);
          uint8_t check_crc = packet.header.crc;
          packet.header.crc = 0;
          if (check_crc == crc8(packet.buf, sizeof(Header) + packet.header.size)) {
            Communications.callbacks.call(packet.header.command, packet);
            memset(rx_buffer, 0, i);
            rx_buffer_last_index -= i;
            memmove(rx_buffer, &rx_buffer[i], rx_buffer_last_index);
            memset(&rx_buffer[rx_buffer_last_index], 0, rx_buffer_last_index + i);
            i = 0;
          }
        }
      } while (i < rx_buffer_last_index);
    }

    bool connected = false;
    rfgw_pipe_id_t pipe_id;
    uint32_t lastTimeCommunication{0};
    uint8_t rx_buffer[512]{};
    uint16_t rx_buffer_last_index{0};
    std::queue<Communications_protocol_rf::WrapperPacket> tx_messages;
    void sendPacket(Packet &packet) {
      if (!usbd_ready()) return;
      packet.header.crc    = 0;
      packet.header.device = Communications_protocol::RF_NEURON_DEFY;
      packet.header.crc    = crc8(packet.buf, sizeof(Header) + packet.header.size);
      tx_messages.emplace(packet);
    };
  };
  static Side left;
  static Side right;
};

RFGW_parser::Side RFGW_parser::left(RFGW_PIPE_ID_KEYSCANNER_LEFT);
RFGW_parser::Side RFGW_parser::right(RFGW_PIPE_ID_KEYSCANNER_RIGHT);


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

  callbacks.bind(CONNECTED, [this](Packet p) {
    p.header.size    = 0;
    p.header.device  = p.header.device;
    p.header.command = CONNECTED;
    sendPacket(p);
  });
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
  if (time_counter.get_millis() % 300 == 0) {
    packet.header.command = IS_ALIVE;
    packet.header.size    = 1;
    RFGW_parser::left.sendPacket(packet);
  }
  RFGW_parser::run();
  checkActive();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  if (device_to_send == UNKNOWN) {
    if (usbd_ready()) {
#if COMPILE_SPI0_SUPPORT
      if (spiPort0Device != UNKNOWN)
        spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
      if (spiPort1Device != UNKNOWN) {
        packet.header.device = Communications_protocol::NEURON_DEFY;
        spiPort1.sendPacket(packet);
      }
#endif
#if COMPILE_SPI2_SUPPORT
      if (spiPort2Device != UNKNOWN) {
        packet.header.device = Communications_protocol::NEURON_DEFY;
        spiPort2.sendPacket(packet);
      }
#endif
    } else {
      if (spiPort2Device != UNKNOWN) {
        packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        spiPort2.sendPacket(packet);
      }
    }


    if (RFGW_parser::right.connected)
      RFGW_parser::right.sendPacket(packet);
    if (RFGW_parser::left.connected)
      RFGW_parser::left.sendPacket(packet);
  }

  if (usbd_ready()) {

#if COMPILE_SPI0_SUPPORT
    if (spiPort0Device == device_to_send)
      spiPort0.sendPacket(packet);
#endif
#if COMPILE_SPI1_SUPPORT
    if (spiPort1Device == device_to_send) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort1.sendPacket(packet);
    }
#endif
#if COMPILE_SPI2_SUPPORT
    if (spiPort2Device == device_to_send) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort2.sendPacket(packet);
    }
#endif
  } else {
    if (spiPort2Device != UNKNOWN) {
      packet.header.device = device_to_send;
      spiPort2.sendPacket(packet);
    }
    //No need to continue RF is disabled in ble mode
    return true;
  }

  if (RFGW_parser::left.connected && device_to_send == Communications_protocol::RF_DEFY_LEFT)
    RFGW_parser::left.sendPacket(packet);


  if (RFGW_parser::right.connected && device_to_send == Communications_protocol::RF_DEFY_RIGHT)
    RFGW_parser::right.sendPacket(packet);

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
  now_active = time_counter.get_millis() - spiPort1LastCommunication <= TIMEOUT;
  if (!now_active) {
    spiPort1Device = UNKNOWN;
    //Remove all the left packets at disconnections
    while (spiPort1.readPacket(packet)) {}
    return;
  }
#endif

#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device == UNKNOWN)
    return;
  now_active = time_counter.get_millis() - spiPort2LastCommunication <= TIMEOUT;
  if (!now_active) {
    spiPort2Device = UNKNOWN;
    //Remove all the left packets at disconnections
    while (spiPort2.readPacket(packet)) {}
    return;
  }
#endif
  now_active = time_counter.get_millis() - RFGW_parser::left.lastTimeCommunication <= TIMEOUT;
  if (!now_active) {
    RFGW_parser::left.connected = false;
    while (!RFGW_parser::left.tx_messages.empty()) {
      RFGW_parser::left.tx_messages.pop();
    }
  }

  now_active = time_counter.get_millis() - RFGW_parser::right.lastTimeCommunication <= TIMEOUT;
  if (!now_active) {
    RFGW_parser::right.connected = false;
    while (!RFGW_parser::right.tx_messages.empty()) {
      RFGW_parser::right.tx_messages.pop();
    }
  }
}


class Communications Communications;
#endif