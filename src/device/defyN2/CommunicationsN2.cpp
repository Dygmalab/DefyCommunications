#ifdef NRF52_ARCH
#include "Communications.h"
#include "Communications_protocol_rf.h"
#include "SpiPort.h"
#include "rfgw_config_app.h"
#include "rf_gateway.h"
#include "CRC_wrapper.h"
#include "Arduino.h"
#include "Adafruit_USBD_Device.h"


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
    if (!TinyUSBDevice.ready()) return;
    rfgw_poll();
    left.run();
    right.run();
  }


  struct Side {
    explicit Side(rfgw_pipe_id_t pipe)
      : pipe_id(pipe) {}

    void parseErrProcess(buffer_t *p_buffer) {
      /*
     * TODO: Possible parse error should be handled here by letting the upper layer know about the response failure
     */

      /* Initiate the search for new packet by skipping the first byte */
      buffer_update_read_pos(p_buffer, 1);
    }

    void parseOkProcess(Communications_protocol_rf::parse_t *p_parse, buffer_t *p_buffer) {
      if (p_parse->status_code == Communications_protocol_rf::PARSE_STATUS_SUCCESS) {
        connected = true;
        NRF_LOG_DEBUG("Got packet took %i %i %i %i", millis() - lastTimeCommunication, p_parse->wrapperPacket.packet.header.device, p_parse->pkt_cmd, pipe_id);
        lastTimeCommunication = millis();
        Communications.callbacks.call(p_parse->pkt_cmd, p_parse->wrapperPacket.packet);
        /* Discard the already processed packet */
        buffer_update_read_pos(p_buffer, p_parse->pkt_size);
      } else /* Packet parse failed with clear status code */
      {
        /*
         * TODO: Possible parse error should be handled here by letting the upper layer know about the response failure
         */

        parseErrProcess(p_buffer);
      }
    }

    void parseProcess(void) {
      result_t result = RESULT_ERR;
      buffer_t *p_buffer_in;
      Communications_protocol_rf::parse_t parse;

      /* Get the buffer IN */
      result = rfgw_pipe_recv_buffer_get(pipe_id, &p_buffer_in);
      EXIT_IF_NOK(result);
      //ASSERT_DYGMA(result == RESULT_OK, "rf_pipe_recv_buffer_get failed");

      /* Parse and process the incoming data */
      result = parseBuffer(&parse, p_buffer_in);

      switch (result) {
      case RESULT_OK:

        parseOkProcess(&parse, p_buffer_in);

        break;

      case RESULT_ERR: /* Packet parse failed with un-clear status code */

        parseErrProcess(p_buffer_in);

        break;

      default:

        /*
             * No or incomplete packet.
             */

        break;
      }

    _EXIT:
      return;
    }

    void run() {
      uint16_t pipe_send_loadsize = 0;
      uint16_t pipe_recv_loadsize = 0;

      rfgw_pipe_get_send_freesize(pipe_id, &pipe_send_loadsize);
      rfgw_pipe_get_recv_loadsize(pipe_id, &pipe_recv_loadsize);
      parseProcess();

      if (tx_messages.empty() && millis() - lastTimeCommunicationSend > 150 && connected) {
        Packet packet{};
        packet.header.command = Communications_protocol::IS_ALIVE;
        sendPacket(packet);
      }

      if (!tx_messages.empty()) {
        Communications_protocol_rf::WrapperPacket &packet = tx_messages.front();
        uint16_t size_to_transfer                         = packet.getSize();
        if (pipe_send_loadsize >= size_to_transfer) {
          //          NRF_LOG_DEBUG("Sending packet took %i %i %i %i",millis() - lastTimeCommunicationSend,packet.packet.header.device,packet.packet.header.command,pipe_id);
          rfgw_pipe_send(pipe_id, packet.buf, size_to_transfer);
          tx_messages.pop();
          lastTimeCommunicationSend = millis();
        }
      }
    }

    bool connected = false;
    rfgw_pipe_id_t pipe_id;
    uint64_t lastTimeCommunication{0};
    uint64_t lastTimeCommunicationSend{0};
    std::queue<Communications_protocol_rf::WrapperPacket> tx_messages;
    void sendPacket(Packet &packet) {
      if (!TinyUSBDevice.ready()) return;
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
    spiPort0LastCommunication = millis();
    spiPort0Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort1.readPacket(packet)) {
    spiPort1Device            = packet.header.device;
    spiPort1LastCommunication = millis();
    callbacks.call(packet.header.command, packet);
  }
#endif
#if COMPILE_SPI2_SUPPORT
  if (spiPort2.readPacket(packet)) {
    spiPort2LastCommunication = millis();
    spiPort2Device            = packet.header.device;
    callbacks.call(packet.header.command, packet);
  }
#endif
  RFGW_parser::run();
  checkActive();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  if (device_to_send == UNKNOWN) {
    if (TinyUSBDevice.ready()) {
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
      if (spiPort1Device != UNKNOWN) {
        packet.header.device = Communications_protocol::NEURON_DEFY;
        spiPort1.sendPacket(packet);
      }
    }


    if (RFGW_parser::right.connected)
      RFGW_parser::right.sendPacket(packet);
    if (RFGW_parser::left.connected)
      RFGW_parser::left.sendPacket(packet);
    return true;
  }

  if (TinyUSBDevice.ready()) {

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
    //This could even been improved by checking if the spi port1 is connected and only sending the message to spiport2
    if (spiPort2Device != UNKNOWN) {
      packet.header.device = device_to_send;
      spiPort2.sendPacket(packet);
    }

    if (spiPort1Device == device_to_send) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort1.sendPacket(packet);
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
  if (spiPort0Device != UNKNOWN) {
    now_active = millis() - spiPort0LastCommunication <= timeout;
    if (!now_active) {
      spiPort0Device = UNKNOWN;
      while (spiPort0.readPacket(packet)) {}
    }
  }
#endif

#if COMPILE_SPI1_SUPPORT
  if (spiPort1Device != UNKNOWN) {
    now_active = millis() - spiPort1LastCommunication <= TIMEOUT;
    if (!now_active) {
      packet.header.command = Communications_protocol::DISCONNECTED;
      packet.header.device  = spiPort1Device;
      Communications.callbacks.call(packet.header.command, packet);
      spiPort1Device = UNKNOWN;
      //Remove all the left packets at disconnections
      while (spiPort1.readPacket(packet)) {}
    }
  }
#endif

#if COMPILE_SPI2_SUPPORT
  if (spiPort2Device != UNKNOWN) {
    now_active = millis() - spiPort2LastCommunication <= TIMEOUT;
    if (!now_active) {
      packet.header.command = Communications_protocol::DISCONNECTED;
      packet.header.device  = spiPort1Device;
      Communications.callbacks.call(packet.header.command, packet);
      spiPort2Device = UNKNOWN;
      //Remove all the left packets at disconnections
      while (spiPort2.readPacket(packet)) {}
    }
  }
#endif
  if (RFGW_parser::left.connected) {
    now_active = millis() - RFGW_parser::left.lastTimeCommunication <= TIMEOUT;
    if (!now_active) {
      packet.header.command = Communications_protocol::DISCONNECTED;
      packet.header.device  = Communications_protocol::RF_DEFY_LEFT;
      Communications.callbacks.call(packet.header.command, packet);
      RFGW_parser::left.connected = false;
      while (!RFGW_parser::left.tx_messages.empty()) {
        RFGW_parser::left.tx_messages.pop();
      }
    }
  }

  if (RFGW_parser::right.connected) {
    now_active = millis() - RFGW_parser::right.lastTimeCommunication <= TIMEOUT;
    if (!now_active) {
      packet.header.command = Communications_protocol::DISCONNECTED;
      packet.header.device  = Communications_protocol::RF_DEFY_RIGHT;
      Communications.callbacks.call(packet.header.command, packet);
      RFGW_parser::right.connected = false;
      while (!RFGW_parser::right.tx_messages.empty()) {
        RFGW_parser::right.tx_messages.pop();
      }
    }
  }
}


class Communications Communications;
#endif
