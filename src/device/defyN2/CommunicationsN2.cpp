#ifdef NRF52_ARCH
#include "Communications.h"
#include "Communications_protocol_rf.h"
#include "SpiPort.h"
#include "rfgw_config_app.h"
#include "rf_gateway.h"
#include "CRC_wrapper.h"
#include "Arduino.h"
#include "Adafruit_USBD_Device.h"
#include "Radio_manager.h"


static SpiPort spiPort1(1);
static Devices spiPort1Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort1LastCommunication{0};

static SpiPort spiPort2(2);
static Devices spiPort2Device{Communications_protocol::UNKNOWN};
static uint32_t spiPort2LastCommunication{0};
static uint32_t TIMEOUT = 300;

void checkActive();

class RFGWCommunications {
 public:
  static void cbPipeDisconnection(rfgw_pipe_id_t pipeId) {
//    NRF_LOG_DEBUG("Disconnected RF %lu", pipeId);
    RFGWCommunications::Side &side = pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ? right : left;
    side.connected                 = false;
    Packet packet{};
    packet.header.command = Communications_protocol::DISCONNECTED;
    packet.header.device  = pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ? RF_DEFY_RIGHT : RF_DEFY_LEFT;
    Communications.callbacks.call(packet.header.command, packet);

    while (!side.tx_messages.empty()) {
      side.tx_messages.pop();
    }
  };

  static void cbPipeConnection(rfgw_pipe_id_t pipeId) {
    RFGWCommunications::Side &side = pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ? right : left;
    side.connected                 = true;
//    NRF_LOG_DEBUG("Connected RF %lu", pipeId);
    Packet packet{};
    packet.header.command = Communications_protocol::CONNECTED;
    packet.header.device  = pipeId == RFGW_PIPE_ID_KEYSCANNER_RIGHT ? RF_DEFY_RIGHT : RF_DEFY_LEFT;
    Communications.callbacks.call(packet.header.command, packet);
  };

  static void init() {
    rfgw_cb_pipe_disconnection_set(cbPipeDisconnection);
    rfgw_cb_pipe_connection_set(cbPipeConnection);

    Communications.callbacks.bind(IS_ALIVE, [](Packet p) {
      if (p.header.device == RF_DEFY_LEFT || p.header.device == RF_DEFY_RIGHT) {
        p.header.size    = 0;
        p.header.device  = p.header.device;
        p.header.command = IS_ALIVE;
        Communications.sendPacket(p);
      }
    });
  }

  static void run() {
    if (!kaleidoscope::plugin::RadioManager::isInited()) return;
    kaleidoscope::plugin::RadioManager::poll();
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

      if (!tx_messages.empty()) {
        Communications_protocol_rf::WrapperPacket &packet = tx_messages.front();
        uint16_t size_to_transfer                         = packet.getSize();
        if (pipe_send_loadsize >= size_to_transfer) {
          rfgw_pipe_send(pipe_id, packet.buf, size_to_transfer);
          tx_messages.pop();
        }
      }
    }

    bool connected = false;
    rfgw_pipe_id_t pipe_id;
    std::queue<Communications_protocol_rf::WrapperPacket> tx_messages;
    void sendPacket(Packet &packet) {
      if (!kaleidoscope::plugin::RadioManager::isInited()) return;
      packet.header.crc    = 0;
      packet.header.device = Communications_protocol::RF_NEURON_DEFY;
      packet.header.crc    = crc8(packet.buf, sizeof(Header) + packet.header.size);
      tx_messages.emplace(packet);
    };
  };
  static Side left;
  static Side right;
};

RFGWCommunications::Side RFGWCommunications::left(RFGW_PIPE_ID_KEYSCANNER_LEFT);
RFGWCommunications::Side RFGWCommunications::right(RFGW_PIPE_ID_KEYSCANNER_RIGHT);


class WiredCommunications {
 public:
  static void init() {
    spiPort1.init();
    spiPort2.init();
  }

  static void readPacket(uint8_t port) {
    SpiPort &spiPort             = port == 1 ? spiPort1 : spiPort2;
    Devices &device              = port == 1 ? spiPort1Device : spiPort2Device;
    uint32_t &lastCommunications = port == 1 ? spiPort1LastCommunication : spiPort2LastCommunication;
    Packet packet{};
    if (spiPort.readPacket(packet)) {
      lastCommunications = millis();
      device             = packet.header.device;
      Communications.callbacks.call(packet.header.command, packet);
    }
  }

  static void checkActive(uint8_t port) {
    SpiPort &spiPort                   = port == 1 ? spiPort1 : spiPort2;
    Devices &device                    = port == 1 ? spiPort1Device : spiPort2Device;
    uint32_t const &lastCommunications = port == 1 ? spiPort1LastCommunication : spiPort2LastCommunication;

    bool now_active;
    Packet packet;

    if (device != UNKNOWN) {
      now_active = millis() - lastCommunications <= TIMEOUT;
      if (!now_active) {
        packet.header.command = Communications_protocol::DISCONNECTED;
        packet.header.device  = device;
        Communications.callbacks.call(packet.header.command, packet);
        device = UNKNOWN;
        //Remove all the left packets at disconnections
        spiPort.clearRead();
        spiPort.clearSend();
      }
    }
  }

  static void run() {
    readPacket(1);
    checkActive(1);

    readPacket(2);
    checkActive(2);
  }
};

void Communications::init() {
  callbacks.bind(CONNECTED, [this](Packet p) {
    p.header.size    = 0;
    p.header.device  = p.header.device;
    p.header.command = CONNECTED;
    //NRF_LOG_INFO("Get connected from %i", p.header.device);
    sendPacket(p);
  });


  WiredCommunications::init();


  RFGWCommunications::init();
}

void Communications::run() {
  WiredCommunications::run();

  RFGWCommunications::run();
}

bool Communications::sendPacket(Packet packet) {
  Devices device_to_send = packet.header.device;
  if (device_to_send == UNKNOWN) {
    if (!ble_innited()) {
      if (spiPort1Device != UNKNOWN) {
        packet.header.device = Communications_protocol::NEURON_DEFY;
        spiPort1.sendPacket(packet);
      }
      if (spiPort2Device != UNKNOWN) {
        packet.header.device = Communications_protocol::NEURON_DEFY;
        spiPort2.sendPacket(packet);
      }
    } else {
      if (spiPort2Device != UNKNOWN) {
        if (device_to_send != BLE_DEFY_RIGHT && device_to_send != BLE_DEFY_LEFT) {
          packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        }
        if (device_to_send == UNKNOWN) {
          packet.header.device = Communications_protocol::UNKNOWN;
        }
        spiPort2.sendPacket(packet);
      }
      if (spiPort1Device != UNKNOWN) {
        if (device_to_send != BLE_DEFY_RIGHT && device_to_send != BLE_DEFY_LEFT) {
          packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        }
        if (device_to_send == UNKNOWN) {
          packet.header.device = Communications_protocol::UNKNOWN;
        }
        spiPort1.sendPacket(packet);
      }
    }


    if (RFGWCommunications::right.connected)
      RFGWCommunications::right.sendPacket(packet);
    if (RFGWCommunications::left.connected)
      RFGWCommunications::left.sendPacket(packet);
    return true;
  }

  if (!ble_innited()) {

    if (spiPort1Device == device_to_send) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort1.sendPacket(packet);
    }
    if (spiPort2Device == device_to_send) {
      packet.header.device = Communications_protocol::NEURON_DEFY;
      spiPort2.sendPacket(packet);
    }
  } else {
    //If both of then are connected we just want to use the wired connection in both cases
    if (spiPort2Device != UNKNOWN && spiPort1Device != UNKNOWN) {
      if ((spiPort1Device == KEYSCANNER_DEFY_LEFT && device_to_send == KEYSCANNER_DEFY_LEFT) || (spiPort1Device == KEYSCANNER_DEFY_RIGHT && device_to_send == KEYSCANNER_DEFY_RIGHT)) {
        packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        spiPort1.sendPacket(packet);
      }
      if ((spiPort2Device == KEYSCANNER_DEFY_LEFT && device_to_send == KEYSCANNER_DEFY_LEFT) || (spiPort2Device == KEYSCANNER_DEFY_RIGHT && device_to_send == KEYSCANNER_DEFY_RIGHT)) {
        packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        spiPort2.sendPacket(packet);
      }
    } else {
      if (spiPort2Device != UNKNOWN) {
        if (device_to_send != BLE_DEFY_RIGHT && device_to_send != BLE_DEFY_LEFT) {
          packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        }
        spiPort2.sendPacket(packet);
      }

      if (spiPort1Device != UNKNOWN) {
        if (device_to_send != BLE_DEFY_RIGHT && device_to_send != BLE_DEFY_LEFT) {
          packet.header.device = Communications_protocol::BLE_NEURON_2_DEFY;
        }
        spiPort1.sendPacket(packet);
      }
    }
    //No need to continue RF is disabled in ble mode
    return true;
  }

  if (RFGWCommunications::left.connected && device_to_send == Communications_protocol::RF_DEFY_LEFT)
    RFGWCommunications::left.sendPacket(packet);


  if (RFGWCommunications::right.connected && device_to_send == Communications_protocol::RF_DEFY_RIGHT)
    RFGWCommunications::right.sendPacket(packet);

  return true;
}


class Communications Communications;
#endif
