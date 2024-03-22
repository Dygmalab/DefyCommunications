#if defined(NRF52_ARCH) || defined(KEYSCANNER)
#ifndef KEYSCANNER_COMMUNICATIONS_PROTOCOL_RF_H_
#define KEYSCANNER_COMMUNICATIONS_PROTOCOL_RF_H_
#include "stdio.h"
#include "Communications_protocol.h"

#ifdef KEYSCANNER
#include "RFGateway.hpp"
#endif


#ifdef NRF52_ARCH
#include "types.h"
#include "rf_gateway_api.h"
#include "memory/buffer.h"
#include "rfgw_protocol_types.h"
#endif

namespace Communications_protocol_rf {

using delimiter_rf                               = uint16_t;
static const constexpr delimiter_rf DELIMITER_RF = 0x5044;
static const constexpr uint8_t MAX_TRANSFER_SIZE = sizeof(Communications_protocol::Packet) + sizeof(delimiter_rf);

union WrapperPacket {
  explicit WrapperPacket(const Communications_protocol::Packet &packet_)
    : packet(packet_) {
  }
  WrapperPacket(){};

  Communications_protocol::Packet &getPacket() { return packet; }

  uint16_t getSize() const { return sizeof(packet.header) + packet.header.size + sizeof(delimiter_rf); }

  struct {
    delimiter_rf delimiter{DELIMITER_RF};
    Communications_protocol::Packet packet{};
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};


typedef enum {
  PARSE_STATUS_SUCCESS = 1,

  PARSE_STATUS_ERR_PACKET_SIZE,
  PARSE_STATUS_ERR_DELIMITER,
  PARSE_STATUS_ERR_CRC,
} parse_status_code_t;


typedef struct
{
  /* Parse status code */
  rfgwp_status_code_t status_code;
  Communications_protocol::Commands pkt_cmd;
  uint8_t pkt_size;
  WrapperPacket wrapperPacket;
} parse_t;

result_t parseBuffer(parse_t *p_parse, buffer_t *p_buffer);

}  // namespace Communications_protocol_rf
#endif
#endif
