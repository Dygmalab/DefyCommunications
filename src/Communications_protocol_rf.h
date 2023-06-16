#if defined(NRF52_ARCH) || defined(KEYSCANNER)
#ifndef KEYSCANNER_COMMUNICATIONS_PROTOCOL_RF_H_
#define KEYSCANNER_COMMUNICATIONS_PROTOCOL_RF_H_
#include "stdio.h"
#include "Communications_protocol.h"
#include "RFGateway.hpp"

namespace Communications_protocol_rf {

using delimiter_rf                               = uint16_t;
static const constexpr delimiter_rf DELIMITER_RF = 0x4450;
static const constexpr uint8_t MAX_TRANSFER_SIZE = sizeof(Communications_protocol::Packet) + sizeof(delimiter_rf);

union WrapperPacket {
  explicit WrapperPacket(const Communications_protocol::Packet &packet_)
    : packet(packet_) {
  }
  WrapperPacket(){};

  Communications_protocol::Packet &getPacket() { return packet; }

  uint16_t getSize() const { return packet.header.size + sizeof(delimiter_rf); }

  struct {
    delimiter_rf delimiter{DELIMITER_RF};
    Communications_protocol::Packet packet{};
  };
  uint8_t buf[MAX_TRANSFER_SIZE];
};


typedef struct
{
  /* Parse status code */
  rfgwp_status_code_t status_code;
  Communications_protocol::Commands pkt_cmd;
  uint8_t pkt_size;
  Communications_protocol::Packet packet;
} parse_t;

static inline result_t parseBuffer(parse_t *p_parse, buffer_t *p_buffer);

}  // namespace Communications_protocol_rf
#endif
#endif