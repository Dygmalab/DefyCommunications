/*
 * Copyright (C) 2024  Dygma Lab S.L.
 *
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Author: Gustavo Gomez Lopez @Noteolvides
 *
 */

#if defined(NRF52_ARCH) || defined(KEYSCANNER)

#include "Communications_protocol_rf.h"

#ifdef KEYSCANNER
#include "CRC.h"
#endif

#ifdef NRF52_ARCH
#include "CRC_wrapper.h"
#endif


/********************** Parsing **********************/

/*
 * Suggestions:
 *  - Move the Delimiter to the header to act as the the SOF. In such case, let's make it at least 16-bit. This would help with
 *    identifying the invalid packets much faster.
 *  - Make typedefs for the packet components. This will make it easier in the future if the protocol needs to change. For example the
 *    packet size change from the 8-bit to 16-bit.
 */

#define PKT_CRC_SIZE         sizeof(uint8_t)
#define PKT_DELIMITER_SIZE   sizeof(Communications_protocol_rf::delimiter_rf)
#define PKT_HEADER_SIZE      (sizeof(Communications_protocol::Header))
#define PKT_FULL_HEADER_SIZE (PKT_DELIMITER_SIZE + PKT_HEADER_SIZE)


typedef struct
{
  /* Parse status code */
  rfgwp_status_code_t status_code;

  Communications_protocol::Commands pkt_cmd;
  uint8_t pkt_size;
  Communications_protocol_rf::WrapperPacket wrapperPacket;
} parse_t;

static result_t _get_header(buffer_t *p_buffer, Communications_protocol::Header *p_header) {
  result_t result          = RESULT_ERR;
  uint16_t buffer_loadsize = buffer_get_loadsize(p_buffer);
  uint16_t actual_read_pos = buffer_get_read_pos(p_buffer);
  uint16_t header_read_pos;

  Communications_protocol_rf::delimiter_rf delimiter;

  /* Check there is enough data to read the Delimiter */
  if (buffer_loadsize < PKT_DELIMITER_SIZE) {
    return RESULT_INCOMPLETE;
  }

  /* Get the delimiter and check it */
  result = buffer_get(p_buffer, (uint8_t *)&delimiter, PKT_DELIMITER_SIZE);
  EXIT_IF_ERR(result, "Packet header search - Delimiter buffer_get failed.");

  if (delimiter != Communications_protocol_rf::DELIMITER_RF) {
    return RESULT_ERR;
  }

  /* Check there is enough data to read the whole packet header */
  if (buffer_loadsize < PKT_FULL_HEADER_SIZE) {
    return RESULT_INCOMPLETE;
  }

  /* Get the header position */
  header_read_pos = buffer_calc_new_pos(p_buffer, actual_read_pos, PKT_DELIMITER_SIZE);

  /* Read the packet header */
  result = buffer_get_from_position(p_buffer, header_read_pos, (uint8_t *)p_header, PKT_HEADER_SIZE);
  EXIT_IF_ERR(result, "Packet header search - _header buffer_get failed.");

_EXIT:
  return result;
}

result_t Communications_protocol_rf::parseBuffer(parse_t *p_parse, buffer_t *p_buffer) {
  result_t result = RESULT_ERR;

  Communications_protocol::Header pkt_header;
  uint16_t buffer_loadsize = buffer_get_loadsize(p_buffer);
  uint16_t actual_read_pos = buffer_get_read_pos(p_buffer);

  uint8_t crc_pkt;
  uint8_t crc_calc;

  memset(p_parse, 0, sizeof(parse_t));
  p_parse->status_code = PARSE_STATUS_SUCCESS;

  /* Get the packet header */
  result = _get_header(p_buffer, &pkt_header);
  EXIT_IF_NOK(result);

  p_parse->pkt_cmd  = pkt_header.command;
  p_parse->pkt_size = PKT_FULL_HEADER_SIZE + pkt_header.size;

  actual_read_pos = buffer_calc_new_pos(p_buffer, actual_read_pos, PKT_FULL_HEADER_SIZE);

  /* Check there is enough space to receive the whole packet */
  if (p_parse->pkt_size >= buffer_get_max_loadsize(p_buffer) || p_parse->pkt_size > sizeof(Communications_protocol_rf::WrapperPacket)) {
    p_parse->status_code = PARSE_STATUS_ERR_PACKET_SIZE;
    return RESULT_OK;
  }

  /* Check the whole packet was received */
  if (buffer_loadsize < p_parse->pkt_size) {
    return RESULT_INCOMPLETE;
  }

  /*
     * The whole packet should be loaded now.
     *
     * From this point, the result is RESULT_OK and the further action should be decided based on the parse->status_code value
     *
     */

  /* Get the whole Packet */
  result = buffer_get(p_buffer, p_parse->wrapperPacket.buf, p_parse->pkt_size);
  EXIT_IF_ERR(result, "Packet buffer_get failed.");

  /* Get the packet CRC */
  crc_pkt = p_parse->wrapperPacket.packet.header.crc;

  /* Temporarily nullify the Packet CRC and calculate the CRC for comparison */

  p_parse->wrapperPacket.packet.header.crc = 0x00;
  crc_calc                                 = crc8(p_parse->wrapperPacket.packet.buf, p_parse->pkt_size - PKT_DELIMITER_SIZE);

  /* Return the Packet CRC to its position in the packet header */

  p_parse->wrapperPacket.packet.header.crc = crc_pkt;

  /* Check the CRC */
  if (crc_calc != crc_pkt) {
    p_parse->status_code = PARSE_STATUS_ERR_CRC;
    return RESULT_OK;
  }

  return RESULT_OK;

_EXIT:
  return result;
}


#endif  // defined(NRF52_ARCH) || defined(KEYSCANNER)
