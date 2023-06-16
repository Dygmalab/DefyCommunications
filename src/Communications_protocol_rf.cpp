#if defined(NRF52_ARCH) || defined(KEYSCANNER)
#include "Communications_protocol_rf.h"


result_t Communications_protocol_rf::parseBuffer(Communications_protocol_rf::parse_t *p_parse, buffer_t *p_buffer) {
  return RESULT_OK;
}

#endif
