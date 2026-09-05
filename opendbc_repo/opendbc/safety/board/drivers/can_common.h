#include "can_common_declarations.h"

// The device CAN driver owns this flag in panda/board. The host safety-test
// shim needs the same symbol so buffered-forwarding safety modes can link.
bool safety_tx_buffered_for_fwd = false;

uint8_t calculate_checksum(const uint8_t *dat, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= dat[i];
  }
  return checksum;
}

void can_set_checksum(CANPacket_t *packet) {
  packet->checksum = 0U;
  packet->checksum = calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet));
}

bool can_check_checksum(CANPacket_t *packet) {
  return (calculate_checksum((uint8_t *) packet, CANPACKET_HEAD_SIZE + GET_LEN(packet)) == 0U);
}
