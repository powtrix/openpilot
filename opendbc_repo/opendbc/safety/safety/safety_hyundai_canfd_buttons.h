#pragma once

#include "safety_declarations.h"
#include "safety_hyundai_common.h"

// Fail-closed transmit policy shared by both Hyundai CAN-FD cruise-button
// encodings. The per-platform TX allowlist remains responsible for bus and
// address routing; this helper constrains the payload and engagement state.
static bool hyundai_canfd_button_tx_allowed(const CANPacket_t *to_send, bool controls_are_allowed,
                                            bool stock_cruise_engaged, bool gas_is_pressed,
                                            bool brake_is_pressed, bool regen_is_braking) {
  const int addr = GET_ADDR(to_send);
  int button = -1;
  bool auxiliary_control_active = false;

  if ((addr == 0x1CF) && (GET_LEN(to_send) == 8U)) {
    button = GET_BYTE(to_send, 2) & 0x7U;
    auxiliary_control_active = GET_BIT(to_send, 19U) || GET_BIT(to_send, 21U) || GET_BIT(to_send, 23U) ||
                               GET_BIT(to_send, 25U) || GET_BIT(to_send, 27U);
  } else if ((addr == 0x1AA) && (GET_LEN(to_send) == 16U)) {
    button = (GET_BYTE(to_send, 4) >> 4) & 0x7U;
    auxiliary_control_active = GET_BIT(to_send, 34U) || GET_BIT(to_send, 39U) || GET_BIT(to_send, 41U);
  } else if ((addr == 0x1CF) || (addr == 0x1AA)) {
    return false;
  } else {
    return true;
  }

  // A synthesized cruise command must represent exactly one physical control.
  // Reject combined main/LFA/paddle inputs even when the cruise button itself
  // and the engagement state would otherwise be valid.
  if (auxiliary_control_active) {
    return false;
  }

  const bool is_resume = button == HYUNDAI_BTN_RESUME;
  // Unlike upstream's generic Hyundai button policy, this fork intentionally
  // transmits SET while controls are active to lower the stock-SCC set speed.
  const bool is_set = button == HYUNDAI_BTN_SET;
  const bool is_cancel = button == HYUNDAI_BTN_CANCEL;
  const bool driver_pedal_active = gas_is_pressed || brake_is_pressed || regen_is_braking;
  return ((is_resume || is_set) && controls_are_allowed && !driver_pedal_active) ||
         (is_cancel && stock_cruise_engaged);
}
