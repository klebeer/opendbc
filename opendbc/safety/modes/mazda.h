#pragma once

#include "opendbc/safety/declarations.h"

// CAN msgs we care about
#define MAZDA_LKAS          0x243U
#define MAZDA_LKAS_HUD      0x440U
#define MAZDA_CRZ_INFO      0x21bU
#define MAZDA_CRZ_CTRL      0x21cU
#define MAZDA_CRZ_BTNS      0x09dU
// Physical TJA button, DBC start bit 11 (byte 1, bit 3). Observed on a CTS-equipped gen1
// Mazda; trims without the button hold it low for the life of a drive.
#define MAZDA_TJA_BUTTON_BIT 11U
// sunnypilot safety param: the TJA button is the MADS lateral switch
#define MAZDA_PARAM_SP_TJA_BUTTON 1U
// Cruise MODE button, DBC start bit 13. On a trim with no MRCC it has nothing to toggle
// between, which frees it to be the lateral switch.
#define MAZDA_MODE_Y_BIT 13U
// sunnypilot safety param: no MRCC module, so CRZ_CTRL stops a few seconds after ignition.
#define MAZDA_PARAM_SP_NON_MRCC 2U
// The camera's CRZ_CTRL sets DBC bit 39 (byte 4, bit 7; unnamed) while another controller
// applies steering torque, and the cluster chimes for as long as it stays set.
#define MAZDA_CRZ_CTRL_STEER_CHIME_BYTE 4U
#define MAZDA_CRZ_CTRL_STEER_CHIME_MASK 0x80U
// Camera frames the relay may copy: openpilot runs a frame or two behind the camera.
#define MAZDA_CAM_CRZ_CTRL_HISTORY 2U
// CAM_TRAFFIC_SIGNS draws the speed limit sign on the HUD. SPEED_SIGN is DBC bits 4..0 of byte 0
// and 7..6 of byte 1, SPEED_SIGN_UNIT bits 5..4 of byte 1 (2 = km/h).
#define MAZDA_TRAFFIC_SIGNS 0x35fU
#define MAZDA_TSR_BYTE0_SIGN_MASK 0x1FU
#define MAZDA_TSR_BYTE1_SIGN_MASK 0xF0U
#define MAZDA_TSR_UNIT_KPH 2U
#define MAZDA_TSR_MIN_KPH 10U
#define MAZDA_TSR_MAX_KPH 120U
// NEW_SIGNAL_4 and SPEED_SIGN_CAM, byte 4 bits 1-0: the cluster and HUD draw the sign only with both set.
#define MAZDA_TSR_SHOWN_BYTE 4U
#define MAZDA_TSR_SHOWN_MASK 0x03U
// Auto Hold button, byte 3 bit 0 of 0x203, sent by a car-side module at 50 Hz. CTR is the high
// nibble of byte 4, and CHKSUM (byte 5) falls by one for each unit added to bytes 0-3 or to CTR.
#define MAZDA_AUTO_HOLD_BTN 0x203U
#define MAZDA_AUTO_HOLD_BTN_BYTE 3U
#define MAZDA_AUTO_HOLD_BTN_MASK 0x01U
// Auto Hold armed, byte 3 bit 6 of 0x079.
#define MAZDA_EPB 0x079U
#define MAZDA_EPB_AUTO_HOLD_ARMED_BYTE 3U
#define MAZDA_EPB_AUTO_HOLD_ARMED_MASK 0x40U
#define MAZDA_RADAR_STATIC  0x499U
#define MAZDA_RADAR_TRACK_1 0x361U
#define MAZDA_RADAR_TRACK_2 0x362U
#define MAZDA_RADAR_TRACK_3 0x363U
#define MAZDA_RADAR_TRACK_4 0x364U
#define MAZDA_RADAR_TRACK_5 0x365U
#define MAZDA_RADAR_TRACK_6 0x366U
#define MAZDA_RADAR_UDS     0x764U
#define MAZDA_STEER_TORQUE  0x240U
#define MAZDA_ENGINE_DATA   0x202U
#define MAZDA_PEDALS        0x165U

// CAN bus numbers
#define MAZDA_MAIN 0
#define MAZDA_CAM  2

#define MAZDA_PARAM_LONGITUDINAL 1U
// Select the steer-to-zero EPS envelope from the firmware-derived interface flag.
#define MAZDA_PARAM_STEER_TO_ZERO_EPS 2U
// The same EPS hardware on firmware that keeps the 45 kph floor: the same envelope.
#define MAZDA_PARAM_LEGACY_FW_EPS 4U

// Keep SET/RES intent fresh until PEDALS reports engagement.
#define MAZDA_ENGAGE_BTN_WINDOW 10U
// A wheel cancel turns MRCC main off for real, and PEDALS trails the press: keep the context
// on the 50 Hz PEDALS clock for carstate's CANCEL_CONTEXT_T so the main-off edge lands under
// braking, where a brake-only bit dropout would otherwise be held.
#define MAZDA_CANCEL_CONTEXT_FRAMES 25U

static bool mazda_longitudinal = false;
// Declared by the driver: the TJA button owns lateral and MRCC no longer drives the main edge.
static bool mazda_tja_button = false;
// Declared by the platform, not the driver: the car has no MRCC module at all.
static bool mazda_non_mrcc = false;
static bool mazda_steer_to_zero_eps = false;
static bool mazda_legacy_fw_eps = false;
static uint32_t mazda_engage_btn_frames = 0U;
static uint32_t mazda_cancel_context_frames = 0U;
static uint8_t mazda_cam_crz_ctrl[MAZDA_CAM_CRZ_CTRL_HISTORY][8];
static uint32_t mazda_cam_crz_ctrl_seen = 0U;
static uint8_t mazda_cam_tsr[MAZDA_CAM_CRZ_CTRL_HISTORY][8];
static uint32_t mazda_cam_tsr_seen = 0U;
static uint8_t mazda_auto_hold_btn[MAZDA_CAM_CRZ_CTRL_HISTORY][8];
static uint32_t mazda_auto_hold_btn_seen = 0U;
// Unknown counts as armed: a press on an armed Auto Hold switches it off.
static bool mazda_auto_hold_armed = true;

// Pin replaced-radar traffic to captured stock patterns where possible.

// Each radar generation sends its own static capture; the controller picks the dialect (mazdacan.py)
static bool mazda_radar_static_msg_valid(const CANPacket_t *msg) {
  bool capture_2022 = (msg->data[0] == 0x00U) && (msg->data[1] == 0x08U) &&
                      (msg->data[2] == 0xc0U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  bool capture_g46l = (msg->data[0] == 0x00U) && (msg->data[1] == 0x98U) &&
                      (msg->data[2] == 0x40U) && (msg->data[3] == 0x00U) &&
                      (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
                      (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
  return capture_2022 || capture_g46l;
}

static bool mazda_empty_radar_track_msg_valid(const CANPacket_t *msg) {
  bool valid = false;

  if ((msg->addr == MAZDA_RADAR_TRACK_1) || (msg->addr == MAZDA_RADAR_TRACK_2) ||
      (msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0xfeU) &&
            (msg->data[4] == 0x1fU);

    if (msg->addr == MAZDA_RADAR_TRACK_2) {
      valid = valid && (msg->data[5] == 0xc7U) && (msg->data[6] == 0x8cU) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    } else if ((msg->addr == MAZDA_RADAR_TRACK_3) || (msg->addr == MAZDA_RADAR_TRACK_4)) {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x00U);
    } else {
      valid = valid && (msg->data[5] == 0xc0U) && (msg->data[6] == 0x00U) &&
              ((msg->data[7] & 0xf0U) == 0x80U);
    }
  } else if ((msg->addr == MAZDA_RADAR_TRACK_5) || (msg->addr == MAZDA_RADAR_TRACK_6)) {
    valid = (msg->data[0] == 0xffU) && (msg->data[1] == 0xf7U) &&
            (msg->data[2] == 0xfeU) && (msg->data[3] == 0x7fU) &&
            (msg->data[4] == 0xfbU) && (msg->data[5] == 0xffU) &&
            (msg->data[6] == 0x3fU) && ((msg->data[7] & 0xf0U) == 0xc0U);
  } else {
  }

  return valid;
}

static bool mazda_synthetic_lead_radar_track_msg_valid(const CANPacket_t *msg) {
  // Permit only the distance and relative-velocity fields in the occupied-track template.
  return (msg->addr == MAZDA_RADAR_TRACK_4) &&
         ((msg->data[1] & 0x0fU) == 0x0eU) && (msg->data[2] == 0x00U) &&
         ((msg->data[4] & 0x1fU) == 0x1cU) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && ((msg->data[7] & 0xf0U) == 0x00U);
}

static bool mazda_radar_track_msg_valid(const CANPacket_t *msg) {
  // Occupied tracks represent perception and remain valid while controls are disengaged.
  return mazda_empty_radar_track_msg_valid(msg) ||
         mazda_synthetic_lead_radar_track_msg_valid(msg);
}

// track msgs coming from OP so that we know what CAM msgs to drop and what to forward
static void mazda_rx_hook(const CANPacket_t *msg) {
  if ((int)msg->bus == MAZDA_MAIN) {
    if (msg->addr == MAZDA_ENGINE_DATA) {
      // sample speed: scale by 0.01 to get kph
      int speed = (msg->data[2] << 8) | msg->data[3];
      vehicle_moving = speed > 10; // moving when speed > 0.1 kph
    }

    if (msg->addr == MAZDA_STEER_TORQUE) {
      int torque_driver_new = msg->data[0] - 127U;
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // enter controls on rising edge of ACC, exit controls on ACC off
    if ((msg->addr == MAZDA_CRZ_CTRL) && !mazda_longitudinal) {
      bool cruise_engaged = msg->data[0] & 0x8U;
      pcm_cruise_check(cruise_engaged);
      // With the TJA button owning lateral, MRCC no longer drives the MADS main edge: its
      // falling edge would exit the panda's lateral while the software's MADS stays on.
      if (!mazda_tja_button) {
        acc_main_on = GET_BIT(msg, 17U);
      }
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_tja_button) {
      // The physical TJA button is the MADS lateral switch, so lateral no longer follows MRCC.
      mads_button_press = GET_BIT(msg, MAZDA_TJA_BUTTON_BIT) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_non_mrcc) {
      // No TJA button on this trim, and no MRCC for the MODE button to switch: it owns lateral.
      mads_button_press = GET_BIT(msg, MAZDA_MODE_Y_BIT) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }

    if ((msg->addr == MAZDA_CRZ_BTNS) && mazda_longitudinal) {
      // A physical cancel press always exits controls, and explains the main-off that follows.
      bool cancel = GET_BIT(msg, 0U);
      if (cancel) {
        controls_allowed = false;
        mazda_cancel_context_frames = MAZDA_CANCEL_CONTEXT_FRAMES;
      }
      // Record SET/RES intent for the engagement qualifier below.
      if (GET_BIT(msg, 2U) || GET_BIT(msg, 4U) || GET_BIT(msg, 5U)) {
        mazda_engage_btn_frames = MAZDA_ENGAGE_BTN_WINDOW;
      } else if (mazda_engage_btn_frames > 0U) {
        mazda_engage_btn_frames -= 1U;
      } else {
      }
    }

    if (msg->addr == MAZDA_ENGINE_DATA) {
      gas_pressed = (msg->data[4] || (msg->data[5] & 0xF0U));
    }

    if (msg->addr == MAZDA_PEDALS) {
      bool brake = (msg->data[0] & 0x10U);
      if (mazda_longitudinal) {
        // Derive cruise state from PEDALS after radar teardown. Ignore transient brake-only
        // samples where both cruise bits are low.
        bool cruise_engaged = GET_BIT(msg, 3U);
        bool acc_armed = GET_BIT(msg, 2U) || cruise_engaged;
        bool brake_free = !brake && !brake_pressed_prev;

        // Main mirrors carstate's cruise_available: it follows arming, and a both-low sample is
        // held under braking unless a wheel cancel explains it. Without the cancel path, main
        // toggled at a stop with the brake held never falls, the next press has no rising edge,
        // and MADS runs into 200 rejected frames (route 000001c9--0b2a64a214 seg 0).
        if (mazda_tja_button) {
          // the button is the lateral switch; MRCC is cruise only
        } else if (acc_armed) {
          // Main follows PEDALS arming from the first frame; the radar takeover gates cruise
          // (controls_allowed below), never main.
          acc_main_on = true;
        } else if (brake_free || (mazda_cancel_context_frames > 0U)) {
          acc_main_on = false;
        } else {
        }
        if (mazda_cancel_context_frames > 0U) {
          mazda_cancel_context_frames -= 1U;
        }

        if (acc_armed || cruise_engaged_prev || brake_free) {
          // Require recent SET/RES intent on the engaged edge; ACC_ACTIVE alone may acknowledge
          // synthetic traffic rather than a driver request.
          if (cruise_engaged && !cruise_engaged_prev && (mazda_engage_btn_frames > 0U)) {
            controls_allowed = true;
          }
          if (!cruise_engaged) {
            controls_allowed = false;
          }
          cruise_engaged_prev = cruise_engaged;
        }
      }
      brake_pressed = brake;
    }
  }

  if (((int)msg->bus == MAZDA_CAM) && (msg->addr == MAZDA_CRZ_CTRL) && mazda_non_mrcc) {
    for (uint32_t i = 0U; i < 8U; i++) {
      mazda_cam_crz_ctrl[1][i] = mazda_cam_crz_ctrl[0][i];
      mazda_cam_crz_ctrl[0][i] = msg->data[i];
    }
    if (mazda_cam_crz_ctrl_seen < MAZDA_CAM_CRZ_CTRL_HISTORY) {
      mazda_cam_crz_ctrl_seen += 1U;
    }
  }

  if (((int)msg->bus == MAZDA_CAM) && (msg->addr == MAZDA_TRAFFIC_SIGNS) && mazda_non_mrcc) {
    for (uint32_t i = 0U; i < 8U; i++) {
      mazda_cam_tsr[1][i] = mazda_cam_tsr[0][i];
      mazda_cam_tsr[0][i] = msg->data[i];
    }
    if (mazda_cam_tsr_seen < MAZDA_CAM_CRZ_CTRL_HISTORY) {
      mazda_cam_tsr_seen += 1U;
    }
  }

  if (((int)msg->bus == MAZDA_MAIN) && (msg->addr == MAZDA_AUTO_HOLD_BTN) && mazda_non_mrcc) {
    for (uint32_t i = 0U; i < 8U; i++) {
      mazda_auto_hold_btn[1][i] = mazda_auto_hold_btn[0][i];
      mazda_auto_hold_btn[0][i] = msg->data[i];
    }
    if (mazda_auto_hold_btn_seen < MAZDA_CAM_CRZ_CTRL_HISTORY) {
      mazda_auto_hold_btn_seen += 1U;
    }
  }

  if (((int)msg->bus == MAZDA_MAIN) && (msg->addr == MAZDA_EPB) && mazda_non_mrcc) {
    mazda_auto_hold_armed = (msg->data[MAZDA_EPB_AUTO_HOLD_ARMED_BYTE] & MAZDA_EPB_AUTO_HOLD_ARMED_MASK) != 0U;
  }
}

// openpilot's CAM_TRAFFIC_SIGNS on a car without MRCC: a recent camera frame, byte for byte,
// except the speed sign fields, which may only change to a km/h limit within the posted range,
// and then only with both sign-shown bits set.
static bool mazda_tsr_relay_valid(const CANPacket_t *msg) {
  bool valid = false;
  uint32_t speed = (((uint32_t)msg->data[0] & MAZDA_TSR_BYTE0_SIGN_MASK) << 2U) | ((uint32_t)msg->data[1] >> 6U);
  uint32_t unit = ((uint32_t)msg->data[1] >> 4U) & 0x3U;
  bool plausible = (unit == MAZDA_TSR_UNIT_KPH) && (speed >= MAZDA_TSR_MIN_KPH) && (speed <= MAZDA_TSR_MAX_KPH);

  for (uint32_t f = 0U; f < mazda_cam_tsr_seen; f++) {
    bool rest_same = ((msg->data[0] & (uint8_t)(~MAZDA_TSR_BYTE0_SIGN_MASK)) == (mazda_cam_tsr[f][0] & (uint8_t)(~MAZDA_TSR_BYTE0_SIGN_MASK))) &&
                     ((msg->data[1] & (uint8_t)(~MAZDA_TSR_BYTE1_SIGN_MASK)) == (mazda_cam_tsr[f][1] & (uint8_t)(~MAZDA_TSR_BYTE1_SIGN_MASK)));
    for (uint32_t i = 2U; i < 8U; i++) {
      uint8_t mask = (i == MAZDA_TSR_SHOWN_BYTE) ? (uint8_t)(~MAZDA_TSR_SHOWN_MASK) : 0xffU;
      if ((msg->data[i] & mask) != (mazda_cam_tsr[f][i] & mask)) {
        rest_same = false;
      }
    }
    bool sign_same = ((msg->data[0] & MAZDA_TSR_BYTE0_SIGN_MASK) == (mazda_cam_tsr[f][0] & MAZDA_TSR_BYTE0_SIGN_MASK)) &&
                     ((msg->data[1] & MAZDA_TSR_BYTE1_SIGN_MASK) == (mazda_cam_tsr[f][1] & MAZDA_TSR_BYTE1_SIGN_MASK)) &&
                     ((msg->data[MAZDA_TSR_SHOWN_BYTE] & MAZDA_TSR_SHOWN_MASK) == (mazda_cam_tsr[f][MAZDA_TSR_SHOWN_BYTE] & MAZDA_TSR_SHOWN_MASK));
    bool map_limit = plausible && ((msg->data[MAZDA_TSR_SHOWN_BYTE] & MAZDA_TSR_SHOWN_MASK) == MAZDA_TSR_SHOWN_MASK);
    valid = valid || (rest_same && (sign_same || map_limit));
  }

  return valid;
}

// openpilot's CRZ_CTRL on a car without MRCC: a recent camera frame, byte for byte, with only
// the steering chime bit cleared. Nothing else in the frame, cruise state included, can change.
static bool mazda_crz_ctrl_relay_valid(const CANPacket_t *msg) {
  bool valid = false;

  if ((msg->data[MAZDA_CRZ_CTRL_STEER_CHIME_BYTE] & MAZDA_CRZ_CTRL_STEER_CHIME_MASK) == 0U) {
    for (uint32_t f = 0U; f < mazda_cam_crz_ctrl_seen; f++) {
      bool same = true;
      for (uint32_t i = 0U; i < 8U; i++) {
        uint8_t mask = (i == MAZDA_CRZ_CTRL_STEER_CHIME_BYTE) ? (uint8_t)(~MAZDA_CRZ_CTRL_STEER_CHIME_MASK) : 0xffU;
        if ((msg->data[i] & mask) != (mazda_cam_crz_ctrl[f][i] & mask)) {
          same = false;
        }
      }
      valid = valid || same;
    }
  }

  return valid;
}

// openpilot's Auto Hold press: one of the car's last two frames, released, with only the button
// set, the next counter, and the checksum that follows from both. The counter wraps 15 -> 0.
static bool mazda_auto_hold_press_valid(const CANPacket_t *msg) {
  bool valid = false;

  for (uint32_t f = 0U; f < mazda_auto_hold_btn_seen; f++) {
    const uint8_t *car = mazda_auto_hold_btn[f];
    uint8_t ctr = (uint8_t)((((uint32_t)car[4] >> 4U) + 1U) & 0xFU);
    uint8_t chk = (ctr == 0U) ? (uint8_t)((uint32_t)car[5] + 14U) : (uint8_t)((uint32_t)car[5] - 2U);
    bool released = (car[MAZDA_AUTO_HOLD_BTN_BYTE] & MAZDA_AUTO_HOLD_BTN_MASK) == 0U;
    bool same = released &&
                (msg->data[0] == car[0]) && (msg->data[1] == car[1]) && (msg->data[2] == car[2]) &&
                (msg->data[3] == (uint8_t)(car[3] | MAZDA_AUTO_HOLD_BTN_MASK)) &&
                (msg->data[4] == (uint8_t)((uint8_t)(ctr << 4U) | (car[4] & 0x0FU))) &&
                (msg->data[5] == chk) && (msg->data[6] == car[6]) && (msg->data[7] == car[7]);
    valid = valid || same;
  }

  return valid;
}

static bool mazda_is_lka_addr(int addr) {
  return (((unsigned int)addr == MAZDA_LKAS) || ((unsigned int)addr == MAZDA_LKAS_HUD));
}

// The camera owns the LKAS addresses whenever openpilot is not steering. Lateral is its own
// axis under MADS (controls_allowed_lateral); with MADS off it follows cruise. Cruise alone must
// not claim them: under stock long that silenced the camera's own TJA/CTS with MADS off while the
// dash showed nothing (route 00000018--5655da2c1c seg 15).
static bool mazda_openpilot_controlling(void) {
  return controls_allowed_lateral || (controls_allowed && !m_mads_state.system_enabled);
}

// The one CRZ_BTNS frame openpilot may put on the camera bus: the TJA button pressed over the
// wheel's idle pattern (00 09 ff Cx 00 00 00 00, Cx = MODE_X_INV, MODE_Y_INV and the counter),
// no other button. It presses the camera's own TJA/CTS off whenever the camera is armed, so the
// two lane-centering systems never run at once and the camera never takes the wheel behind a
// MADS-off press. Accepted in every state: the frame only reaches the camera and can only
// toggle its lane centering, which the wheel button does anyway; the camera's torque is still
// vetoed whenever openpilot steers (docs/zoompilot/mazda-lateral.md, "The camera's own TJA/CTS
// state").
static bool mazda_cam_tja_press_msg_valid(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) && (msg->data[1] == 0x09U) && (msg->data[2] == 0xffU) &&
         ((msg->data[3] & 0xc3U) == 0xc0U) && (msg->data[4] == 0x00U) && (msg->data[5] == 0x00U) &&
         (msg->data[6] == 0x00U) && (msg->data[7] == 0x00U);
}

static bool mazda_tx_hook(const CANPacket_t *msg) {
  // Stock pre-2022 EPS envelope.
  const TorqueSteeringLimits MAZDA_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // The measured EPS envelope, selected by either firmware bit: the EPS's 12-count hardware
  // slew, with max_rate_down equal to the controller retreat rate so driver-limit winddown
  // frames remain valid.
  const TorqueSteeringLimits MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS = {
    .max_torque = 1200,
    .max_rate_up = 12,
    .max_rate_down = 12,
    .max_rt_delta = 384,
    .driver_torque_multiplier = 15,
    .driver_torque_allowance = 15,
    .type = TorqueDriverLimited,
  };

  // CRZ_INFO.ACCEL_CMD uses 0.001 m/s2 raw units after removing the offset.
  const LongitudinalLimits MAZDA_LONG_LIMITS = {
    .max_accel = 2000,
    .min_accel = -3500,
    .inactive_accel = 0,
  };

  bool tx = true;
  bool main_bus = msg->bus == (unsigned char)MAZDA_MAIN;
  bool long_replacement_bus = main_bus || (msg->bus == (unsigned char)MAZDA_CAM);

  if (main_bus && (msg->addr == MAZDA_LKAS)) {
    int desired_torque = (((msg->data[0] & 0x0FU) << 8) | msg->data[1]) - 2048U;

    const TorqueSteeringLimits *limits = &MAZDA_STEERING_LIMITS;
    if (mazda_steer_to_zero_eps || mazda_legacy_fw_eps) {
      limits = &MAZDA_STEER_TO_ZERO_EPS_STEERING_LIMITS;
    } else {
      // upstream's pre-2022 envelope, no longer selected by the interface
    }
    if (steer_torque_cmd_checks(desired_torque, -1, *limits)) {
      tx = false;
    }
  }

  // Run after steering checks, which reset rate-limit state while disengaged.
  if (main_bus && mazda_is_lka_addr(msg->addr) && !mazda_openpilot_controlling()) {
    tx = false;
  }

  if (mazda_non_mrcc && main_bus && (msg->addr == MAZDA_TRAFFIC_SIGNS)) {
    // Stands in for the camera's frame only while that frame is blocked below.
    if (!mazda_openpilot_controlling() || !mazda_tsr_relay_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_non_mrcc && main_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    // Stands in for the camera's frame only while that frame is blocked below.
    if (!mazda_openpilot_controlling() || !mazda_crz_ctrl_relay_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_non_mrcc && main_bus && (msg->addr == MAZDA_AUTO_HOLD_BTN)) {
    // Stopped with the brake held and Auto Hold off only: pressed while armed it switches off.
    if (vehicle_moving || !brake_pressed || mazda_auto_hold_armed || !mazda_auto_hold_press_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_INFO)) {
    // Allow byte-exact stock standby patterns with the raw 8190 command sentinel.
    bool stock_standby = (msg->data[0] == 0x01U) && (msg->data[1] == 0xffU) &&
                         (msg->data[2] == 0xe3U) && (msg->data[3] == 0xffU) &&
                         ((msg->data[4] & 0xfbU) == 0xc0U) &&
                         ((msg->data[5] & 0x7fU) == 0x00U) &&
                         ((msg->data[6] & 0xf0U) == 0x00U) &&
                         (msg->data[7] == ((0xffU - ((msg->data[0] + msg->data[1] + msg->data[2] + msg->data[3] +
                                                     msg->data[4] + msg->data[5] + msg->data[6]) & 0xffU)) & 0xffU));

    // Assemble the 13-bit ACCEL_CMD unsigned for MISRA 10.1, then remove its 4096 offset.
    uint32_t accel_raw = (((uint32_t)msg->data[2] & 0x3U) << 11) | ((uint32_t)msg->data[3] << 3) | ((uint32_t)msg->data[4] >> 5);
    int desired_accel = (int)accel_raw - 4096;
    if (!stock_standby && longitudinal_accel_checks(desired_accel, MAZDA_LONG_LIMITS)) {
      tx = false;
    }

    // ACC_ACTIVE requires prior controls permission established from the physical SET edge.
    bool acc_active = GET_BIT(msg, 33U);
    if (!controls_allowed && acc_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_CRZ_CTRL)) {
    bool cruise_active = GET_BIT(msg, 3U);
    if (!controls_allowed && cruise_active) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr == MAZDA_RADAR_STATIC)) {
    if (!mazda_radar_static_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && long_replacement_bus && (msg->addr >= MAZDA_RADAR_TRACK_1) && (msg->addr <= MAZDA_RADAR_TRACK_6)) {
    if (!mazda_radar_track_msg_valid(msg)) {
      tx = false;
    }
  }

  if (mazda_longitudinal && main_bus && (msg->addr == MAZDA_RADAR_UDS)) {
    // Allow tester-present and default/programming session control only.
    bool tester_present = (msg->data[0] == 0x02U) && (msg->data[1] == 0x3eU) && (msg->data[2] == 0x80U);
    bool session_control = (msg->data[0] == 0x02U) && (msg->data[1] == 0x10U) &&
                           ((msg->data[2] == 0x01U) || (msg->data[2] == 0x02U));
    if (!tester_present && !session_control) {
      tx = false;
    }
  }

  if (main_bus && (msg->addr == MAZDA_CRZ_BTNS)) {
    // Permit resume only while controlling and cancel only while not controlling.
    bool cancel_cmd = (msg->data[0] == 0x1U);
    if (!controls_allowed && !cancel_cmd) {
      tx = false;
    }
    // The TJA button is never pressed on the car's side: it would toggle MADS through the
    // rx hook and arm MRCC in the body.
    if (GET_BIT(msg, MAZDA_TJA_BUTTON_BIT)) {
      tx = false;
    }
  }

  if ((msg->bus == (unsigned char)MAZDA_CAM) && (msg->addr == MAZDA_CRZ_BTNS)) {
    // The camera-side press exists only to switch the camera's TJA/CTS off: byte-exact, any state.
    if (!mazda_cam_tja_press_msg_valid(msg)) {
      tx = false;
    }
  }

  return tx;
}

static bool mazda_fwd_hook(int bus_num, int addr) {
  bool block_msg = false;

  if (bus_num == MAZDA_CAM) {
    if (mazda_is_lka_addr(addr)) {
      block_msg = mazda_openpilot_controlling();
    }
    if (mazda_non_mrcc && (((unsigned int)addr == MAZDA_CRZ_CTRL) || ((unsigned int)addr == MAZDA_TRAFFIC_SIGNS))) {
      block_msg = mazda_openpilot_controlling();
    }
  }

  return block_msg;
}

static safety_config mazda_init(uint16_t param) {
  mazda_engage_btn_frames = 0U;
  mazda_cancel_context_frames = 0U;
  mazda_cam_crz_ctrl_seen = 0U;
  mazda_cam_tsr_seen = 0U;
  mazda_auto_hold_btn_seen = 0U;
  mazda_auto_hold_armed = true;

  static const CanMsg MAZDA_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    // The camera press: no relay check, so the wheel's own 0x09d keeps forwarding to the camera.
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
  };

  // No relay check on CRZ_CTRL: the body publishes it on the main bus for the first seconds.
  static const CanMsg MAZDA_NON_MRCC_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_TRAFFIC_SIGNS, 0, 8, .check_relay = false},
    // No relay check: the car's own module keeps publishing the button on the main bus.
    {MAZDA_AUTO_HOLD_BTN, 0, 8, .check_relay = false},
  };

// Replaced-radar addresses omit relay checks because the radar remains live during boot and
// hand-back. carstate enforces single ownership instead.
  static const CanMsg MAZDA_LONG_TX_MSGS[] = {
    {MAZDA_LKAS, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, 0, 8, .check_relay = false},
    {MAZDA_LKAS_HUD, 0, 8, .check_relay = true, .disable_static_blocking = true},
    {MAZDA_CRZ_BTNS, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, 0, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, 0, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, 0, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, 0, 8, .check_relay = false},
    {MAZDA_RADAR_UDS, 0, 8, .check_relay = false},
    {MAZDA_CRZ_INFO, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_CRZ_CTRL, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_STATIC, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_1, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_2, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_3, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_4, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_5, MAZDA_CAM, 8, .check_relay = false},
    {MAZDA_RADAR_TRACK_6, MAZDA_CAM, 8, .check_relay = false},
  };

  static RxCheck mazda_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_CTRL,     0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // CRZ_CTRL intentionally disappears after radar teardown.
  static RxCheck mazda_long_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  // The camera's CRZ_CTRL and the car's Auto Hold frames are checked so the rx hook sees the
  // frames the relays copy and the armed state the press depends on.
  static RxCheck mazda_non_mrcc_rx_checks[] = {
    {.msg = {{MAZDA_CRZ_BTNS,     0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_STEER_TORQUE, 0, 8, 83U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_ENGINE_DATA,  0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_PEDALS,       0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_CRZ_CTRL, MAZDA_CAM, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_TRAFFIC_SIGNS, MAZDA_CAM, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_AUTO_HOLD_BTN, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{MAZDA_EPB, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  mazda_longitudinal = GET_FLAG(param, MAZDA_PARAM_LONGITUDINAL);
  mazda_steer_to_zero_eps = GET_FLAG(param, MAZDA_PARAM_STEER_TO_ZERO_EPS);
  mazda_legacy_fw_eps = GET_FLAG(param, MAZDA_PARAM_LEGACY_FW_EPS);
  mazda_tja_button = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_TJA_BUTTON);
  mazda_non_mrcc = GET_FLAG(current_safety_param_sp, MAZDA_PARAM_SP_NON_MRCC);
  acc_main_on = false;

  // A car with no MRCC never publishes CRZ_CTRL on the main bus past the first seconds, so it
  // checks the camera's instead, without opening the longitudinal TX list.
  return mazda_longitudinal ? BUILD_SAFETY_CFG(mazda_long_rx_checks, MAZDA_LONG_TX_MSGS) :
         mazda_non_mrcc     ? BUILD_SAFETY_CFG(mazda_non_mrcc_rx_checks, MAZDA_NON_MRCC_TX_MSGS) :
                              BUILD_SAFETY_CFG(mazda_rx_checks, MAZDA_TX_MSGS);
}

const safety_hooks mazda_hooks = {
  .init = mazda_init,
  .rx = mazda_rx_hook,
  .tx = mazda_tx_hook,
  .fwd = mazda_fwd_hook,
};
