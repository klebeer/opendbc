from opendbc.car.can_definitions import CanData
from opendbc.car.mazda.values import Buttons, MazdaFlags

# Captured empty radar tracks required by the body ECU for stop-and-go. Only the counter
# nibble changes.
RADAR_STATIC_MSG = (0x499, bytes.fromhex("0008c00000000000"))
RADAR_TRACK_MSGS = {
  0x361: bytes.fromhex("fff7fefe1fc00080"),
  0x362: bytes.fromhex("fff7fefe1fc78c80"),
  0x363: bytes.fromhex("fff7fefe1fc00000"),
  0x364: bytes.fromhex("fff7fefe1fc00000"),
  0x365: bytes.fromhex("fff7fe7ffbff3fc0"),
  0x366: bytes.fromhex("fff7fe7ffbff3fc0"),
}

# The G46L radar (2016.5 bodies) sends only this static frame and no track messages at
# all; fully static — no counter, no checksum.
G46L_RADAR_STATIC_MSG = (0x499, bytes.fromhex("0098400000000000"))

# Auto Hold button, byte 3 bit 0 of 0x203, sent by a car-side module at 50 Hz. CTR is the high
# nibble of byte 4, and CHKSUM (byte 5) falls by one for each unit added to bytes 0-3 or to CTR.
AUTO_HOLD_BTN_ADDR = 0x203
AUTO_HOLD_BTN_BYTE = 3
AUTO_HOLD_BTN_MASK = 0x01
AUTO_HOLD_PARAM = "MazdaAutoHold"
# Auto Hold armed, byte 3 bit 6 of 0x079.
EPB_ADDR = 0x079
EPB_AUTO_HOLD_ARMED_BYTE = 3
EPB_AUTO_HOLD_ARMED_MASK = 0x40

CRZ_CTRL_ADDR = 0x21c
# On a car without MRCC the camera publishes CRZ_CTRL, and sets DBC bit 39 (unnamed) while
# another controller applies steering torque; the cluster chimes for as long as it is set.
CRZ_CTRL_STEER_CHIME_BYTE = 4
CRZ_CTRL_STEER_CHIME_MASK = 0x80

# CAM_TRAFFIC_SIGNS draws the speed limit sign on the HUD. SPEED_SIGN is bits 4..0 of byte 0 and
# 7..6 of byte 1, SPEED_SIGN_UNIT bits 5..4 of byte 1 (0 = no sign, 2 = km/h).
TRAFFIC_SIGNS_ADDR = 0x35f
HUD_SPEED_LIMIT_PARAM = "HudSpeedLimitKph"
TSR_MIN_KPH, TSR_MAX_KPH = 10, 120
TSR_UNIT_KPH = 2
# NEW_SIGNAL_4 and SPEED_SIGN_CAM, byte 4 bits 1-0: the cluster and the HUD draw the sign only with
# both set (parked probe on the car); with the speed fields alone they show nothing.
TSR_SHOWN_BYTE = 4
TSR_SHOWN_MASK = 0x03


def crz_info_checksum(dat: bytes) -> int:
  # Invert the sum of the first seven bytes, excluding STOPPING and RESUME_UNLATCHING.
  return (0xFF - ((sum(dat[:7]) - (dat[5] & 0x04) - (dat[6] & 0x40)) & 0xFF)) & 0xFF


def create_acc_command(packer, bus, counter, accel, *, long_active, acc_available,
                       brake_pressed=False, stopping=False, resume_unlatching=False):
  # CRZ_INFO replaces the disabled radar's acceleration command and armed-idle state.
  values = {
    "ERROR_STATUS": 1,
    "STATIC_1": 0x7ff,
    "CTR": counter % 16,
    "ACCEL_CMD": accel if long_active else 4.094,  # stock non-controlling sentinel
    "NEW_SIGNAL_7": int(long_active or acc_available),
  }
  if long_active:
    values.update({
      "ACC_ACTIVE": 1,
      "ACC_SET_ALLOWED": 1,
      "STOPPING": int(stopping),
      "STOPPING_2": int(stopping),
      "RESUME_UNLATCHING": int(resume_unlatching),
    })
  elif acc_available:
    values["ACC_SET_ALLOWED"] = int(not brake_pressed)

  dat = packer.make_can_msg("CRZ_INFO", bus, values)[1]
  values["CHKSUM"] = crz_info_checksum(dat)
  return packer.make_can_msg("CRZ_INFO", bus, values)


def create_crz_ctrl(packer, bus, long_active, acc_available, gap_setting, acc_active_2, *, lead_distance=0,
                    hbc_request=False):
  # CRZ_CTRL replaces radar cruise state and mirrors the driver's gap selection. The cluster draws
  # the lead from RADAR_HAS_LEAD; RADAR_LEAD_RELATIVE_DISTANCE is stock's 1 to 5 closeness bucket.
  # NEW_SIGNAL_3 (bit 13) relays the camera's high-beam request (0x440 BIT2) as the stock radar
  # does; the body raises the lamps and the cluster's green HBC light on it.
  values = {
    "MSG_1_INV": 1,
    "MSG_1_INV_COPY": 1,
    "NEW_SIGNAL_8": 1,
    "CRZ_ACTIVE": int(long_active),
    "CRZ_AVAILABLE": int(long_active or acc_available),
    "DISTANCE_SETTING": gap_setting,
    "RADAR_HAS_LEAD": int(lead_distance > 0),
    "RADAR_LEAD_RELATIVE_DISTANCE": lead_distance,
    "ACC_ACTIVE_2": int(acc_active_2),
    "NEW_SIGNAL_3": int(hbc_request),
  }
  return packer.make_can_msg("CRZ_CTRL", bus, values)


def create_radar_frames(bus, counter, g46l=False):
  if g46l:
    return [CanData(G46L_RADAR_STATIC_MSG[0], G46L_RADAR_STATIC_MSG[1], bus)]
  frames = [CanData(RADAR_STATIC_MSG[0], RADAR_STATIC_MSG[1], bus)]
  for addr, dat in RADAR_TRACK_MSGS.items():
    frames.append(CanData(addr, dat[:7] + bytes([(dat[7] & 0xf0) | (counter % 16)]), bus))
  return frames


def create_steering_control(packer, CP, frame, apply_torque, lkas):

  tmp = apply_torque + 2048

  lo = tmp & 0xFF
  hi = tmp >> 8

  # copy values from camera
  b1 = int(lkas["BIT_1"])
  er1 = int(lkas["ERR_BIT_1"])
  lnv = 0
  ldw = 0
  er2 = int(lkas["ERR_BIT_2"])

  # Some older models do have these, newer models don't.
  # Either way, they all work just fine if set to zero.
  steering_angle = 0
  b2 = 0

  tmp = steering_angle + 2048
  ahi = tmp >> 10
  amd = (tmp & 0x3FF) >> 2
  amd = (amd >> 4) | ((amd & 0xF) << 4)
  alo = (tmp & 0x3) << 2

  ctr = frame % 16
  # bytes:     [    1  ] [ 2 ] [             3               ]  [           4         ]
  csum = 249 - ctr - hi - lo - (lnv << 3) - er1 - (ldw << 7) - (er2 << 4) - (b1 << 5)

  # bytes      [ 5 ] [ 6 ] [    7   ]
  csum = csum - ahi - amd - alo - b2

  if ahi == 1:
    csum = csum + 15

  if csum < 0:
    if csum < -256:
      csum = csum + 512
    else:
      csum = csum + 256

  csum = csum % 256

  values = {}
  if CP.flags & MazdaFlags.GEN1:
    values = {
      "LKAS_REQUEST": apply_torque,
      "CTR": ctr,
      "ERR_BIT_1": er1,
      "LINE_NOT_VISIBLE": lnv,
      "LDW": ldw,
      "BIT_1": b1,
      "ERR_BIT_2": er2,
      "STEERING_ANGLE": steering_angle,
      "ANGLE_ENABLED": b2,
      "CHKSUM": csum
    }

  return packer.make_can_msg("CAM_LKAS", 0, values)


def create_alert_command(packer, cam_msg: dict, ldw: bool, steer_required: bool):
  # Preserve camera LKAS state. Keep TJA modes clear because its state machine does not own
  # the injected steering command.
  values = {s: cam_msg[s] for s in [
    "LINE_VISIBLE",
    "LINE_NOT_VISIBLE",
    "LANE_LINES",
    "BIT1",
    "BIT2",
    "BIT3",
    "NO_ERR_BIT",
    "ERR_BIT",
    "S1",
    "S1_HBEAM",
  ]}
  values.update({
    # Mapped on the car (2026-09-30, route 00000267): the cluster draws the hands-on-wheel text
    # only with HANDS_WARN_3_BITS=7 and HANDS_ON_STEER_WARN_2 together. Either alone, or
    # HANDS_ON_STEER_WARN alone, is mirrored by the radar into CRZ_CTRL but draws nothing.
    # The 0b111 code is also what puts the radar into its hands-off state, so the two cannot be
    # separated. The camera's own frames with HANDS_ON_STEER_WARN are lane-departure warnings
    # (LDW_WARN_LL / _RL set alongside), not a hands-off warning.
    "HANDS_WARN_3_BITS": 0b111 if steer_required else 0,
    "HANDS_ON_STEER_WARN": steer_required,
    "HANDS_ON_STEER_WARN_2": steer_required,

    # TODO: right lane works, left doesn't
    # TODO: need to do something about L/R
    "LDW_WARN_LL": 0,
    "LDW_WARN_RL": 0,
  })
  return packer.make_can_msg("CAM_LANEINFO", 0, values)


def create_crz_ctrl_relay(cam_frame: bytes) -> CanData:
  # Copied raw: most of the frame has no DBC signal, and the panda accepts only a byte-exact
  # copy of a recent camera frame.
  dat = bytearray(cam_frame)
  dat[CRZ_CTRL_STEER_CHIME_BYTE] &= ~CRZ_CTRL_STEER_CHIME_MASK & 0xff
  return CanData(CRZ_CTRL_ADDR, bytes(dat), 0)


def create_auto_hold_press(car_frame: bytes) -> CanData:
  # The car's last frame with the button set and the next counter, so it lands ahead of the car's
  # own frame with that counter. The panda accepts nothing else on this address. The same-counter
  # variant never armed on the road (routes 00000036, 00000037); this one does.
  dat = bytearray(car_frame)
  ctr = ((dat[4] >> 4) + 1) & 0xf
  dat[AUTO_HOLD_BTN_BYTE] |= AUTO_HOLD_BTN_MASK
  dat[4] = (ctr << 4) | (dat[4] & 0x0f)
  dat[5] = (dat[5] - 1 - (1 if ctr else -15)) & 0xff
  return CanData(AUTO_HOLD_BTN_ADDR, bytes(dat), 0)


def create_traffic_signs_relay(cam_frame: bytes, limit_kph: int) -> CanData:
  # The camera's sign wins; the map's limit fills the HUD only when the camera shows none.
  dat = bytearray(cam_frame)
  camera_unit = (dat[1] >> 4) & 0x3
  if camera_unit == 0 and TSR_MIN_KPH <= limit_kph <= TSR_MAX_KPH:
    dat[0] = (dat[0] & 0xe0) | ((limit_kph >> 2) & 0x1f)
    dat[1] = (dat[1] & 0x0f) | ((limit_kph & 0x3) << 6) | (TSR_UNIT_KPH << 4)
    dat[TSR_SHOWN_BYTE] |= TSR_SHOWN_MASK
  return CanData(TRAFFIC_SIGNS_ADDR, bytes(dat), 0)


def create_button_cmd(packer, CP, counter, button):

  can = int(button == Buttons.CANCEL)
  res = int(button == Buttons.RESUME)
  inc = int(button == Buttons.SET_PLUS)
  dec = int(button == Buttons.SET_MINUS)

  if CP.flags & MazdaFlags.GEN1:
    values = {
      "CAN_OFF": can,
      "CAN_OFF_INV": (can + 1) % 2,

      "SET_P": inc,
      "SET_P_INV": (inc + 1) % 2,

      "RES": res,
      "RES_INV": (res + 1) % 2,

      "SET_M": dec,
      "SET_M_INV": (dec + 1) % 2,

      "DISTANCE_LESS": 0,
      "DISTANCE_LESS_INV": 1,

      "DISTANCE_MORE": 0,
      "DISTANCE_MORE_INV": 1,

      "MODE_X": 0,
      "MODE_X_INV": 1,

      "MODE_Y": 0,
      "MODE_Y_INV": 1,

      "BIT1": 1,
      "BIT2": 1,
      "BIT3": 1,
      "CTR": (counter + 1) % 16,
    }

    return packer.make_can_msg("CRZ_BTNS", 0, values)
