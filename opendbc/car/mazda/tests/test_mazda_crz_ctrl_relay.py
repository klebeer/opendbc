"""
The CRZ_CTRL relay on a car without MRCC.

The camera publishes CRZ_CTRL there and sets DBC bit 39 while another controller applies
steering torque; the cluster chimes for as long as it stays set. The controller sends the
camera's frame back, byte for byte, with that bit cleared: one copy per camera frame. The panda
forwards the camera's frame and drops the copy until openpilot is steering, then swaps them.
"""
from opendbc.car import structs
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.conftest import car_control, car_control_sp, car_controller, car_interface, frames, mazda_car_state, step
from opendbc.car.mazda.values import CAR

CRZ_CTRL = mazdacan.CRZ_CTRL_ADDR
# camera frames captured on the car (route 0000001e--eb2f9b1e95)
CAM_CHIME = bytes.fromhex("0201000a80000000")
CAM_QUIET = bytes.fromhex("0201000c00000000")


def rig():
  cc = car_controller(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  return cc, cs


def camera(cs, dat):
  cs.cam_crz_ctrl = dat
  cs.cam_crz_ctrl_frames += 1


class TestRelayFrame:

  def test_clears_only_the_chime_bit(self):
    assert mazdacan.create_crz_ctrl_relay(CAM_CHIME) == CanData(CRZ_CTRL, bytes.fromhex("0201000a00000000"), 0)

  def test_frame_without_the_chime_passes_unchanged(self):
    assert mazdacan.create_crz_ctrl_relay(CAM_QUIET).dat == CAM_QUIET


class TestController:

  def test_nothing_before_the_first_camera_frame(self):
    cc, cs = rig()
    _, sends = step(cc, cs, lat_active=True)
    assert not frames(sends, CRZ_CTRL)

  def test_one_copy_per_camera_frame(self):
    cc, cs = rig()
    camera(cs, CAM_CHIME)
    _, sends = step(cc, cs, lat_active=True)
    assert frames(sends, CRZ_CTRL) == [bytes.fromhex("0201000a00000000")]
    # controls run at 100 Hz, the camera at 50 Hz: no new frame, no copy
    _, sends = step(cc, cs, lat_active=True)
    assert not frames(sends, CRZ_CTRL)
    camera(cs, CAM_QUIET)
    _, sends = step(cc, cs, lat_active=True)
    assert frames(sends, CRZ_CTRL) == [CAM_QUIET]

  def test_sent_while_not_steering_too(self):
    # the panda decides which of the two frames reaches the car, on its own controls state
    cc, cs = rig()
    camera(cs, CAM_CHIME)
    _, sends = step(cc, cs, lat_active=False)
    assert len(frames(sends, CRZ_CTRL)) == 1

  def test_never_on_the_camera_bus(self):
    cc, cs = rig()
    camera(cs, CAM_CHIME)
    _, sends = step(cc, cs, lat_active=True)
    assert not frames(sends, CRZ_CTRL, bus=2)


class TestCapture:

  def _packets(self, *frames_):
    return [(0, [CanData(addr, dat, src) for addr, dat, src in frames_])]

  def test_camera_frame_captured_raw(self):
    ci = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
    ci.update(self._packets((CRZ_CTRL, CAM_CHIME, 2)))
    assert ci.CS.cam_crz_ctrl == CAM_CHIME
    assert ci.CS.cam_crz_ctrl_frames == 1

  def test_plain_tuples_as_card_sends_them(self):
    # openpilot's card converts capnp to (address, dat, src) tuples, not CanData
    ci = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
    ci.update([(0, [(CRZ_CTRL, CAM_CHIME, 2)])])
    assert ci.CS.cam_crz_ctrl == CAM_CHIME

  def test_car_side_frame_ignored(self):
    # the body's own CRZ_CTRL in the first seconds after ignition is not the camera's
    ci = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
    ci.update(self._packets((CRZ_CTRL, CAM_CHIME, 0)))
    assert ci.CS.cam_crz_ctrl is None

  def test_mrcc_car_captures_nothing(self):
    ci = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5_2022)
    ci.update(self._packets((CRZ_CTRL, CAM_CHIME, 2)))
    assert ci.CS.cam_crz_ctrl is None


TRAFFIC_SIGNS = mazdacan.TRAFFIC_SIGNS_ADDR
CAM_NO_SIGN = bytes.fromhex("00000000005c0000")


def with_limit(kph):
  cc_sp = car_control_sp()
  cc_sp.params = [structs.CarControlSP.Param(key=mazdacan.HUD_SPEED_LIMIT_PARAM, value=str(kph).encode(),
                                             type=structs.CarControlSP.ParamType.int)]
  return cc_sp


class TestTrafficSignsRelay:

  def test_map_limit_matches_a_real_camera_50_kph_frame(self):
    out = mazdacan.create_traffic_signs_relay(bytes.fromhex("0000000002000900"), 50)
    assert out == CanData(TRAFFIC_SIGNS, bytes.fromhex("0ca0000002000900"), 0)

  def test_camera_sign_wins(self):
    cam = bytes.fromhex("1920000002010900")  # camera reads 100 km/h
    assert mazdacan.create_traffic_signs_relay(cam, 50).dat == cam

  def test_no_limit_or_implausible_copies_the_camera(self):
    for kph in (0, 5, 125):
      assert mazdacan.create_traffic_signs_relay(CAM_NO_SIGN, kph).dat == CAM_NO_SIGN

  def test_controller_sends_one_per_camera_frame_with_the_map_limit(self):
    cc, cs = rig()
    cs.cam_traffic_signs = CAM_NO_SIGN
    cs.cam_traffic_signs_frames = 1
    _, sends = cc.update(car_control(lat_active=True), with_limit(50), cs, 0)
    assert frames(sends, TRAFFIC_SIGNS) == [bytes.fromhex("0ca00000005c0000")]
    _, sends = cc.update(car_control(lat_active=True), with_limit(50), cs, 0)
    assert not frames(sends, TRAFFIC_SIGNS)

  def test_capture_from_card_tuples(self):
    ci = car_interface(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
    ci.update([(0, [(TRAFFIC_SIGNS, CAM_NO_SIGN, 2)])])
    assert ci.CS.cam_traffic_signs == CAM_NO_SIGN
