"""
Auto Hold pressed on the driver's behalf.

The Mazda forgets Auto Hold at every ignition. With the toggle on, the controller copies the
car's own 0x203 frame with the button set, the next counter and the matching checksum, one copy
per car frame for AUTO_HOLD_PRESS_T, stopped with the brake held, until 0x079 reads armed. The
first armed reading ends it for the drive, so a driver who switches Auto Hold off keeps it off.
"""
from opendbc.car import DT_CTRL
from opendbc.car.can_definitions import CanData
from opendbc.car.mazda import mazdacan
from opendbc.car.mazda.tests.conftest import car_controller, frames, mazda_car_state, step
from opendbc.car.mazda.values import CAR, CarControllerParams

BTN = mazdacan.AUTO_HOLD_BTN_ADDR
PRESS_STEPS = int(CarControllerParams.AUTO_HOLD_PRESS_T / DT_CTRL)
RETRY_STEPS = int(CarControllerParams.AUTO_HOLD_RETRY_T / DT_CTRL)
# Pressed frames captured on the car (routes 00000023-27), each paired with the released frame one
# counter earlier.
RUNNING_RELEASED = bytes.fromhex("4000a65610010800")
RUNNING_PRESSED = bytes.fromhex("4000a65720ff0800")
STARTING_RELEASED = bytes.fromhex("0000a640001a0000")
STARTING_PRESSED = bytes.fromhex("0000a64110180000")
READY = {"standstill": True, "brake_pressed": True, "auto_hold": True}


def rig(armed=False):
  cc = car_controller(alpha_long=False, candidate=CAR.MAZDA_CX5_2022_NON_MRCC)
  cs = mazda_car_state(cc.CP, cc.CP_SP)
  cs.auto_hold_armed = armed
  return cc, cs


def car(cs, dat=RUNNING_RELEASED):
  cs.auto_hold_btn = dat
  cs.auto_hold_btn_frames += 1


def drive(cc, cs, n, **kwargs):
  """n controller steps with a car frame every other one (50 Hz vs 100 Hz); returns presses sent."""
  sent = []
  for i in range(n):
    if i % 2 == 0:
      car(cs)
    sent += frames(step(cc, cs, **kwargs)[1], BTN)
  return sent


class TestPressFrame:

  def test_matches_the_drivers_own_press(self):
    assert mazdacan.create_auto_hold_press(RUNNING_RELEASED) == CanData(BTN, RUNNING_PRESSED, 0)
    assert mazdacan.create_auto_hold_press(STARTING_RELEASED).dat == STARTING_PRESSED

  def test_counter_wraps_and_checksum_follows(self):
    released_ctr15 = bytes.fromhex("4000a656f0f30800")
    assert mazdacan.create_auto_hold_press(released_ctr15).dat == bytes.fromhex("4000a65700010800")


class TestController:

  def test_off_unless_the_toggle_is_on(self):
    cc, cs = rig()
    assert not drive(cc, cs, 50, standstill=True, brake_pressed=True)

  def test_one_copy_per_car_frame_while_pressing(self):
    cc, cs = rig()
    car(cs)
    assert frames(step(cc, cs, **READY)[1], BTN) == [RUNNING_PRESSED]
    assert not frames(step(cc, cs, **READY)[1], BTN), "no new car frame, no copy"

  def test_needs_standstill_brake_and_a_known_unarmed_state(self):
    for kwargs, armed in (({"standstill": False, "brake_pressed": True}, False),
                          ({"standstill": True, "brake_pressed": False}, False),
                          ({"standstill": True, "brake_pressed": True}, None),
                          ({"standstill": True, "brake_pressed": True}, True)):
      cc, cs = rig(armed=armed)
      assert not drive(cc, cs, 50, auto_hold=True, **kwargs), (kwargs, armed)

  def test_press_length_retry_gap_and_cap(self):
    cc, cs = rig()
    sent = drive(cc, cs, CarControllerParams.AUTO_HOLD_PRESS_MAX * (PRESS_STEPS + RETRY_STEPS) + 400, **READY)
    per_press = (PRESS_STEPS + 1) // 2   # car frames arrive every other step
    assert len(sent) == CarControllerParams.AUTO_HOLD_PRESS_MAX * per_press
    assert cc.auto_hold_attempts == CarControllerParams.AUTO_HOLD_PRESS_MAX

  def test_first_armed_reading_ends_it_for_the_drive(self):
    cc, cs = rig()
    assert drive(cc, cs, 10, **READY)
    cs.auto_hold_armed = True
    assert not drive(cc, cs, 10, **READY)
    # the driver switches it off again: it stays off
    cs.auto_hold_armed = False
    assert not drive(cc, cs, 2 * (PRESS_STEPS + RETRY_STEPS), **READY)

  def test_nothing_while_the_driver_holds_the_button(self):
    cc, cs = rig()
    car(cs, RUNNING_PRESSED)
    assert not frames(step(cc, cs, **READY)[1], BTN)
