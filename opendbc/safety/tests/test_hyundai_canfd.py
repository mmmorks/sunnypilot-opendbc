#!/usr/bin/env python3
from opendbc.testing import parameterized_class
import unittest

from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety
from opendbc.safety.tests.hyundai_common import HyundaiButtonBase, HyundaiLongitudinalBase

# All combinations of radar/camera-SCC and gas/hybrid/EV cars
ALL_GAS_EV_HYBRID_COMBOS = [
  # Radar SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 0, "SAFETY_PARAM": 0},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 0, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS},
  # Camera SCC
  {"GAS_MSG": ("ACCELERATOR_BRAKE_ALT", "ACCELERATOR_PEDAL_PRESSED"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.EV_GAS | HyundaiSafetyFlags.CAMERA_SCC},
  {"GAS_MSG": ("ACCELERATOR_ALT", "ACCELERATOR_PEDAL"), "SCC_BUS": 2, "SAFETY_PARAM": HyundaiSafetyFlags.HYBRID_GAS | HyundaiSafetyFlags.CAMERA_SCC},
]


class TestHyundaiCanfdBase(HyundaiButtonBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest, common.SteerRequestCutSafetyTest):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  STANDSTILL_THRESHOLD = 12  # 0.375 kph
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  MAX_RATE_UP = 2
  MAX_RATE_DOWN = 3
  MAX_TORQUE_LOOKUP = [0], [270]

  MAX_RT_DELTA = 112

  DRIVER_TORQUE_ALLOWANCE = 250
  DRIVER_TORQUE_FACTOR = 2

  # Safety around steering req bit
  MIN_VALID_STEERING_FRAMES = 89
  MAX_INVALID_STEERING_FRAMES = 2

  PT_BUS = 0
  SCC_BUS = 2
  STEER_BUS = 0
  STEER_MSG = ""
  GAS_MSG = ("", "")
  BUTTONS_TX_BUS = 1

  def _torque_driver_msg(self, torque):
    values = {"STEERING_COL_TORQUE": torque}
    return self.packer.make_can_msg_safety("MDPS", self.PT_BUS, values)

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"TORQUE_REQUEST": torque, "STEER_REQ": steer_req}
    return self.packer.make_can_msg_safety(self.STEER_MSG, self.STEER_BUS, values)

  def _speed_msg(self, speed):
    values = {f"WHL_Spd{pos}Val": speed * 0.03125 for pos in ["FL", "FR", "RL", "RR"]}
    return self.packer.make_can_msg_safety("WHEEL_SPEEDS", self.PT_BUS, values)

  def _user_brake_msg(self, brake):
    values = {"DriverBraking": brake}
    return self.packer.make_can_msg_safety("TCS", self.PT_BUS, values)

  def _user_gas_msg(self, gas):
    values = {self.GAS_MSG[1]: gas}
    return self.packer.make_can_msg_safety(self.GAS_MSG[0], self.PT_BUS, values)

  def _pcm_status_msg(self, enable):
    values = {"ACCMode": 1 if enable else 0}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _button_msg(self, buttons, main_button=0, bus=None):
    if bus is None:
      bus = self.PT_BUS
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", bus, values)

  def _acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.SCC_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS", self.PT_BUS, values)

  def _main_cruise_button_msg(self, enabled):
    return self._button_msg(0, enabled)


class TestHyundaiCanfdLFASteeringBase(TestHyundaiCanfdBase):

  TX_MSGS = [[0x12A, 0], [0x1A0, 1], [0x1CF, 0], [0x1E0, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0)}  # LFA, LFAHDA_CLUSTER
  FWD_BLACKLISTED_ADDRS = {2: [0x12A, 0x1E0]}

  STEER_MSG = "LFA"
  BUTTONS_TX_BUS = 2
  SAFETY_PARAM: int

  @classmethod
  def setUpClass(cls):
    super().setUpClass()
    if cls.__name__ in ("TestHyundaiCanfdLFASteering", "TestHyundaiCanfdLFASteeringAltButtons"):
      cls.packer = None
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, self.SAFETY_PARAM)
    self.safety.init_tests()


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteering(TestHyundaiCanfdLFASteeringBase):
  pass


class TestHyundaiCanfdLFASteeringAltButtonsBase(TestHyundaiCanfdLFASteeringBase):

  SAFETY_PARAM: int

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _button_msg(self, buttons, main_button=0, bus=1):
    values = {
      "CRUISE_BUTTONS": buttons,
      "ADAPTIVE_CRUISE_MAIN_BTN": main_button,
    }
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _lkas_button_msg(self, enabled):
    values = {"LDA_BTN": enabled}
    return self.packer.make_can_msg_safety("CRUISE_BUTTONS_ALT", self.PT_BUS, values)

  def _acc_cancel_msg(self, cancel, accel=0):
    values = {"ACCMode": 4 if cancel else 0, "aReqRaw": accel, "aReqValue": accel}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def test_button_sends(self):
    """
      No button send allowed with alt buttons.
    """
    for enabled in (True, False):
      for btn in range(8):
        self.safety.set_controls_allowed(enabled)
        self.assertFalse(self._tx(self._button_msg(btn)))

  def test_acc_cancel(self):
    # FIXME: the CANFD_ALT_BUTTONS cars are the only ones that use SCC_CONTROL to cancel, why can't we use buttons?
    for enabled in (True, False):
      self.safety.set_controls_allowed(enabled)
      self.assertTrue(self._tx(self._acc_cancel_msg(True)))
      self.assertFalse(self._tx(self._acc_cancel_msg(True, accel=1)))
      self.assertFalse(self._tx(self._acc_cancel_msg(False)))


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringAltButtons(TestHyundaiCanfdLFASteeringAltButtonsBase):
  pass


class TestHyundaiCanfdLKASteeringEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}  # LKAS, CAM_0x2A4
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()


# TODO: Handle ICE and HEV configurations once we see cars that use the new messages
class TestHyundaiCanfdLKASteeringAltEV(TestHyundaiCanfdBase):

  TX_MSGS = [[0x110, 0], [0x1CF, 1], [0x362, 0]]
  RELAY_MALFUNCTION_ADDRS = {0: (0x110, 0x362)}  # LKAS_ALT, CAM_0x362
  FWD_BLACKLISTED_ADDRS = {2: [0x110, 0x362]}

  PT_BUS = 1
  SCC_BUS = 1
  STEER_MSG = "LKAS_ALT"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT)
    self.safety.init_tests()


class TestHyundaiCanfdLKASteeringLongEV(HyundaiLongitudinalBase, TestHyundaiCanfdLKASteeringEV):

  TX_MSGS = [[0x50, 0], [0x1CF, 1], [0x2A4, 0], [0x51, 0], [0x730, 1], [0x12a, 1], [0x160, 1],
             [0x1e0, 1], [0x1a0, 1], [0x1ea, 1], [0x200, 1], [0x345, 1], [0x1da, 1]]

  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4), 1: (0x1a0,)}  # LKAS, CAM_0x2A4, SCC_CONTROL

  # Longitudinal addresses blocked by the fwd hook on both forwarding directions.
  # fwd hook blocks these regardless of source bus when longitudinal + LKA-steering.
  _LONG_FWD_BLOCKED = [0x51, 0x160, 0x1a0, 0x1da, 0x1ea, 0x200, 0x345]
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4] + _LONG_FWD_BLOCKED, 0: _LONG_FWD_BLOCKED}

  DISABLED_ECU_UDS_MSG = (0x730, 1)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 1)

  STEER_MSG = "LFA"
  GAS_MSG = ("ACCELERATOR", "ACCELERATOR_PEDAL")
  STEER_BUS = 1

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)


# Tests longitudinal for ICE, hybrid, EV cars with LFA steering
class TestHyundaiCanfdLFASteeringLongBase(HyundaiLongitudinalBase, TestHyundaiCanfdLFASteeringBase):

  FWD_BLACKLISTED_ADDRS = {2: [0x12a, 0x1e0, 0x1a0, 0x160]}

  RELAY_MALFUNCTION_ADDRS = {0: (0x12A, 0x1E0, 0x1a0, 0x160)}  # LFA, LFAHDA_CLUSTER, SCC_CONTROL, ADRV_0x160

  DISABLED_ECU_UDS_MSG = (0x7D0, 0)
  DISABLED_ECU_ACTUATION_MSG = (0x1a0, 0)

  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongBase":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | self.SAFETY_PARAM)
    self.safety.init_tests()

  def _accel_msg(self, accel, aeb_req=False, aeb_decel=0):
    values = {
      "aReqRaw": accel,
      "aReqValue": accel,
    }
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  def _tx_acc_state_msg(self, enable):
    values = {"MainMode_ACC": enable}
    return self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, values)

  # no knockout
  def test_tester_present_allowed(self):
    pass


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLong(TestHyundaiCanfdLFASteeringLongBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLong":
      cls.safety = None
      raise unittest.SkipTest


@parameterized_class(ALL_GAS_EV_HYBRID_COMBOS)
class TestHyundaiCanfdLFASteeringLongAltButtons(TestHyundaiCanfdLFASteeringLongBase, TestHyundaiCanfdLFASteeringAltButtonsBase):
  @classmethod
  def setUpClass(cls):
    if cls.__name__ == "TestHyundaiCanfdLFASteeringLongAltButtons":
      cls.safety = None
      raise unittest.SkipTest

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CANFD_ALT_BUTTONS | self.SAFETY_PARAM)
    self.safety.init_tests()

  def test_acc_cancel(self):
    # Alt buttons does not use SCC_CONTROL to cancel if longitudinal
    pass


class TestHyundaiCanfdLKASteeringLongDynamicHandoff(TestHyundaiCanfdLKASteeringLongEV):
  """
  Verifies that when CANFD_DYNAMIC_HANDOFF is set, SCC_CONTROL and ADRV addresses
  are rejected when controls_allowed=False, and accepted when controls_allowed=True.
  Mirrors TestHyundaiCanfdLKASteeringLongEV but with the dynamic handoff flag added.
  """

  # With dynamic handoff and controls_allowed=False (test_fwd_hook default), the fwd hook
  # forwards longitudinal addresses to let stock SCC pass through. Only LKAS outputs are blocked.
  FWD_BLACKLISTED_ADDRS = {2: [0x50, 0x2a4]}

  # Under dynamic handoff the ADAS DRV ECU is intentionally kept alive and broadcasts stock SCC_CONTROL
  # (0x1a0) on E-CAN whenever disengaged, so its presence on the bus is NOT a relay-malfunction signal here
  # (the handoff TX list drops check_relay on 0x1a0). The lateral/camera relay addrs still apply.
  RELAY_MALFUNCTION_ADDRS = {0: (0x50, 0x2a4)}

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS |
                                 HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    self.safety.init_tests()

  def test_disabled_ecu_alive(self):
    """Under dynamic handoff the ADAS DRV ECU is intentionally NOT knocked out — it stays alive and broadcasts
    stock SCC_CONTROL (0x1a0) on E-CAN whenever disengaged. That must NOT trigger relay malfunction, otherwise
    panda blocks all tx and all forwarding (including the stock SCC/AEB the handoff is meant to preserve)."""
    addr, bus = self.DISABLED_ECU_ACTUATION_MSG
    self.assertFalse(self.safety.get_relay_malfunction())
    for _ in range(10):
      self._rx(common.make_msg(bus, addr, 8))
    self.assertFalse(self.safety.get_relay_malfunction())

  def test_dynamic_handoff_scc_control_blocked_without_controls_allowed(self):
    """SCC_CONTROL must be rejected when controls_allowed=False under dynamic handoff."""
    self.safety.set_controls_allowed(False)
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 0.0, "aReqValue": 0.0})
    self.assertFalse(self._tx(msg))

  def test_dynamic_handoff_scc_control_allowed_with_controls_allowed(self):
    """SCC_CONTROL must be accepted when controls_allowed=True under dynamic handoff."""
    self.safety.set_controls_allowed(True)
    # Use 0 accel (inactive) so longitudinal_accel_checks passes
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 0.0, "aReqValue": 0.0})
    self.assertTrue(self._tx(msg))

  def test_dynamic_handoff_adrv_0x160_blocked_without_controls_allowed(self):
    """ADRV address 0x160 must be rejected when controls_allowed=False under dynamic handoff."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(False)
    msg = _lspy.make_CANPacket(0x160, 1, b'\x00' * 16)
    self.assertFalse(self._tx(msg))

  def test_dynamic_handoff_adrv_0x160_allowed_with_controls_allowed(self):
    """ADRV address 0x160 must be accepted when controls_allowed=True under dynamic handoff."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(True)
    msg = _lspy.make_CANPacket(0x160, 1, b'\x00' * 16)
    self.assertTrue(self._tx(msg))

  def test_fwd_hook_scc_control_blocked_when_controls_allowed(self):
    """When engaged (controls_allowed=True), stock SCC_CONTROL must NOT be forwarded to vehicle CAN."""
    self.safety.set_controls_allowed(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1A0))

  def test_fwd_hook_scc_control_forwarded_when_not_controls_allowed(self):
    """When disengaged (controls_allowed=False), stock SCC_CONTROL must be forwarded to vehicle CAN."""
    self.safety.set_controls_allowed(False)
    result = self.safety.safety_fwd_hook(2, 0x1A0)
    self.assertNotEqual(-1, result)

  def test_fwd_hook_adrv_0x160_blocked_when_controls_allowed(self):
    """When engaged (controls_allowed=True), stock ADRV 0x160 must NOT be forwarded to vehicle CAN."""
    self.safety.set_controls_allowed(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x160))

  def test_fwd_hook_adrv_0x160_forwarded_when_not_controls_allowed(self):
    """When disengaged (controls_allowed=False), stock ADRV 0x160 must be forwarded to vehicle CAN."""
    self.safety.set_controls_allowed(False)
    result = self.safety.safety_fwd_hook(2, 0x160)
    self.assertNotEqual(-1, result)

  def test_uds_path_730_tx_allowed_when_disengaged(self):
    """0x730 (UDS tester-present) must be accepted by TX hook regardless of controls_allowed."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(False)
    msg = _lspy.make_CANPacket(0x730, 1, b"\x02\x3E\x80\x00\x00\x00\x00\x00")
    self.assertTrue(self._tx(msg))

  def test_uds_path_730_tx_allowed_when_engaged(self):
    """0x730 (UDS tester-present) must be accepted by TX hook regardless of controls_allowed."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(True)
    msg = _lspy.make_CANPacket(0x730, 1, b"\x02\x3E\x80\x00\x00\x00\x00\x00")
    self.assertTrue(self._tx(msg))

  def test_uds_730_handoff_frames_allowed(self):
    """Under dynamic handoff, 0x730 accepts the non-suppress UDS frames the carcontroller emits.

    The restore/session frames are allowed in any state; the SILENCING frame (0x28 disableRxAndTx) is only
    allowed while controls_allowed so a stray edge cannot disable the stock SCC/AEB when openpilot is not the
    longitudinal authority. Suppress-response is deliberately NOT set so positive acks and NRCs land on 0x738.
    """
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    always_allowed = (
      b"\x02\x10\x03\x00\x00\x00\x00\x00",  # 0x10 extendedDiagnosticSession (engage step 1) — does not silence
      b"\x03\x28\x00\x01\x00\x00\x00\x00",  # 0x28 enableRxAndTx             (disengage step 1) — restores stock
      b"\x02\x10\x01\x00\x00\x00\x00\x00",  # 0x10 defaultSession            (disengage step 2) — restores stock
    )
    silencing_frame = b"\x03\x28\x03\x01\x00\x00\x00\x00"  # 0x28 disableRxAndTx (engage step 2) — engaged only
    for controls_allowed in (False, True):
      self.safety.set_controls_allowed(controls_allowed)
      for payload in always_allowed:
        msg = _lspy.make_CANPacket(0x730, 1, payload)
        self.assertTrue(self._tx(msg), f"{payload.hex()} must pass with controls_allowed={controls_allowed}")

    # disableRxAndTx: rejected when disengaged, accepted when engaged.
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(_lspy.make_CANPacket(0x730, 1, silencing_frame)),
                     "disableRxAndTx must be rejected when controls_allowed=False")
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(_lspy.make_CANPacket(0x730, 1, silencing_frame)),
                    "disableRxAndTx must be accepted when controls_allowed=True")

  def test_uds_730_other_payloads_blocked(self):
    """Under dynamic handoff, 0x730 still rejects payloads not on the exact allowlist (including the
    suppress-response variants of the handoff frames — the carcontroller no longer emits those)."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(False)
    for payload in (
      b"\x02\x10\x83\x00\x00\x00\x00\x00",  # 0x10 extendedSession WITH suppress — old pattern, no longer permitted
      b"\x03\x28\x83\x01\x00\x00\x00\x00",  # 0x28 disableRxAndTx WITH suppress  — old pattern, no longer permitted
      b"\x03\x28\x80\x01\x00\x00\x00\x00",  # 0x28 enableRxAndTx  WITH suppress  — old pattern, no longer permitted
      b"\x02\x10\x81\x00\x00\x00\x00\x00",  # 0x10 defaultSession WITH suppress  — old pattern, no longer permitted
      b"\x03\x28\x00\x00\x00\x00\x00\x00",  # 0x28 with wrong communicationType
      b"\x02\x11\x03\x00\x00\x00\x00\x00",  # ECU reset — never permitted
    ):
      msg = _lspy.make_CANPacket(0x730, 1, payload)
      self.assertFalse(self._tx(msg), f"payload {payload.hex()} must be rejected")

  def test_accel_actuation_limits(self):
    """
    Under dynamic handoff, ALL SCC_CONTROL is blocked when controls_allowed=False —
    including inactive accel (0.0). Override the base test to reflect this stricter policy.
    """
    import numpy as np
    from opendbc.safety.tests.common import ALTERNATIVE_EXPERIENCE
    limits = ((self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.DEFAULT),
              (self.MIN_ACCEL, self.MAX_ACCEL, ALTERNATIVE_EXPERIENCE.RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX))
    for min_accel, max_accel, alternative_experience in limits:
      for accel in np.concatenate((np.arange(min_accel - 1, max_accel + 1, 0.05), [0, self.INACTIVE_ACCEL])):
        accel = round(accel, 2)
        for controls_allowed in [True, False]:
          self.safety.set_controls_allowed(controls_allowed)
          self.safety.set_alternative_experience(alternative_experience)
          # With dynamic handoff: nothing passes when controls_allowed=False
          if controls_allowed:
            should_tx = min_accel <= accel <= max_accel or accel == self.INACTIVE_ACCEL
          else:
            should_tx = False
          self.assertEqual(should_tx, self._tx(self._accel_msg(accel)))

  def test_acc_main_sync_mismatches_reset(self):
    """
    Under dynamic handoff, SCC_CONTROL TX is blocked when controls_allowed=False,
    so the acc_main_on sync mechanism via _tx_acc_state_msg does not fire.
    This test is not applicable in dynamic handoff mode.
    """
    pass

  def test_acc_main_sync_mismatch_counter(self):
    """
    Under dynamic handoff, SCC_CONTROL TX is blocked when controls_allowed=False,
    so the acc_main_on mismatch counter cannot be driven via TX. Not applicable here.
    """
    pass

  def test_acc_main_sync_mismatch_recovery(self):
    """
    Under dynamic handoff, SCC_CONTROL TX is blocked when controls_allowed=False,
    so the acc_main_on mismatch recovery cannot be driven via TX. Not applicable here.
    """
    pass


class TestHyundaiCanfdNoHandoffRegression(TestHyundaiCanfdLKASteeringLongEV):
  """
  Regression test for alpha-long without dynamic handoff (CANFD_DYNAMIC_HANDOFF unset).
  Verifies that when the handoff bit is NOT set, the safety code behaves exactly as before:
  - SCC_CONTROL is accepted regardless of controls_allowed state (as long as accel is valid)
  - Stock SCC frames (0x1A0) are NOT forwarded in either state (forward hook returns -1)
  """

  def setUp(self):
    self.packer = CANPackerSafety("hyundai_canfd_generated")
    self.safety = libsafety_py.libsafety
    # NO CANFD_DYNAMIC_HANDOFF flag — just LONG | CANFD_LKA_STEERING | EV_GAS
    self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiCanfd, HyundaiSafetyFlags.CANFD_LKA_STEERING |
                                 HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.EV_GAS)
    self.safety.init_tests()

  def test_scc_control_tx_allowed_when_disengaged(self):
    """Without dynamic handoff, SCC_CONTROL with zero accel is accepted when controls_allowed=False."""
    self.safety.set_controls_allowed(False)
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 0.0, "aReqValue": 0.0})
    self.assertTrue(self._tx(msg))

  def test_scc_control_tx_allowed_when_engaged(self):
    """Without dynamic handoff, SCC_CONTROL with zero accel is accepted when controls_allowed=True."""
    self.safety.set_controls_allowed(True)
    msg = self.packer.make_can_msg_safety("SCC_CONTROL", self.PT_BUS, {"aReqRaw": 0.0, "aReqValue": 0.0})
    self.assertTrue(self._tx(msg))

  def test_stock_scc_blocked_when_disengaged(self):
    """Without dynamic handoff, stock SCC (0x1A0) must be blocked regardless of controls_allowed state."""
    self.safety.set_controls_allowed(False)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1A0))

  def test_stock_scc_blocked_when_engaged(self):
    """Without dynamic handoff, stock SCC (0x1A0) must be blocked regardless of controls_allowed state."""
    self.safety.set_controls_allowed(True)
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, 0x1A0))

  def test_uds_730_handoff_frames_blocked_without_handoff(self):
    """Without dynamic handoff, none of the handoff UDS frames are permitted on 0x730 (strict tester-present-only policy)."""
    from opendbc.safety.tests.libsafety import libsafety_py as _lspy
    self.safety.set_controls_allowed(False)
    for payload in (
      b"\x02\x10\x03\x00\x00\x00\x00\x00",
      b"\x03\x28\x03\x01\x00\x00\x00\x00",
      b"\x03\x28\x00\x01\x00\x00\x00\x00",
      b"\x02\x10\x01\x00\x00\x00\x00\x00",
    ):
      msg = _lspy.make_CANPacket(0x730, 1, payload)
      self.assertFalse(self._tx(msg), f"{payload.hex()} must be rejected when handoff bit is unset")


if __name__ == "__main__":
  unittest.main()
