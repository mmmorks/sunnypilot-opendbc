import types
import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus, gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiSafetyFlags
from opendbc.sunnypilot.car.interfaces import setup_interfaces

# A dynamic-radar-handoff capable HDA II CAN-FD platform.
HANDOFF_CAR = "GENESIS_GV70_ELECTRIFIED_1ST_GEN"
UDS_RESPONSE_MSG = "ADAS_DRV_UDS_RESPONSE"

LFA = 0x12a  # steering command to the MDPS, on E-CAN


def _handoff_car_params(handoff=True):
  """A realistic HDA II CAN-FD CP with openpilot longitudinal + (optionally) the dynamic-handoff bit set."""
  fingerprint = gen_empty_fingerprint()
  fingerprint[CanBus(None, fingerprint).CAM][0x50] = 8  # CANFD_LKA_STEER_MSG
  adas_fw = [CarParams.CarFw(ecu=CarParams.Ecu.adas, fwVersion=b'test', address=0x0, subAddress=0)]
  CP = CarInterface.get_params(HANDOFF_CAR, fingerprint, adas_fw, True, False, False)
  CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
  setup_interfaces(CarInterface, CP, CP_SP, [{"DynamicRadarHandoffEnabled": "1"}, {"AlphaLongitudinalEnabled": "1"}])
  assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF
  if not handoff:
    CP.safetyConfigs[-1].safetyParam = int(CP.safetyConfigs[-1].safetyParam & ~HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
  return CP


def _handoff_pt_parser():
  CP = CarInterface.get_non_essential_params(HANDOFF_CAR)
  CP.safetyConfigs[-1].safetyParam = int(CP.safetyConfigs[-1].safetyParam | HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
  # get_can_parsers_canfd only reads CP, never self
  return CarState.get_can_parsers_canfd(None, CP)[Bus.pt]  # type: ignore[arg-type]


class TestHandoffSteeringHandoff(unittest.TestCase):
  """The LFA (0x12a) steering command must hand off cleanly between openpilot and the ADAS DRV ECU.

  Under dynamic handoff the ECU is restored whenever disengaged and resumes broadcasting its own LFA on E-CAN.
  If openpilot also transmits LFA while disengaged, the two senders' independent rolling counters collide at the
  MDPS -> counter-validation fault -> lane-keep DTC. So openpilot must be the LFA source ONLY while engaged
  (when the ECU is silenced); otherwise the ECU is the sole source.
  """
  def _lfa_count(self, CP, enabled):
    packer = CANPacker("hyundai_canfd_generated")
    CAN = CanBus(CP)
    msgs = hyundaicanfd.create_steering_messages(packer, CP, CAN, enabled, enabled, 0, 0)
    return sum(1 for addr, _, bus in msgs if addr == LFA and bus == CAN.ECAN)

  def test_no_lfa_when_disengaged_under_handoff(self):
    self.assertEqual(self._lfa_count(_handoff_car_params(handoff=True), enabled=False), 0)

  def test_lfa_present_when_engaged_under_handoff(self):
    self.assertEqual(self._lfa_count(_handoff_car_params(handoff=True), enabled=True), 1)

  def test_lfa_always_present_without_handoff(self):
    # Static-disable cars keep the ADAS DRV ECU dead all drive, so openpilot must stay the sole continuous LFA
    # source regardless of engagement -- gating it off would leave the MDPS with no LFA -> timeout fault.
    CP = _handoff_car_params(handoff=False)
    self.assertEqual(self._lfa_count(CP, enabled=False), 1)
    self.assertEqual(self._lfa_count(CP, enabled=True), 1)


class TestHandoffClusterHandoff(unittest.TestCase):
  """LFAHDA_CLUSTER (0x1e0) is the other steering-path message openpilot transmits on E-CAN; under dynamic
  handoff it likewise collides with the restored ECU's broadcast when disengaged, so it must be gated the same
  way as LFA."""
  LFAHDA = 0x1e0

  def _build_controller(self, handoff=True):
    from opendbc.car.hyundai.carcontroller import CarController
    CP = _handoff_car_params(handoff)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, HANDOFF_CAR)
    return CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)

  @staticmethod
  def _fakes(enabled):
    cs = types.SimpleNamespace(lfa_block_msg={f"BYTE{i}": 0 for i in range(3, 32)} | {"COUNTER": 0},
                               main_cruise_enabled=False)
    cc = types.SimpleNamespace(enabled=enabled, leftBlinker=False, rightBlinker=False,
                               cruiseControl=types.SimpleNamespace(override=False))
    return cs, cc

  def _lfahda_count(self, enabled, handoff=True):
    cc_ctrl = self._build_controller(handoff)
    cc_ctrl.frame = 5  # %5==0 so LFAHDA_CLUSTER is due; %2!=0 so the heavy acc_control path is skipped
    cs, cc = self._fakes(enabled)
    msgs = cc_ctrl.create_canfd_msgs(False, 0, 0, 0.0, False, types.SimpleNamespace(), cs, cc)
    return sum(1 for addr, _, _ in msgs if addr == self.LFAHDA)

  def test_no_lfahda_cluster_when_disengaged_under_handoff(self):
    self.assertEqual(self._lfahda_count(enabled=False, handoff=True), 0)

  def test_lfahda_cluster_present_when_engaged_under_handoff(self):
    self.assertEqual(self._lfahda_count(enabled=True, handoff=True), 1)


class TestCanfdDynamicHandoff(unittest.TestCase):
  def test_uds_response_does_not_gate_can_valid(self):
    # The ADAS DRV UDS response (0x738) is sporadic: it only arrives in reply to the carcontroller's
    # engage/disengage-edge requests. It must be registered ignore-alive (NaN frequency) so its absence does
    # not invalidate the bus. Registering it at frequency 0 instead requires it to be seen at least once,
    # which deadlocks onroad: canError -> can't engage -> no request -> no response -> never seen -> canError
    # forever (the canError alert text reads "Unknown Vehicle Variant", so it looks like a fingerprint error).
    pt = _handoff_pt_parser()

    uds_addr = pt.dbc.name_to_msg[UDS_RESPONSE_MSG].address
    self.assertTrue(pt.message_states[uds_addr].ignore_alive,
                    f"{UDS_RESPONSE_MSG} must be registered ignore-alive (NaN freq) so it never gates can_valid")

    # Feed only CRUISE_BUTTONS (the genuinely-periodic checked message); never deliver 0x738.
    packer = CANPacker(pt.dbc_name)
    for i in range(1, 300):  # ~3s @ 100Hz, with valid auto-incrementing counter
      t = int(0.01 * i * 1e9)
      pt.update([t, [packer.make_can_msg("CRUISE_BUTTONS", pt.bus, {})]])

    self.assertTrue(pt.can_valid, "pt parser must be valid without ever receiving the sporadic ADAS_DRV_UDS_RESPONSE")

  def test_uds_response_still_parsed_for_watchdog(self):
    # The watchdog still needs the message parsed so it can read acks/NRCs off cp.vl / cp.ts_nanos.
    pt = _handoff_pt_parser()
    self.assertIn(UDS_RESPONSE_MSG, pt.dbc.name_to_msg)
    self.assertIn(pt.dbc.name_to_msg[UDS_RESPONSE_MSG].address, pt.message_states)


if __name__ == "__main__":
  unittest.main()
