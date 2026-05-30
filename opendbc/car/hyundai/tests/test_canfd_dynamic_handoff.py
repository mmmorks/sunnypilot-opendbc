import unittest

from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.values import HyundaiSafetyFlags

# A dynamic-radar-handoff capable HDA II CAN-FD platform.
HANDOFF_CAR = "GENESIS_GV70_ELECTRIFIED_1ST_GEN"
UDS_RESPONSE_MSG = "ADAS_DRV_UDS_RESPONSE"


def _handoff_pt_parser():
  CP = CarInterface.get_non_essential_params(HANDOFF_CAR)
  CP.safetyConfigs[-1].safetyParam = int(CP.safetyConfigs[-1].safetyParam | HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
  # get_can_parsers_canfd only reads CP, never self
  return CarState.get_can_parsers_canfd(None, CP)[Bus.pt]  # type: ignore[arg-type]


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
