from hypothesis import settings, given, strategies as st

import unittest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.structs import CarParams
from opendbc.car.fw_versions import build_fw_dict
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.radar_interface import RADAR_START_ADDR
from opendbc.car.hyundai.values import CAMERA_SCC_CAR, CANFD_CAR, CAN_GEARS, CAR, CHECKSUM, DATE_FW_ECUS, \
                                         HYBRID_CAR, EV_CAR, FW_QUERY_CONFIG, LEGACY_SAFETY_MODE_CAR, CANFD_FUZZY_WHITELIST, \
                                         UNSUPPORTED_LONGITUDINAL_CAR, PLATFORM_CODE_ECUS, HYUNDAI_VERSION_REQUEST_LONG, \
                                         HyundaiFlags, get_platform_codes, HyundaiSafetyFlags, \
                                         NON_SCC_CAR
from opendbc.car.hyundai.fingerprints import FW_VERSIONS
from opendbc.sunnypilot.car.interfaces import setup_interfaces

Ecu = CarParams.Ecu

# Some platforms have date codes in a different format we don't yet parse (or are missing).
# For now, assert list of expected missing date cars
NO_DATES_PLATFORMS = {
  # CAN FD
  CAR.KIA_SPORTAGE_5TH_GEN,
  CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN,
  CAR.HYUNDAI_TUCSON_4TH_GEN,
  # CAN
  CAR.HYUNDAI_ELANTRA,
  CAR.HYUNDAI_ELANTRA_GT_I30,
  CAR.KIA_CEED,
  CAR.KIA_FORTE,
  CAR.KIA_OPTIMA_G4,
  CAR.KIA_OPTIMA_G4_FL,
  CAR.KIA_SORENTO,
  CAR.HYUNDAI_KONA,
  CAR.HYUNDAI_KONA_EV,
  CAR.HYUNDAI_KONA_EV_2022,
  CAR.HYUNDAI_KONA_HEV,
  CAR.HYUNDAI_SONATA_LF,
  CAR.HYUNDAI_VELOSTER,
  CAR.HYUNDAI_KONA_2022,
}

CANFD_EXPECTED_ECUS = {Ecu.fwdCamera, Ecu.fwdRadar}


class TestHyundaiFingerprint(unittest.TestCase):
  def test_feature_detection(self):
    # LKA steering
    for lka_steering in (True, False):
      fingerprint = gen_empty_fingerprint()
      if lka_steering:
        cam_can = CanBus(None, fingerprint).CAM
        fingerprint[cam_can] = [0x50, 0x110]  # LKA steering messages
      CP = CarInterface.get_params(CAR.KIA_EV6, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.CANFD_LKA_STEERING) == lka_steering

    # radar available
    for radar in (True, False):
      fingerprint = gen_empty_fingerprint()
      if radar:
        fingerprint[1][RADAR_START_ADDR] = 8
      CP = CarInterface.get_params(CAR.HYUNDAI_SONATA, fingerprint, [], False, False, False)
      assert CP.radarUnavailable != radar

  def test_alternate_limits(self):
    # Alternate lateral control limits, for high torque cars, verify Panda safety mode flag is set
    fingerprint = gen_empty_fingerprint()
    for car_model in CAR:
      CP = CarInterface.get_params(car_model, fingerprint, [], False, False, False)
      assert bool(CP.flags & HyundaiFlags.ALT_LIMITS) == bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.ALT_LIMITS)

  def test_can_features(self):
    # Test no EV/HEV in any gear lists (should all use ELECT_GEAR)
    assert set.union(*CAN_GEARS.values()) & (HYBRID_CAR | EV_CAR) == set()

    # Test CAN FD car not in CAN feature lists
    can_specific_feature_list = set.union(*CAN_GEARS.values(), *CHECKSUM.values(), LEGACY_SAFETY_MODE_CAR, UNSUPPORTED_LONGITUDINAL_CAR, CAMERA_SCC_CAR)
    for car_model in CANFD_CAR:
      assert car_model not in can_specific_feature_list, "CAN FD car unexpectedly found in a CAN feature list"

  def test_hybrid_ev_sets(self):
    assert HYBRID_CAR & EV_CAR == set(), "Shared cars between hybrid and EV"
    assert CANFD_CAR & HYBRID_CAR == set(), "Hard coding CAN FD cars as hybrid is no longer supported"

  def test_canfd_ecu_whitelist(self):
    # Asserts only expected Ecus can exist in database for CAN-FD cars
    for car_model in CANFD_CAR:
      ecus = {fw[0] for fw in FW_VERSIONS[car_model].keys()}
      ecus_not_in_whitelist = ecus - CANFD_EXPECTED_ECUS
      ecu_strings = ", ".join([f"Ecu.{ecu}" for ecu in ecus_not_in_whitelist])
      assert len(ecus_not_in_whitelist) == 0, \
                       f"{car_model}: Car model has unexpected ECUs: {ecu_strings}"

  def test_blacklisted_parts(self):
    # Asserts no ECUs known to be shared across platforms exist in the database.
    # Tucson having Santa Cruz camera and EPS for example
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        if car_model == CAR.HYUNDAI_SANTA_CRUZ_1ST_GEN:
          raise unittest.SkipTest("Skip checking Santa Cruz for its parts")

        for code, _ in get_platform_codes(ecus[(Ecu.fwdCamera, 0x7c4, None)]):
          if b"-" not in code:
            continue
          part = code.split(b"-")[1]
          assert not part.startswith(b'CW'), "Car has bad part number"

  def test_correct_ecu_response_database(self):
    """
    Assert standard responses for certain ECUs, since they can
    respond to multiple queries with different data
    """
    expected_fw_prefix = HYUNDAI_VERSION_REQUEST_LONG[1:]
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          assert all(fw.startswith(expected_fw_prefix) for fw in fws), \
                          f"FW from unexpected request in database: {(ecu, fws)}"

  @settings(max_examples=100)
  @given(data=st.data())
  def test_platform_codes_fuzzy_fw(self, data):
    """Ensure function doesn't raise an exception"""
    fw_strategy = st.lists(st.binary())
    fws = data.draw(fw_strategy)
    get_platform_codes(fws)

  def test_expected_platform_codes(self):
    # Ensures we don't accidentally add multiple platform codes for a car unless it is intentional
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          # Third and fourth character are usually EV/hybrid identifiers
          codes = {code.split(b"-")[0][:2] for code, _ in get_platform_codes(fws)}
          if car_model == CAR.HYUNDAI_PALISADE:
            assert codes == {b"LX", b"ON"}, f"Car has unexpected platform codes: {car_model} {codes}"
          elif car_model == CAR.HYUNDAI_KONA_EV and ecu[0] == Ecu.fwdCamera:
            assert codes == {b"OE", b"OS"}, f"Car has unexpected platform codes: {car_model} {codes}"
          else:
            assert len(codes) == 1, f"Car has multiple platform codes: {car_model} {codes}"

  # Tests for platform codes, part numbers, and FW dates which Hyundai will use to fuzzy
  # fingerprint in the absence of full FW matches:
  def test_platform_code_ecus_available(self):
    # TODO: add queries for these non-CAN FD cars to get EPS
    no_eps_platforms = CANFD_CAR | {CAR.KIA_SORENTO, CAR.KIA_OPTIMA_G4, CAR.KIA_OPTIMA_G4_FL, CAR.KIA_OPTIMA_H, CAR.KIA_K7_2017,
                                    CAR.KIA_OPTIMA_H_G4_FL, CAR.HYUNDAI_SONATA_LF, CAR.HYUNDAI_TUCSON, CAR.GENESIS_G90, CAR.GENESIS_G80, CAR.HYUNDAI_ELANTRA}

    # Asserts ECU keys essential for fuzzy fingerprinting are available on all platforms
    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for platform_code_ecu in PLATFORM_CODE_ECUS:
          if platform_code_ecu in (Ecu.fwdRadar, Ecu.eps) and car_model == CAR.HYUNDAI_GENESIS:
            continue
          if platform_code_ecu == Ecu.eps and car_model in no_eps_platforms:
            continue
          if car_model in NON_SCC_CAR:
            continue
          assert platform_code_ecu in [e[0] for e in ecus]

  def test_fw_format(self):
    # Asserts:
    # - every supported ECU FW version returns one platform code
    # - every supported ECU FW version has a part number
    # - expected parsing of ECU FW dates

    for car_model, ecus in FW_VERSIONS.items():
      with self.subTest(car_model=car_model.value):
        for ecu, fws in ecus.items():
          if ecu[0] not in PLATFORM_CODE_ECUS:
            continue

          if car_model in NON_SCC_CAR:
            continue

          codes = set()
          for fw in fws:
            result = get_platform_codes([fw])
            assert 1 == len(result), f"Unable to parse FW: {fw}"
            codes |= result

          if ecu[0] not in DATE_FW_ECUS or car_model in NO_DATES_PLATFORMS:
            assert all(date is None for _, date in codes)
          else:
            assert all(date is not None for _, date in codes)

          if car_model == CAR.HYUNDAI_GENESIS:
            raise unittest.SkipTest("No part numbers for car model")

          # Hyundai places the ECU part number in their FW versions, assert all parsable
          # Some examples of valid formats: b"56310-L0010", b"56310L0010", b"56310/M6300"
          assert all(b"-" in code for code, _ in codes), \
                          f"FW does not have part number: {fw}"

  def test_platform_codes_spot_check(self):
    # Asserts basic platform code parsing behavior for a few cases
    results = get_platform_codes([b"\xf1\x00DH LKAS 1.1 -150210"])
    assert results == {(b"DH", b"150210")}

    # Some cameras and all radars do not have dates
    results = get_platform_codes([b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         "])
    assert results == {(b"AEhe-G2000", None)}

    results = get_platform_codes([b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         "])
    assert results == {(b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00DH LKAS 1.1 -150210",
      b"\xf1\x00AEhe SCC H-CUP      1.01 1.01 96400-G2000         ",
      b"\xf1\x00CV1_ RDR -----      1.00 1.01 99110-CV000         ",
    ])
    assert results == {(b"DH", b"150210"), (b"AEhe-G2000", None), (b"CV1-CV000", None)}

    results = get_platform_codes([
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.07 99211-S8100 220222",
      b"\xf1\x00LX2 MFC  AT USA LHD 1.00 1.08 99211-S8100 211103",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.01 99211-S9100 190405",
      b"\xf1\x00ON  MFC  AT USA LHD 1.00 1.03 99211-S9100 190720",
    ])
    assert results == {(b"LX2-S8100", b"220222"), (b"LX2-S8100", b"211103"),
                               (b"ON-S9100", b"190405"), (b"ON-S9100", b"190720")}

  def test_fuzzy_excluded_platforms(self):
    # Asserts a list of platforms that will not fuzzy fingerprint with platform codes due to them being shared.
    # This list can be shrunk as we combine platforms and detect features
    excluded_platforms = {
      CAR.GENESIS_G70,            # shared platform code, part number, and date
      CAR.GENESIS_G70_2020,
    }
    excluded_platforms |= CANFD_CAR - EV_CAR - CANFD_FUZZY_WHITELIST  # shared platform codes
    excluded_platforms |= NO_DATES_PLATFORMS  # date codes are required to match

    platforms_with_shared_codes = set()
    for platform, fw_by_addr in FW_VERSIONS.items():
      car_fw = []
      for ecu, fw_versions in fw_by_addr.items():
        ecu_name, addr, sub_addr = ecu
        for fw in fw_versions:
          car_fw.append(CarParams.CarFw(ecu=ecu_name, fwVersion=fw, address=addr,
                                        subAddress=0 if sub_addr is None else sub_addr))

      if platform in NON_SCC_CAR:
        continue

      CP = CarParams(carFw=car_fw)
      matches = FW_QUERY_CONFIG.match_fw_to_car_fuzzy(build_fw_dict(CP.carFw), CP.carVin, FW_VERSIONS)
      if len(matches) == 1:
        assert list(matches)[0] == platform
      else:
        platforms_with_shared_codes.add(platform)

    assert platforms_with_shared_codes == excluded_platforms


class TestHyundaiCarParamsDynamicHandoff(unittest.TestCase):
  """Tests that CANFD_DYNAMIC_HANDOFF safety bit is set iff all conditions are met.

  Conditions:
    1. HDA II detected (CANFD_LKA_STEERING flag set)
    2. CANFD_NO_RADAR_DISABLE not set
    3. CANFD_CAMERA_SCC not set
    4. DynamicRadarHandoffEnabled param == "1"
    5. AlphaLongitudinalEnabled param == "1"
  """

  # GENESIS_GV70_ELECTRIFIED_1ST_GEN: CAN-FD EV, no CANFD_NO_RADAR_DISABLE, no CANFD_CAMERA_SCC by default.
  # Adding 0x50 to fingerprint[cam_can] triggers CANFD_LKA_STEERING (HDA II).
  CANDIDATE = CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN

  # Provide an ADAS ECU so alphaLongitudinalAvailable is True for LKA steering cars.
  ADAS_FW = [CarParams.CarFw(ecu=CarParams.Ecu.adas, fwVersion=b'test', address=0x0, subAddress=0)]

  def _build_fingerprint_with_lka(self):
    fingerprint = gen_empty_fingerprint()
    cam_can = CanBus(None, fingerprint).CAM
    fingerprint[cam_can][0x50] = 8  # triggers CANFD_LKA_STEERING
    return fingerprint

  def _build_fingerprint_no_lka(self):
    return gen_empty_fingerprint()

  def _get_params_and_apply(self, fingerprint, car_fw, alpha_long, params_list):
    """Build CP+CP_SP then apply setup_interfaces with the given params_list."""
    CP = CarInterface.get_params(self.CANDIDATE, fingerprint, car_fw, alpha_long, False, False)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, self.CANDIDATE)
    setup_interfaces(CarInterface, CP, CP_SP, params_list)
    return CP

  def _all_params(self):
    return [{"DynamicRadarHandoffEnabled": "1"}, {"AlphaLongitudinalEnabled": "1"}]

  def test_all_conditions_met_bit_set(self):
    """All five conditions satisfied -> CANFD_DYNAMIC_HANDOFF bit must be set."""
    fingerprint = self._build_fingerprint_with_lka()
    CP = self._get_params_and_apply(fingerprint, self.ADAS_FW, True, self._all_params())
    assert CP.flags & HyundaiFlags.CANFD_LKA_STEERING, "Precondition: LKA_STEERING must be set"
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF, \
      "CANFD_DYNAMIC_HANDOFF bit must be set when all conditions are met"

  def test_no_openpilot_long_bit_clear(self):
    """AlphaLongitudinalEnabled param set but openpilot longitudinal not actually active (get_params
    alpha_long=False) -> bit must be clear. Otherwise the engage edge would silence the stock ADAS DRV ECU
    while openpilot is not the longitudinal authority, leaving the car with no longitudinal safety."""
    fingerprint = self._build_fingerprint_with_lka()
    CP = self._get_params_and_apply(fingerprint, self.ADAS_FW, False, self._all_params())
    assert not CP.openpilotLongitudinalControl, "Precondition: openpilot long must be inactive"
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when openpilotLongitudinalControl is False"

  def test_dynamic_radar_handoff_param_false_bit_clear(self):
    """DynamicRadarHandoffEnabled=0 -> bit clear."""
    fingerprint = self._build_fingerprint_with_lka()
    params = [{"DynamicRadarHandoffEnabled": "0"}, {"AlphaLongitudinalEnabled": "1"}]
    CP = self._get_params_and_apply(fingerprint, self.ADAS_FW, True, params)
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when DynamicRadarHandoffEnabled is false"

  def test_alpha_long_param_false_bit_clear(self):
    """AlphaLongitudinalEnabled=0 -> bit clear."""
    fingerprint = self._build_fingerprint_with_lka()
    params = [{"DynamicRadarHandoffEnabled": "1"}, {"AlphaLongitudinalEnabled": "0"}]
    CP = self._get_params_and_apply(fingerprint, self.ADAS_FW, True, params)
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when AlphaLongitudinalEnabled is false"

  def test_no_lka_steering_bit_clear(self):
    """HDA II flag absent (no LKA steering messages) -> bit clear."""
    fingerprint = self._build_fingerprint_no_lka()
    CP = self._get_params_and_apply(fingerprint, [], False, self._all_params())
    assert not (CP.flags & HyundaiFlags.CANFD_LKA_STEERING), "Precondition: LKA_STEERING must not be set"
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when CANFD_LKA_STEERING is not set"

  def test_canfd_no_radar_disable_bit_clear(self):
    """CANFD_NO_RADAR_DISABLE present -> bit clear (car cannot disable radar ECU)."""
    fingerprint = self._build_fingerprint_with_lka()
    # HYUNDAI_KONA_EV_2ND_GEN has CANFD_NO_RADAR_DISABLE in its platform flags; adding LKA steering fingerprint
    # triggers CANFD_LKA_STEERING as well, giving us both conditions to test the exclusion.
    candidate = CAR.HYUNDAI_KONA_EV_2ND_GEN
    CP = CarInterface.get_params(candidate, fingerprint, self.ADAS_FW, True, False, False)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, candidate)
    setup_interfaces(CarInterface, CP, CP_SP, self._all_params())
    assert CP.flags & HyundaiFlags.CANFD_NO_RADAR_DISABLE, "Precondition: NO_RADAR_DISABLE must be set"
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when CANFD_NO_RADAR_DISABLE is set"

  def test_canfd_camera_scc_bit_clear(self):
    """CANFD_CAMERA_SCC present -> bit clear."""
    # Build a fingerprint without LKA steering but also not RADAR_SCC -> gets CANFD_CAMERA_SCC
    fingerprint = self._build_fingerprint_no_lka()
    # For a non-LKA-steering CANFD car without RADAR_SCC flag, CANFD_CAMERA_SCC is set.
    # KIA_EV6 doesn't have RADAR_SCC in its platform flags.
    candidate = CAR.KIA_EV6
    CP = CarInterface.get_params(candidate, fingerprint, [], False, False, False)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, candidate)
    setup_interfaces(CarInterface, CP, CP_SP, self._all_params())
    assert CP.flags & HyundaiFlags.CANFD_CAMERA_SCC, "Precondition: CANFD_CAMERA_SCC must be set"
    assert not (CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF), \
      "CANFD_DYNAMIC_HANDOFF bit must be clear when CANFD_CAMERA_SCC is set"


class _FakeCarStateForHandoff:
  """Minimal CS stand-in for unit-testing the dynamic-handoff watchdog without spinning up CAN parsers."""
  def __init__(self):
    self.adas_drv_uds_response_count = 0
    self.adas_drv_uds_response_isotp_len = 0
    self.adas_drv_uds_response_byte1 = 0
    self.adas_drv_uds_response_byte2 = 0
    self.adas_drv_uds_response_byte3 = 0

  def post_response(self, byte1: int, byte2: int = 0, byte3: int = 0):
    self.adas_drv_uds_response_count += 1
    self.adas_drv_uds_response_byte1 = byte1
    self.adas_drv_uds_response_byte2 = byte2
    self.adas_drv_uds_response_byte3 = byte3


class TestHyundaiHandoffWatchdog(unittest.TestCase):
  """Unit tests for the dynamic-handoff response watchdog in CarController."""

  CANDIDATE = CAR.GENESIS_GV70_ELECTRIFIED_1ST_GEN

  def _build_controller(self):
    from opendbc.car.hyundai.carcontroller import CarController
    fingerprint = gen_empty_fingerprint()
    fingerprint[CanBus(None, fingerprint).CAM][0x50] = 8  # CANFD_LKA_STEERING
    adas_fw = [CarParams.CarFw(ecu=CarParams.Ecu.adas, fwVersion=b'test', address=0x0, subAddress=0)]
    CP = CarInterface.get_params(self.CANDIDATE, fingerprint, adas_fw, True, False, False)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, self.CANDIDATE)
    setup_interfaces(CarInterface, CP, CP_SP, [{"DynamicRadarHandoffEnabled": "1"}, {"AlphaLongitudinalEnabled": "1"}])
    assert CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF
    return CarController({"pt": "hyundai_canfd_generated", "cam": "hyundai_canfd_generated"}, CP, CP_SP)

  @staticmethod
  def _step(expected, nrc_service, retries=0):
    return {'msg': None, 'expected': expected, 'nrc_service': nrc_service, 'sent_frame': None,
            'deadline': None, 'retries_left': retries}

  def _engage_edge(self, cc):
    """Set up the engage-edge sequential watchdog steps the carcontroller would build on a real edge."""
    cc._handoff_seq = [self._step(0x50, 0x10), self._step(0x68, 0x28)]  # extendedSession, then disableRxAndTx
    cc._handoff_seq_kind = 1

  def _disengage_edge(self, cc):
    cc._handoff_seq = [self._step(0x68, 0x28), self._step(0x50, 0x10)]  # enableRxAndTx, then defaultSession
    cc._handoff_seq_kind = 2

  @staticmethod
  def _tick(cc, cs):
    cc._tick_handoff_watchdog(cs, [])

  def test_engage_positive_acks_clear_pending_no_fault(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._engage_edge(cc)
    self._tick(cc, cs)                                        # sends step 0 (extendedSession)
    cc.frame += 1
    cs.post_response(0x50, 0x03)                              # ack step 0 → advance
    self._tick(cc, cs)
    cc.frame += 1
    self._tick(cc, cs)                                        # sends step 1 (disableRxAndTx)
    cc.frame += 1
    cs.post_response(0x68, 0x03, 0x01)                        # ack step 1 → done
    self._tick(cc, cs)
    cc.frame += 1
    self.assertEqual(cc._handoff_seq, [])
    self.assertEqual(cc.handoff_fault, 0)

  def test_requests_sent_one_at_a_time(self):
    # Core anti-coalescing property: only one UDS request is ever outstanding, so the latest-frame-only
    # 0x738 parser cannot drop an ack/NRC.
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._engage_edge(cc)
    s1 = []
    cc._tick_handoff_watchdog(cs, s1)                         # sends step 0 only
    self.assertEqual(len(s1), 1)
    cs.post_response(0x50, 0x03)
    s2 = []
    cc._tick_handoff_watchdog(cs, s2)                         # acks step 0; does NOT send step 1 same tick
    self.assertEqual(len(s2), 0)
    s3 = []
    cc._tick_handoff_watchdog(cs, s3)                         # now sends step 1
    self.assertEqual(len(s3), 1)

  def test_engage_nrc_on_session_control_sets_engage_fault(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._engage_edge(cc)
    self._tick(cc, cs)                                         # send step 0
    # 0x7F 0x10 <code>: ECU rejected the SessionControl request.
    cs.post_response(0x7F, 0x10, 0x22)
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 1)

  def test_engage_timeout_sets_engage_fault(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._engage_edge(cc)
    self._tick(cc, cs)                                         # send step 0, start deadline
    # No response, advance past the deadline.
    cc.frame += cc.HANDOFF_RESPONSE_DEADLINE_FRAMES + 1
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 1)

  def test_disengage_nrc_sets_disengage_fault(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._disengage_edge(cc)
    self._tick(cc, cs)                                         # send step 0
    cs.post_response(0x7F, 0x28, 0x22)
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 2)

  def test_fault_latches_then_clears(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._disengage_edge(cc)
    self._tick(cc, cs)                                         # send step 0
    cs.post_response(0x7F, 0x28, 0x22)
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 2)
    # Fault must persist for HANDOFF_FAULT_LATCH_FRAMES so selfdrived has time to observe it.
    cc.frame += cc.HANDOFF_FAULT_LATCH_FRAMES - 1
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 2)
    cc.frame += 2
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 0)

  def test_engage_fault_outranks_disengage_fault(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    self._disengage_edge(cc)
    self._tick(cc, cs)                                         # send disengage step 0
    cs.post_response(0x7F, 0x28, 0x22)
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 2)
    # Now an engage edge supersedes and faults too — fault must escalate to 1 (engage), not stay at 2.
    self._engage_edge(cc)
    self._tick(cc, cs)                                         # send engage step 0
    cs.post_response(0x7F, 0x10, 0x22)
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 1)

  def test_make_handoff_step_sets_retries(self):
    # Steps the carcontroller builds on a real edge must carry a positive retry budget so a single dropped
    # request (notably panda rejecting the silencing frame until controls_allowed settles) does not fault.
    cc = self._build_controller()
    self.assertGreater(cc.HANDOFF_STEP_MAX_RETRIES, 0)
    step = cc._make_handoff_step(None, 0x50, 0x10)
    self.assertEqual(step['retries_left'], cc.HANDOFF_STEP_MAX_RETRIES)

  def test_timeout_retries_then_succeeds(self):
    # A timed-out step must be re-sent (re-armed) rather than immediately latching a fault. This covers the
    # engage silencing frame being dropped by panda until controls_allowed has settled.
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    cc._handoff_seq = [self._step(0x50, 0x10, retries=2)]
    cc._handoff_seq_kind = 1
    self._tick(cc, cs)                                         # send attempt 1
    cc.frame += cc.HANDOFF_RESPONSE_DEADLINE_FRAMES + 1
    self._tick(cc, cs)                                         # attempt 1 times out → retry, no fault
    self.assertEqual(cc.handoff_fault, 0)
    self.assertEqual(len(cc._handoff_seq), 1)                  # step still pending
    self._tick(cc, cs)                                         # re-send attempt 2
    cs.post_response(0x50, 0x03)                               # ECU acks this time
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 0)
    self.assertEqual(cc._handoff_seq, [])                      # advanced past the step

  def test_timeout_exhausts_retries_then_faults(self):
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    cc._handoff_seq = [self._step(0x50, 0x10, retries=2)]      # initial attempt + 2 retries = 3 timeouts to fault
    cc._handoff_seq_kind = 1
    for _ in range(3):
      self.assertEqual(cc.handoff_fault, 0)
      self._tick(cc, cs)                                       # (re)send
      cc.frame += cc.HANDOFF_RESPONSE_DEADLINE_FRAMES + 1
      self._tick(cc, cs)                                       # time out
    self.assertEqual(cc.handoff_fault, 1)

  def test_nrc_does_not_retry(self):
    # An NRC is an explicit ECU rejection; retrying won't help, so the fault latches immediately even with
    # retries still available.
    cc = self._build_controller()
    cs = _FakeCarStateForHandoff()
    cc._handoff_seq = [self._step(0x50, 0x10, retries=3)]
    cc._handoff_seq_kind = 1
    self._tick(cc, cs)                                         # send
    cs.post_response(0x7F, 0x10, 0x22)                         # NRC for SessionControl
    self._tick(cc, cs)
    self.assertEqual(cc.handoff_fault, 1)
    self.assertEqual(cc._handoff_seq, [])

  def test_carstate_declares_cc_backref(self):
    # CarStateBase must declare CC (default None) so the back-ref is an explicit, type-safe attribute and a
    # missing wiring reads as None (fail-soft) rather than raising AttributeError.
    from opendbc.car.hyundai.carstate import CarState
    fingerprint = gen_empty_fingerprint()
    fingerprint[CanBus(None, fingerprint).CAM][0x50] = 8
    adas_fw = [CarParams.CarFw(ecu=CarParams.Ecu.adas, fwVersion=b'test', address=0x0, subAddress=0)]
    CP = CarInterface.get_params(self.CANDIDATE, fingerprint, adas_fw, True, False, False)
    CP_SP = CarInterface.get_non_essential_params_sp(CP, self.CANDIDATE)
    cs = CarState(CP, CP_SP)
    self.assertIsNone(cs.CC)
