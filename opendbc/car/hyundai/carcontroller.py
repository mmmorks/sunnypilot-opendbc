import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_communication_control_msg, make_diagnostic_session_control_msg, \
                        make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, common_fault_avoidance
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, HyundaiSafetyFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

from opendbc.sunnypilot.car.hyundai.escc import EsccCarController
from opendbc.sunnypilot.car.hyundai.icbm import IntelligentCruiseButtonManagementInterface
from opendbc.sunnypilot.car.hyundai.longitudinal.controller import LongitudinalController
from opendbc.sunnypilot.car.hyundai.lead_data_ext import LeadDataCarController
from opendbc.sunnypilot.car.hyundai.mads import MadsCarController

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController(CarControllerBase, EsccCarController, LeadDataCarController, LongitudinalController, MadsCarController,
                    IntelligentCruiseButtonManagementInterface):
  def __init__(self, dbc_names, CP, CP_SP):
    CarControllerBase.__init__(self, dbc_names, CP, CP_SP)
    EsccCarController.__init__(self, CP, CP_SP)
    MadsCarController.__init__(self)
    LeadDataCarController.__init__(self, CP)
    LongitudinalController.__init__(self, CP, CP_SP)
    IntelligentCruiseButtonManagementInterface.__init__(self, CP, CP_SP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0

    self.dynamic_radar_handoff_enabled = bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    self.prev_enabled = False

    # Dynamic radar handoff response watchdog. Each entry is (deadline_frame, expected_positive_byte1, label).
    # On edge fire the watchdog queues the expected ack(s); each tick we check CS.adas_drv_uds_response_* for
    # a positive ack (UDS_BYTE_1 == request_id + 0x40) or NRC (UDS_BYTE_1 == 0x7F with UDS_BYTE_2 == request_id).
    # A timed-out, missing, or NRC'd entry sets self.handoff_fault, which the CarState bridge surfaces to
    # CarStateSP.adasDrvHandoffFault and selfdrived turns into an EventName event.
    # 0=none, 1=engageFailed (IMMEDIATE_DISABLE), 2=disengageFailed (warning only).
    self.handoff_fault: int = 0
    self.handoff_fault_clear_frame: int = 0
    self.handoff_pending_engage: list[tuple[int, int]] = []     # (deadline_frame, expected_byte1)
    self.handoff_pending_disengage: list[tuple[int, int]] = []
    self._handoff_last_response_seen_count: int = 0
    # Window after edge in which we accept the ECU's ack. 50 frames @ 100Hz = 500ms; UDS S6/S7 typically <50ms.
    self.HANDOFF_RESPONSE_DEADLINE_FRAMES: int = 50
    # Latch fault for 5s (500 frames) so selfdrived has time to observe and post the event.
    self.HANDOFF_FAULT_LATCH_FRAMES: int = 500

  def update(self, CC, CC_SP, CS, now_nanos):
    EsccCarController.update(self, CS)
    LeadDataCarController.update(self, CC_SP)
    MadsCarController.update(self, self.CP, CC, CC_SP, self.frame)
    if self.frame % 5 == 0:
      LongitudinalController.update(self, CC, CS)

    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    # Hold torque with induced temporary fault when cutting the actuation bit
    # FIXME: we don't use this with CAN FD?
    torque_fault = CC.latActive and not apply_steer_req

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    disengage_edge = (self.prev_enabled and not CC.enabled
                      and self.dynamic_radar_handoff_enabled
                      and self.CP.openpilotLongitudinalControl
                      and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC)
                      and bool(self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING))
    engage_edge = (not self.prev_enabled and CC.enabled and self.dynamic_radar_handoff_enabled)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl and \
            (not self.dynamic_radar_handoff_enabled or CC.enabled):
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # dynamic handoff: on engage->disengage edge, re-enable stock SCC/AEB by restoring ADAS DRV ECU communication.
    # 0x28 0x00 re-enables Rx/Tx in the current (extended) session; 0x10 0x01 drops back to defaultSession,
    # which on Hyundai also resets any residual CommunicationControl state. Either frame alone restores comm;
    # sending both gives a redundant fast-recovery path (no 5s S3 wait). Responses on 0x738 are NOT suppressed
    # so positive acks ("50 01", "68 00") and NRCs ("7F 10 <code>", "7F 28 <code>") land in route/cabana logs.
    if disengage_edge:
      can_sends.append(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x00, suppress_response=False))
      can_sends.append(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x01, suppress_response=False))
      # Watchdog: expect positive responses 0x68 (CommunicationControl ack) and 0x50 (SessionControl ack).
      deadline = self.frame + self.HANDOFF_RESPONSE_DEADLINE_FRAMES
      self.handoff_pending_disengage = [(deadline, 0x68), (deadline, 0x50)]

    # dynamic handoff: on disengage->engage edge, re-silence the ADAS DRV ECU. Boot disable was skipped under
    # dynamic handoff so the ECU is in default session here; the 1Hz tester-present that resumes this frame
    # keeps the extended session alive (ECU S3 timer ~5s). Responses on 0x738 not suppressed (same rationale).
    if engage_edge:
      can_sends.append(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False))
      can_sends.append(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False))
      deadline = self.frame + self.HANDOFF_RESPONSE_DEADLINE_FRAMES
      self.handoff_pending_engage = [(deadline, 0x50), (deadline, 0x68)]
      # Reset the response-seen baseline so we start counting fresh responses for THIS edge.
      self._handoff_last_response_seen_count = CS.adas_drv_uds_response_count if self.dynamic_radar_handoff_enabled else 0

    # Watchdog tick: consume any new ADAS DRV UDS response, advance pending queues, latch faults.
    if self.dynamic_radar_handoff_enabled:
      self._tick_handoff_watchdog(CS)

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    # Intelligent Cruise Button Management
    can_sends.extend(IntelligentCruiseButtonManagementInterface.update(self, CS, CC_SP, self.packer, self.frame, self.last_button_frame, self.CAN))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.accel = self.tuning.actual_accel

    self.prev_enabled = CC.enabled
    self.frame += 1
    return new_actuators, can_sends

  def _tick_handoff_watchdog(self, CS):
    """Advance the dynamic-handoff response watchdog. Called every tick when dynamic handoff is enabled.

    Reads the latest ADAS DRV UDS response from CS (parsed from the 0x738 message). Each pending edge has a
    queue of (deadline_frame, expected_positive_byte1) tuples. We:
      - Consume any new response (CS.adas_drv_uds_response_count delta).
      - If a positive ack matches an entry's expected byte → remove that entry (success).
      - If an NRC (UDS_BYTE_1 == 0x7F) for the corresponding service is observed → set fault, clear queue.
      - If the deadline expires with the entry still pending → set fault, clear queue.
    Fault latches for HANDOFF_FAULT_LATCH_FRAMES so selfdrived has time to observe it.
    """
    # Consume newly-arrived response (if any).
    response_byte1 = response_byte2 = None
    response_is_nrc = False
    if CS.adas_drv_uds_response_count != self._handoff_last_response_seen_count:
      self._handoff_last_response_seen_count = CS.adas_drv_uds_response_count
      response_byte1 = CS.adas_drv_uds_response_byte1
      response_byte2 = CS.adas_drv_uds_response_byte2
      response_is_nrc = response_byte1 == 0x7F

    def advance(queue: list[tuple[int, int]]) -> tuple[list[tuple[int, int]], bool]:
      """Returns (new_queue, fault_observed_this_tick)."""
      if not queue:
        return queue, False
      fault = False
      new_queue: list[tuple[int, int]] = []
      for deadline, expected_b1 in queue:
        # 0x50 ack of 0x10 request → expected_b1==0x50; NRC byte2==0x10. 0x68 ack of 0x28 → byte2==0x28.
        nrc_service_for_expected = 0x10 if expected_b1 == 0x50 else 0x28
        matched_positive = response_byte1 == expected_b1
        matched_nrc = response_is_nrc and response_byte2 == nrc_service_for_expected
        if matched_positive:
          continue  # success, drop entry
        if matched_nrc:
          fault = True
          continue  # drop entry, fault recorded
        if self.frame > deadline:
          fault = True
          continue  # timeout, drop entry
        new_queue.append((deadline, expected_b1))
      return new_queue, fault

    self.handoff_pending_engage, engage_fault = advance(self.handoff_pending_engage)
    self.handoff_pending_disengage, disengage_fault = advance(self.handoff_pending_disengage)

    if engage_fault and self.handoff_fault != 1:
      self.handoff_fault = 1
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES
    elif disengage_fault and self.handoff_fault not in (1, 2):
      # Engage fault takes priority — if engage already faulted, don't downgrade.
      self.handoff_fault = 2
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES

    if self.handoff_fault and self.frame >= self.handoff_fault_clear_frame:
      self.handoff_fault = 0

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning,
                                              self.lkas_icon))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if CC.cruiseControl.cancel:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      self.lead_data, hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP,
                                                      CS.main_cruise_enabled, self.tuning, self.ESCC))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled, self.lfa_icon))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP, self.ESCC))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl and not self.ESCC.enabled:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, self.lkas_icon))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEERING_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if not (self.dynamic_radar_handoff_enabled and not CC.enabled):
        if lka_steering:
          can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
        else:
          can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
        if self.frame % 2 == 0:
          can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                           set_speed_in_units, hud_control, self.lead_data, CS.main_cruise_enabled, self.tuning))
          self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

    return can_sends
