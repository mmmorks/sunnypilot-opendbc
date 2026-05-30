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

# On some HKG CAN and CAN FD non-CANFD_ALT_BUTTONS, the cancel button (CF_Clu_CruiseSwState / CRUISE_BUTTONS = 4) is
# a pause/resume toggle, not a dedicated cancel. Firing it mid-brake inadvertently can cause a re-enable attempt
# and triggers the "SCC Conditions Not Met" alert. Delaying the button send lets factory SCC disengage
# naturally on brake press. We send ~100 ms later if it fails to do so, or if we want to cancel for another reason.
CANCEL_BUTTON_DELAY_FRAMES = 10


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
    self.cancel_counter = 0

    self.dynamic_radar_handoff_enabled = bool(CP.safetyConfigs[-1].safetyParam & HyundaiSafetyFlags.CANFD_DYNAMIC_HANDOFF)
    self.prev_enabled = False

    # Dynamic radar handoff response watchdog.
    # On an engage/disengage edge we build a SEQUENTIAL list of UDS steps. Only one request is ever outstanding
    # at a time: the watchdog sends step N, waits for its specific ack on 0x738, then sends step N+1. This is
    # required because the 0x738 parser (carstate) only retains the LATEST frame per 100Hz cycle — firing both
    # requests in one frame allows the two responses to coalesce into one cycle, silently dropping an ack (false
    # timeout) or an NRC (false success). Serializing guarantees at most one response per cycle.
    # Each step is a dict: {'msg', 'expected' (positive ack byte1), 'nrc_service' (orig service id), 'sent_frame', 'deadline'}.
    # A timed-out or NRC'd step sets self.handoff_fault, which the CarState bridge surfaces to
    # CarStateSP.adasDrvHandoffFault and selfdrived turns into an EventName event.
    # 0=none, 1=engageFailed (IMMEDIATE_DISABLE), 2=disengageFailed (warning only).
    self.handoff_fault: int = 0
    self.handoff_fault_clear_frame: int = 0
    self._handoff_seq: list[dict] = []      # remaining sequential UDS steps for the active edge
    self._handoff_seq_kind: int = 0         # fault type to latch if the active sequence fails (1=engage, 2=disengage)
    self._handoff_last_response_seen_count: int = 0
    # Window in which we accept each step's ack. 50 frames @ 100Hz = 500ms; UDS S6/S7 typically <50ms.
    self.HANDOFF_RESPONSE_DEADLINE_FRAMES: int = 50
    # Re-send a step this many times on timeout before latching a fault. Absorbs transient drops — notably
    # panda rejecting the engage silencing frame (disableRxAndTx) until controls_allowed has settled, and
    # one-off CAN losses — without escalating straight to IMMEDIATE_DISABLE. NRCs are NOT retried.
    self.HANDOFF_STEP_MAX_RETRIES: int = 3
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

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    # dynamic_radar_handoff_enabled is derived from the CANFD_DYNAMIC_HANDOFF safety bit, which
    # _initialize_dynamic_radar_handoff only sets when every precondition holds (HDA II, radar-disable
    # capable, not camera-SCC, and openpilotLongitudinalControl). So the bit alone is authoritative here —
    # the engage and disengage edges gate on the same predicate, symmetrically.
    disengage_edge = self.prev_enabled and not CC.enabled and self.dynamic_radar_handoff_enabled
    engage_edge = not self.prev_enabled and CC.enabled and self.dynamic_radar_handoff_enabled

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not ((self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) or self.ESCC.enabled) and \
            self.CP.openpilotLongitudinalControl and \
            (not self.dynamic_radar_handoff_enabled or CC.enabled):
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # Delay the cancel button send so the brake can disengage factory SCC first.
    # Reset whenever openpilot is no longer requesting cancel.
    self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

    # dynamic handoff: on engage->disengage edge, re-enable stock SCC/AEB by restoring ADAS DRV ECU communication.
    # 0x28 0x00 re-enables Rx/Tx in the current (extended) session; 0x10 0x01 drops back to defaultSession,
    # which on Hyundai also resets any residual CommunicationControl state. We send them sequentially (comm
    # control first, then session) so each response is observed on its own cycle. Responses on 0x738 are NOT
    # suppressed so positive acks ("68 00", "50 01") and NRCs ("7F 28 <code>", "7F 10 <code>") land in logs.
    if disengage_edge:
      self._handoff_seq = [
        self._make_handoff_step(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x00, suppress_response=False),
                                expected=0x68, nrc_service=0x28),
        self._make_handoff_step(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x01, suppress_response=False),
                                expected=0x50, nrc_service=0x10),
      ]
      self._handoff_seq_kind = 2

    # dynamic handoff: on disengage->engage edge, re-silence the ADAS DRV ECU. Boot disable was skipped under
    # dynamic handoff so the ECU is in default session here; the 1Hz tester-present that resumes this frame
    # keeps the extended session alive (ECU S3 timer ~5s). extendedSession is established first, then
    # disableRxAndTx (which panda only accepts while controls_allowed) — sequencing also allows controls_allowed to
    # settle before the silencing frame is sent. A new edge supersedes any in-flight sequence.
    if engage_edge:
      self._handoff_seq = [
        self._make_handoff_step(make_diagnostic_session_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False),
                                expected=0x50, nrc_service=0x10),
        self._make_handoff_step(make_communication_control_msg(0x730, self.CAN.ECAN, sub_function=0x03, suppress_response=False),
                                expected=0x68, nrc_service=0x28),
      ]
      self._handoff_seq_kind = 1

    # Watchdog tick: send the next pending UDS step, consume its response, latch faults on NRC/timeout.
    if self.dynamic_radar_handoff_enabled:
      self._tick_handoff_watchdog(CS, can_sends)

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      # Hold torque with induced temporary fault when cutting the actuation bit
      # FIXME: we don't use this with CAN FD?
      torque_fault = CC.latActive and not apply_steer_req

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

  def _make_handoff_step(self, msg, expected, nrc_service):
    # One sequential UDS step for the handoff watchdog. retries_left allows a timed-out step to be re-sent before
    # it escalates to a latched fault (see HANDOFF_STEP_MAX_RETRIES).
    return {'msg': msg, 'expected': expected, 'nrc_service': nrc_service,
            'sent_frame': None, 'deadline': None, 'retries_left': self.HANDOFF_STEP_MAX_RETRIES}

  def _tick_handoff_watchdog(self, CS, can_sends):
    """Advance the dynamic-handoff response watchdog. Called every tick when dynamic handoff is enabled.

    Drives the active UDS sequence one step at a time (see __init__). Per tick:
      - Consume any newly-arrived 0x738 response (CS.adas_drv_uds_response_count delta). Because requests are
        serialized, at most one response is outstanding, so the latest-frame-only parser cannot drop it.
      - If the current step has not been sent yet, append it to can_sends and start its deadline.
      - Else, if its positive ack arrived → advance to the next step. If an NRC for its service arrived, or its
        deadline expired → latch the sequence's fault type and abort the sequence.
    Fault latches for HANDOFF_FAULT_LATCH_FRAMES so selfdrived has time to observe it.
    """
    # Consume newly-arrived response (if any). 0x738 frames only appear as responses to our (non-suppressed)
    # handoff requests, so a fresh delta here is unambiguously the ack/NRC for the outstanding step.
    response_byte1 = response_byte2 = None
    response_is_nrc = False
    if CS.adas_drv_uds_response_count != self._handoff_last_response_seen_count:
      self._handoff_last_response_seen_count = CS.adas_drv_uds_response_count
      response_byte1 = CS.adas_drv_uds_response_byte1
      response_byte2 = CS.adas_drv_uds_response_byte2
      response_is_nrc = response_byte1 == 0x7F

    if self._handoff_seq:
      step = self._handoff_seq[0]
      if step['sent_frame'] is None:
        # Send this step now and start its response window. Don't evaluate a response on the same tick.
        can_sends.append(step['msg'])
        step['sent_frame'] = self.frame
        step['deadline'] = self.frame + self.HANDOFF_RESPONSE_DEADLINE_FRAMES
      else:
        matched_positive = response_byte1 == step['expected']
        matched_nrc = response_is_nrc and response_byte2 == step['nrc_service']
        if matched_positive:
          self._handoff_seq.pop(0)  # success → next step sends on the following tick
        elif matched_nrc:
          # Explicit ECU rejection — retrying won't help, latch immediately.
          self._handoff_seq = []
          self._latch_handoff_fault(self._handoff_seq_kind)
        elif self.frame > step['deadline']:
          if step['retries_left'] > 0:
            # Re-arm the step: re-send and restart its response window on the next tick.
            step['retries_left'] -= 1
            step['sent_frame'] = None
            step['deadline'] = None
          else:
            self._handoff_seq = []
            self._latch_handoff_fault(self._handoff_seq_kind)

    if self.handoff_fault and self.frame >= self.handoff_fault_clear_frame:
      self.handoff_fault = 0

  def _latch_handoff_fault(self, kind: int) -> None:
    # Engage fault (1) outranks disengage fault (2); never downgrade an already-latched engage fault.
    if kind == 1 and self.handoff_fault != 1:
      self.handoff_fault = 1
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES
    elif kind == 2 and self.handoff_fault not in (1, 2):
      self.handoff_fault = 2
      self.handoff_fault_clear_frame = self.frame + self.HANDOFF_FAULT_LATCH_FRAMES

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
      if self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
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

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque, self.lkas_icon))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))

    # LFA and HDA icons
    if self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled, self.lfa_icon))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
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
          # Here we send ACC message to cancel, not buttons. Don't delay
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          elif self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
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
