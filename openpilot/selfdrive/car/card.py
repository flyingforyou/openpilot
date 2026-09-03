#!/usr/bin/env python3
import os
import time
import threading

import openpilot.cereal.messaging as messaging

from openpilot.cereal import log
from opendbc.car.structs import car

from openpilot.common.params import Params
from openpilot.common.realtime import config_realtime_process, Priority, Ratekeeper
from openpilot.common.swaglog import cloudlog, ForwardingHandler

from opendbc.car import DT_CTRL, structs
from opendbc.car.can_definitions import CanData, CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.fw_versions import ObdCallback
from opendbc.car.car_helpers import get_car, interfaces
from opendbc.car.interfaces import CarInterfaceBase, RadarInterfaceBase
from opendbc.car.tesla.values import TeslaFlags, TeslaSafetyFlags
from openpilot.selfdrive.pandad import can_capnp_to_list, can_list_to_can_capnp
from openpilot.selfdrive.car.cruise import VCruiseHelper

REPLAY = "REPLAY" in os.environ

EventName = log.OnroadEvent.EventName

# forward
carlog.addHandler(ForwardingHandler(cloudlog))


def obd_callback(params: Params) -> ObdCallback:
  def set_obd_multiplexing(obd_multiplexing: bool):
    if params.get_bool("ObdMultiplexingEnabled") != obd_multiplexing:
      cloudlog.warning(f"Setting OBD multiplexing to {obd_multiplexing}")
      params.remove("ObdMultiplexingChanged")
      params.put_bool("ObdMultiplexingEnabled", obd_multiplexing, block=True)
      params.get_bool("ObdMultiplexingChanged", block=True)
      cloudlog.warning("OBD multiplexing set successfully")
  return set_obd_multiplexing


def can_comm_callbacks(logcan: messaging.SubSocket, sendcan: messaging.PubSocket) -> tuple[CanRecvCallable, CanSendCallable]:
  def can_recv(wait_for_one: bool = False) -> list[list[CanData]]:
    """
    wait_for_one: wait the normal logcan socket timeout for a CAN packet, may return empty list if nothing comes

    Returns: CAN packets comprised of CanData objects for easy access
    """
    ret = []
    for can in messaging.drain_sock(logcan, wait_for_one=wait_for_one):
      ret.append([CanData(msg.address, msg.dat, msg.src) for msg in can.can])
    return ret

  def can_send(msgs: list[CanData]) -> None:
    sendcan.send(can_list_to_can_capnp(msgs, msgtype='sendcan'))

  return can_recv, can_send


class Car:
  CI: CarInterfaceBase
  RI: RadarInterfaceBase
  CP: car.CarParams

  def __init__(self, CI=None, RI=None) -> None:
    self.can_sock = messaging.sub_sock('can', timeout=20)
    self.sm = messaging.SubMaster(['pandaStates', 'carControl', 'onroadEvents', 'radarState'])
    self.pm = messaging.PubMaster(['sendcan', 'carState', 'carParams', 'carOutput', 'radarTracks'])

    self.can_rcv_cum_timeout_counter = 0

    self.CC_prev = car.CarControl.new_message()
    self.CS_prev = car.CarState.new_message()
    self.initialized_prev = False

    self.last_actuators_output = structs.CarControl.Actuators()

    self.params = Params()

    self.can_callbacks = can_comm_callbacks(self.can_sock, self.pm.sock['sendcan'])

    is_release = self.params.get_bool("IsReleaseBranch")

    if CI is None:
      # wait for one pandaState and one CAN packet
      print("Waiting for CAN messages...")
      while True:
        can = messaging.recv_one_retry(self.can_sock)
        if len(can.can) > 0:
          break

      alpha_long_allowed = self.params.get_bool("AlphaLongitudinalEnabled")
      num_pandas = len(messaging.recv_one_retry(self.sm.sock['pandaStates']).pandaStates)

      cached_params = None
      cached_params_raw = self.params.get("CarParamsCache")
      if cached_params_raw is not None:
        with car.CarParams.from_bytes(cached_params_raw) as _cached_params:
          cached_params = _cached_params

      self.CI = get_car(*self.can_callbacks, obd_callback(self.params), alpha_long_allowed, is_release, cached_params, num_pandas=num_pandas)
      self.RI = interfaces[self.CI.CP.carFingerprint].RadarInterface(self.CI.CP)
      self.CP = self.CI.CP

      # continue onto next fingerprinting step in pandad
      self.params.put_bool("FirmwareQueryDone", True, block=True)
    else:
      self.CI, self.CP = CI, CI.CP
      self.RI = RI

    self.CP.alternativeExperience = 0
    openpilot_enabled_toggle = self.params.get_bool("OpenpilotEnabledToggle")
    log_only = self.params.get_bool("TeslaLogOnly")
    controller_available = self.CI.CC is not None and openpilot_enabled_toggle and not self.CP.dashcamOnly and not log_only
    self.CP.passive = not controller_available or self.CP.dashcamOnly
    if self.CP.passive:
      safety_config = structs.CarParams.SafetyConfig()
      safety_config.safetyModel = structs.CarParams.SafetyModel.noOutput
      self.CP.safetyConfigs = [safety_config]

    # Stock longitudinal: hand speed control back to the car's own ACC and do lateral only.
    # Clearing openpilotLongitudinalControl stops controlsd/the planner actuating long and makes
    # the car controller go silent on DAS_control; dropping the panda LONG_CONTROL flag is what
    # then makes the factory DAS_control forward through instead of being blocked, and drops the
    # message from the TX allowlist so openpilot cannot transmit it at all. Both halves are
    # required -- with only the first, the stock frames stayed blocked and the car lost TACC and
    # Autopilot together. Set at init, so a restart is required to change it.
    if self.CP.brand == "tesla" and not self.CP.passive and self.params.get_bool("TeslaStockLong"):
      self.CP.openpilotLongitudinalControl = False
      for cfg in self.CP.safetyConfigs:
        cfg.safetyParam &= ~TeslaSafetyFlags.LONG_CONTROL.value

    # Cooperative steering: openpilot shifts its angle target toward the driver instead of
    # letting go of the wheel, so a takeover never has to push hard enough for the EPS to
    # inhibit. CarState reads CP.flags every frame off this same object.
    if self.CP.brand == "tesla" and self.params.get_bool("TeslaCoopSteer"):
      self.CP.flags |= TeslaFlags.COOP_STEER.value
      # the two numbers are read here rather than in the car port, which has no access to params
      if self.CI.CC is not None and hasattr(self.CI.CC, "coop_steer"):
        self.CI.CC.coop_steering = True
        self.CI.CC.coop_steer.set_tuning(
          int(self.params.get("TeslaCoopMaxTorqueCNm", return_default=True) or 250) / 100.0,
          int(self.params.get("TeslaCoopLatAccelCms", return_default=True) or 150) / 100.0)

    # Braking jerk authority floor, m/s^3 (param is in 0.1 m/s^3 steps). 0 leaves DAS_jerkMin at
    # the fault limit, which is what makes the car grab harder than the command asks for -- see
    # CarControllerParams.JERK_BRAKE_GAIN. Read here because the car port has no access to params.
    if self.CP.brand == "tesla" and self.CI.CC is not None and hasattr(self.CI.CC, "brake_jerk_base"):
      self.CI.CC.brake_jerk_base = (self.params.get("TeslaBrakeJerk", return_default=True) or 0) / 10.0

    # Cluster MAX speed sync. Writes the cruise stalk, so it stays opt-in and panda restricts the
    # lever field to the four speed steps -- the same field selects gear on this car.
    if self.CP.brand == "tesla" and not self.CP.passive and self.params.get_bool("TeslaSyncClusterSpeed"):
      self.CP.flags |= TeslaFlags.SYNC_CLUSTER_SPEED.value
      for cfg in self.CP.safetyConfigs:
        if cfg.safetyModel == structs.CarParams.SafetyModel.teslaLegacy:
          cfg.safetyParam |= TeslaSafetyFlags.SYNC_CLUSTER_SPEED.value

    # Tesla Unity-style AP1 instrument-cluster integration (HW1 only). Only wire it up when the
    # feature is actually enabled -- it was UNCONDITIONAL, so the panda blocked the factory's own
    # 0x239/0x399 (AutopilotStatus, which carries blindspot/FCW and a counter the DI validates)
    # the moment openpilot engaged, and our re-send did not reproduce that stream cleanly. Measured:
    # 0x399 vanished off the party bus through the engaged window and the DI faulted its own TACC
    # 0.4 s later, on every drive, even with the switch off (passthrough). Gating on the param means
    # a disabled feature blocks nothing and the factory frames flow untouched. Restart to toggle.
    self._ic_frame = 0
    self._ic_enabled = self.params.get_bool("TeslaICIntegration")
    if self.CP.brand == "tesla" and not self.CP.passive and self._ic_enabled:
      for cfg in self.CP.safetyConfigs:
        if (cfg.safetyModel == structs.CarParams.SafetyModel.teslaLegacy and
            cfg.safetyParam & TeslaSafetyFlags.FLAG_HW1.value):
          cfg.safetyParam |= TeslaSafetyFlags.IC_INTEGRATION.value
          self.CP.flags |= TeslaFlags.IC_INTEGRATION.value

    # No panda-side change needed: tesla_legacy.h already blocks the factory's own
    # DAS_steeringControl (0x488) from reaching EPAS unconditionally (see the fwd_hook comment
    # there), so a double-stroke can never actually hand steering to the factory regardless of
    # this flag. All this does is stop carstate's invalidLkasSetting -- driven by the factory's
    # own DAS_autosteerEnabled -- from refusing openpilot's engagement (NO_ENTRY) just because
    # the factory also thinks it's autosteering.
    if (self.CP.brand == "tesla" and not self.CP.passive and
        self.params.get_bool("TeslaDoubleStrokeOverride")):
      for cfg in self.CP.safetyConfigs:
        if (cfg.safetyModel == structs.CarParams.SafetyModel.teslaLegacy and
            cfg.safetyParam & TeslaSafetyFlags.FLAG_HW1.value):
          self.CP.flags |= TeslaFlags.DOUBLE_STROKE_OVERRIDE.value

    # Experiment: hold our own steering correction at zero until the genuine AP1 computer reports
    # Active_nominal on its own (see WAIT_FOR_STOCK_AP in values.py). Meant to be combined with
    # TeslaDoubleStrokeOverride above -- that one keeps openpilot from faulting on the double
    # stroke, this one keeps our torque out of the way while the stock computer decides whether to
    # arm.
    if (self.CP.brand == "tesla" and not self.CP.passive and
        self.params.get_bool("TeslaWaitForStockAP")):
      for cfg in self.CP.safetyConfigs:
        if (cfg.safetyModel == structs.CarParams.SafetyModel.teslaLegacy and
            cfg.safetyParam & TeslaSafetyFlags.FLAG_HW1.value):
          self.CP.flags |= TeslaFlags.WAIT_FOR_STOCK_AP.value

    # Let the stock HW1 autopark module drive while openpilot is disengaged. Panda ignores the
    # flag on anything but teslaLegacy HW1, but the toggle is only meaningful there anyway.
    if not self.CP.passive and self.params.get_bool("TeslaStockAutopark"):
      for cfg in self.CP.safetyConfigs:
        if cfg.safetyModel == structs.CarParams.SafetyModel.teslaLegacy:
          cfg.safetyParam |= TeslaSafetyFlags.STOCK_AUTOPARK.value

    if self.CP.secOcRequired:
      # Copy user key if available
      try:
        with open("/cache/params/SecOCKey") as f:
          user_key = f.readline().strip()
          if len(user_key) == 32:
            self.params.put("SecOCKey", user_key, block=True)
      except Exception:
        pass

      secoc_key = self.params.get("SecOCKey")
      if secoc_key is not None:
        saved_secoc_key = bytes.fromhex(secoc_key.strip())
        if len(saved_secoc_key) == 16:
          self.CP.secOcKeyAvailable = True
          self.CI.CS.secoc_key = saved_secoc_key
          if controller_available:
            self.CI.CC.secoc_key = saved_secoc_key
        else:
          cloudlog.warning("Saved SecOC key is invalid")

    # Write previous route's CarParams
    prev_cp = self.params.get("CarParamsPersistent")
    if prev_cp is not None:
      self.params.put("CarParamsPrevRoute", prev_cp, block=True)

    # Write CarParams for controls and radard
    cp_bytes = self.CP.to_bytes()
    self.params.put("CarParams", cp_bytes, block=True)
    self.params.put("CarParamsCache", cp_bytes)
    self.params.put("CarParamsPersistent", cp_bytes)

    self.v_cruise_helper = VCruiseHelper(self.CP)

    self.is_metric = self.params.get_bool("IsMetric")
    self.experimental_mode = self.params.get_bool("ExperimentalMode")

    # card is driven by can recv, expected at 100Hz
    self.rk = Ratekeeper(100, print_delay_threshold=None)

  def state_update(self) -> tuple[car.CarState, structs.RadarDataT | None]:
    """carState update loop, driven by can"""

    can_strs = messaging.drain_sock_raw(self.can_sock, wait_for_one=True)
    can_list = can_capnp_to_list(can_strs)

    # Update carState from CAN
    CS = self.CI.update(can_list)

    # Update radar tracks from CAN. vEgo goes in because absolute lead speed, and the
    # acceleration and jerk derived from it, are only recoverable at this point -- the radar
    # reports closing speed and nothing else.
    RD: structs.RadarDataT | None = self.RI.update(can_list, CS.vEgo)

    self.sm.update(0)

    can_rcv_valid = len(can_strs) > 0

    # Check for CAN timeout
    if not can_rcv_valid:
      self.can_rcv_cum_timeout_counter += 1

    if can_rcv_valid and REPLAY:
      self.can_log_mono_time = messaging.log_from_bytes(can_strs[0]).logMonoTime

    self.v_cruise_helper.update_v_cruise(CS, self.sm['carControl'].enabled, self.is_metric)
    if self.sm['carControl'].enabled and not self.CC_prev.enabled:
      # Use CarState w/ buttons from the step selfdrived enables on
      self.v_cruise_helper.initialize_v_cruise(self.CS_prev, self.experimental_mode)

    # TODO: mirror the carState.cruiseState struct?
    CS.vCruise = float(self.v_cruise_helper.v_cruise_kph)
    CS.vCruiseCluster = float(self.v_cruise_helper.v_cruise_cluster_kph)

    return CS, RD

  def state_publish(self, CS: car.CarState, RD: structs.RadarDataT | None):
    """carState and carParams publish loop"""

    # carParams - logged every 50 seconds (> 1 per segment)
    if self.sm.frame % int(50. / DT_CTRL) == 0:
      cp_send = messaging.new_message('carParams')
      cp_send.valid = True
      cp_send.carParams = self.CP
      self.pm.send('carParams', cp_send)

    # publish new carOutput
    co_send = messaging.new_message('carOutput')
    co_send.valid = self.sm.all_checks(['carControl'])
    co_send.carOutput.actuatorsOutput = self.last_actuators_output
    self.pm.send('carOutput', co_send)

    # kick off controlsd step while we actuate the latest carControl packet
    cs_send = messaging.new_message('carState')
    cs_send.valid = CS.canValid
    cs_send.carState = CS
    cs_send.carState.canErrorCounter = self.can_rcv_cum_timeout_counter
    cs_send.carState.cumLagMs = -self.rk.remaining * 1000.
    self.pm.send('carState', cs_send)

    if RD is not None:
      tracks_msg = messaging.new_message('radarTracks')
      tracks_msg.valid = not any(RD.errors.to_dict().values())
      tracks_msg.radarTracks = RD
      self.pm.send('radarTracks', tracks_msg)

  def controls_update(self, CS: car.CarState, CC: car.CarControl):
    """control update loop, driven by carControl"""

    if not self.initialized_prev:
      # Initialize CarInterface, once controls are ready
      # TODO: this can make us miss at least a few cycles when doing an ECU knockout
      self.CI.init(self.CP, *self.can_callbacks)
      # signal pandad to switch to car safety mode
      self.params.put_bool("ControlsReady", True)

    if self.sm.all_alive(['carControl']):
      # radarState is a display-only input for the AP1 stock instrument cluster's lead icons. Kept
      # local to the Tesla controller instead of extending CarControl just for HUD data.
      if self.CP.flags & TeslaFlags.IC_INTEGRATION.value and self.CI.CC is not None:
        self.CI.CC.ic_radar = self.sm['radarState']
        # Live switch, so it can be toggled from /live without a restart. Re-read at ~2 Hz.
        self._ic_frame += 1
        if self._ic_frame % 50 == 0:
          self._ic_enabled = self.params.get_bool("TeslaICIntegration")
        self.CI.CC.ic_enabled = self._ic_enabled
      # send car controls over can
      now_nanos = self.can_log_mono_time if REPLAY else int(time.monotonic() * 1e9)
      self.last_actuators_output, can_sends = self.CI.apply(CC, now_nanos)
      self.pm.send('sendcan', can_list_to_can_capnp(can_sends, msgtype='sendcan', valid=CS.canValid))

      self.CC_prev = CC

  def step(self):
    CS, RD = self.state_update()

    self.state_publish(CS, RD)

    initialized = (not any(e.name == EventName.selfdriveInitializing for e in self.sm['onroadEvents']) and
                   self.sm.seen['onroadEvents'])
    if not self.CP.passive and initialized:
      self.controls_update(CS, self.sm['carControl'])

    self.initialized_prev = initialized
    self.CS_prev = CS

  def params_thread(self, evt):
    while not evt.is_set():
      self.is_metric = self.params.get_bool("IsMetric")
      self.experimental_mode = self.params.get_bool("ExperimentalMode") and self.CP.openpilotLongitudinalControl
      time.sleep(0.1)

  def card_thread(self):
    e = threading.Event()
    t = threading.Thread(target=self.params_thread, args=(e, ))
    try:
      t.start()
      while True:
        self.step()
        self.rk.monitor_time()
    finally:
      e.set()
      t.join()


def main():
  config_realtime_process(4, Priority.CTRL_HIGH)
  car = Car()
  car.card_thread()


if __name__ == "__main__":
  main()
