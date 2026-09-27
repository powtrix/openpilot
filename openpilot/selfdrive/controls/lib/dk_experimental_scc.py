"""Mode2 target comparison only; no new CAN, brake, or resume authority."""
import math

from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N


MIN_SET_SPEED = 30.0 / 3.6
MAX_INPUT_AGE_NS = round(3 * DT_MDL * 1e9)
ADJUST_BUTTONS = frozenset(("accelCruise", "decelCruise", "resumeCruise", "setCruise"))
INPUT_SERVICES = ("carState", "selfdriveState", "longitudinalPlan", "radarState", "modelV2")


class DkExperimentalScc:
  def __init__(self):
    self.holding = False
    self.last_oem_speed = None
    self.cruise_was_enabled = False
    self.blocked_model_time = 0
    self.adjust_buttons_down = set()

  def _release(self, model_time):
    self.holding = False
    self.last_oem_speed = None
    self.blocked_model_time = max(self.blocked_model_time, model_time)

  @staticmethod
  def _fresh(timestamp, now_ns):
    return timestamp > 0 and 0 <= now_ns - timestamp <= MAX_INPUT_AGE_NS

  def update(self, sm, CC, baseline, mode0_target, speed_from_pcm, now_ns):
    """Lower a valid moving-lead target, then hold the observed OEM set speed.

    After an actual reduction request, loss of perception stops further lead
    reductions but does not restore a target above the observed OEM set speed.
    A source RES/SET or re-engagement releases this hold. Its model frame cannot
    immediately re-arm it, and a held physical button keeps the override open.
    Card independently blocks ordinary Mode2 speed-increase RES for the whole
    experiment, including when it observes a newer OEM speed than this process.
    Existing stop/resume/cancel requests and all native interlocks are unchanged.
    """
    CS = sm['carState']
    plan = sm['longitudinalPlan']
    model_time = max(int(plan.modelMonoTime), int(sm.logMonoTime['modelV2']))
    if speed_from_pcm != 2:
      self._release(model_time)
      self.cruise_was_enabled = False
      self.adjust_buttons_down.clear()
      return baseline

    car_fresh = (CS.canValid and sm.valid['carState'] and
                 self._fresh(sm.logMonoTime['carState'], now_ns))
    if car_fresh:
      adjustment = False
      for event in CS.buttonEvents:
        name = str(event.type)
        if name in ADJUST_BUTTONS:
          adjustment = True
          if event.pressed:
            self.adjust_buttons_down.add(name)
          else:
            self.adjust_buttons_down.discard(name)
      cruise_enabled = CS.cruiseState.enabled
      reengaged = cruise_enabled and not self.cruise_was_enabled
      self.cruise_was_enabled = cruise_enabled
      if not cruise_enabled or reengaged or adjustment or self.adjust_buttons_down:
        self._release(model_time)
        return baseline
      oem_speed = float(CS.cruiseState.speed)
      if math.isfinite(oem_speed) and oem_speed > 0:
        self.last_oem_speed = oem_speed

    # These existing requests do not derive their permission from this target.
    # In particular, neither a stopped lead nor this hold changes resume logic.
    if (not CC.enabled or CC.cruiseControl.cancel or CC.cruiseControl.resume or
        CS.standstill or CS.cruiseState.standstill):
      return baseline

    target = baseline
    if self.holding and self.last_oem_speed is not None:
      target = min(target, self.last_oem_speed)

    if (not car_fresh or self.last_oem_speed is None or
        CS.brakePressed or CS.gasPressed or CS.brakeHoldActive or CS.parkingBrake or CS.accFaulted or
        not math.isfinite(CS.vEgo) or CS.vEgo <= MIN_SET_SPEED or
        not math.isfinite(baseline) or not math.isfinite(mode0_target) or
        not sm.all_checks(INPUT_SERVICES) or
        not all(self._fresh(sm.logMonoTime[s], now_ns) for s in INPUT_SERVICES)):
      return target

    radar = sm['radarState']
    model = sm['modelV2']
    lead = radar.leadOne
    # The planner normally consumes the preceding radar publication. Require
    # bounded age, not equality with the newer radar message received here.
    if (plan.modelMonoTime <= self.blocked_model_time or
        plan.modelMonoTime != sm.logMonoTime['modelV2'] or radar.mdMonoTime != plan.modelMonoTime or
        not self._fresh(plan.deprecated.radarStateMonoTime, now_ns) or
        not self._fresh(model.timestampEof, now_ns) or
        not plan.hasLead or str(plan.longitudinalPlanSource) != 'lead0' or plan.xState != 0 or
        not lead.status or not all(math.isfinite(v) for v in (lead.dRel, lead.vRel, lead.vLead, lead.modelProb)) or
        lead.dRel <= 0 or lead.vRel >= 0 or lead.vLead < 0 or lead.modelProb < 0.5 or
        not model.leadsV3 or model.leadsV3[0].prob < 0.5 or not math.isfinite(model.leadsV3[0].prob)):
      return target

    speeds, accels = plan.speeds, plan.accels
    if (len(speeds) != CONTROL_N or len(accels) != CONTROL_N or
        not all(math.isfinite(v) and v >= 0 for v in speeds) or
        not all(math.isfinite(a) for a in accels) or
        # Failed MPC solves reset the entire trajectory to zero. A moving-lead
        # experiment must not interpret that reset as a demand for minimum speed.
        speeds[0] <= 0 or speeds[-1] >= speeds[0] or speeds[-1] >= CS.vEgo):
      return target

    candidate = max(MIN_SET_SPEED, mode0_target)
    if candidate < min(baseline, self.last_oem_speed):
      self.holding = True
      return min(target, candidate, self.last_oem_speed)
    return target
