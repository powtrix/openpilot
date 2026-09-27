"""Shared scope and startup selection of the DK KA4 stock-SCC experiment."""
from openpilot.cereal import car
from opendbc.car.hyundai.values import CAR, HyundaiFlags, HyundaiSafetyFlags


def dk_scc_scope_supported(cp, branch) -> bool:
  try:
    if isinstance(branch, bytes):
      branch = branch.decode("utf-8")
    flags = int(cp.flags)
    required_flags = int(HyundaiFlags.CANFD | HyundaiFlags.RADAR_SCC | HyundaiFlags.CANFD_ALT_BUTTONS)
    forbidden_flags = int(HyundaiFlags.CANFD_HDA2 | HyundaiFlags.CAMERA_SCC)
    active_configs = [c for c in cp.safetyConfigs if c.safetyModel != car.CarParams.SafetyModel.noOutput]
    safe_topology = len(active_configs) == 1 and active_configs[0].safetyModel == car.CarParams.SafetyModel.hyundaiCanfd
    if safe_topology:
      safety_param = int(active_configs[0].safetyParam)
      forbidden_safety = int(HyundaiSafetyFlags.LONG | HyundaiSafetyFlags.CAMERA_SCC |
                             HyundaiSafetyFlags.CANFD_LKA_STEERING | HyundaiSafetyFlags.CANFD_LKA_STEERING_ALT)
      safe_topology = bool(safety_param & int(HyundaiSafetyFlags.CANFD_ALT_BUTTONS)) and not (safety_param & forbidden_safety)
    return bool(
      branch == "dkcarrot-wip"
      and cp.carFingerprint == CAR.KIA_CARNIVAL_4TH_GEN
      and cp.pcmCruise and not cp.openpilotLongitudinalControl
      and not cp.passive and not cp.dashcamOnly
      and (flags & required_flags) == required_flags and not (flags & forbidden_flags)
      and safe_topology
    )
  except Exception:
    return False


def dk_scc_experiment_enabled(params, cp) -> bool:
  """Read once at process startup; absent uses ON, malformed/unreadable uses OFF."""
  try:
    value = params.get("DkExperimentalScc")
    return (value is None or value is True or value in (b"1", "1")) and dk_scc_scope_supported(cp, params.get("GitBranch"))
  except Exception:
    return False
