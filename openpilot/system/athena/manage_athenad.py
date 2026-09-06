#!/usr/bin/env python3

import time
from multiprocessing import Process

from openpilot.common.external_data import third_party_data_sharing_enabled
from openpilot.common.params import Params
from openpilot.system.manager.process import launcher
from openpilot.common.swaglog import cloudlog
from openpilot.system.hardware import HARDWARE
from openpilot.system.version import get_build_metadata

ATHENA_MGR_PID_PARAM = "AthenadPid"
PRIVACY_POLL_INTERVAL_S = 1.0
ATHENAD_RESTART_DELAY_S = 5.0
ATHENAD_STOP_TIMEOUT_S = 5.0
ATHENA_UPLOAD_QUEUE_PARAM = "AthenadUploadQueue"


def _stop_athenad(proc: Process | None) -> None:
  if proc is None:
    return
  if proc.is_alive():
    proc.terminate()
    proc.join(ATHENAD_STOP_TIMEOUT_S)
  if proc.is_alive():
    proc.kill()
    proc.join()
  else:
    proc.join()


def run_athenad_manager(params: Params, *, sleep=time.sleep) -> None:
  """Keep athenad running only while explicit third-party consent is on."""
  proc: Process | None = None
  restart_after = 0.0
  disabled_state_cleaned = False
  try:
    while True:
      enabled = third_party_data_sharing_enabled(params)
      if not enabled:
        if proc is not None:
          cloudlog.info("stopping athena daemon: third-party data sharing disabled")
          _stop_athenad(proc)
          proc = None
        if not disabled_state_cleaned:
          # Never replay uploads queued by a remote service before consent was
          # withdrawn if the user later opts in again.
          params.remove(ATHENA_UPLOAD_QUEUE_PARAM)
          disabled_state_cleaned = True
        restart_after = 0.0
        sleep(PRIVACY_POLL_INTERVAL_S)
        continue

      disabled_state_cleaned = False

      if proc is not None and not proc.is_alive():
        proc.join()
        cloudlog.event("athenad exited", exitcode=proc.exitcode)
        proc = None
        restart_after = time.monotonic() + ATHENAD_RESTART_DELAY_S

      if proc is None and time.monotonic() >= restart_after:
        cloudlog.info("starting athena daemon")
        proc = Process(name='athenad', target=launcher, args=('openpilot.system.athena.athenad', 'athenad'))
        proc.start()

      sleep(PRIVACY_POLL_INTERVAL_S)
  finally:
    _stop_athenad(proc)


def main():
  params = Params()
  dongle_id = params.get("DongleId")
  build_metadata = get_build_metadata()

  cloudlog.bind_global(dongle_id=dongle_id,
                       version=build_metadata.openpilot.version,
                       origin=build_metadata.openpilot.git_normalized_origin,
                       branch=build_metadata.channel,
                       commit=build_metadata.openpilot.git_commit,
                       dirty=build_metadata.openpilot.is_dirty,
                       device=HARDWARE.get_device_type())

  try:
    run_athenad_manager(params)
  except Exception:
    cloudlog.exception("manage_athenad.exception")
  finally:
    params.remove(ATHENA_MGR_PID_PARAM)


if __name__ == '__main__':
  main()
