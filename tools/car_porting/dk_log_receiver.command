#!/bin/zsh
# User-started macOS launcher. No login item, service, or network configuration.
set -eu
task_script_dir=${0:A:h}
task_repo_dir=${task_script_dir:h:h}
task_python="$task_repo_dir/.venv/bin/python"
if [[ ! -x "$task_python" ]]; then
  print '이 저장소의 Python 실행 환경(.venv)이 없습니다. 먼저 실행 환경을 준비해 주세요.'
  read -r '?엔터를 누르면 닫힙니다. '
  exit 1
fi
task_interface=$(/sbin/route -n get default 2>/dev/null | /usr/bin/awk '/interface:/{print $2; exit}')
task_address=$(/usr/sbin/ipconfig getifaddr "$task_interface" 2>/dev/null || true)
if [[ -z "$task_address" ]]; then
  print '집 네트워크의 IPv4 주소를 찾지 못했습니다. Wi-Fi/유선 연결과 VPN을 확인해 주세요.'
  print '직접 실행하려면 dk_log_receiver.py의 --bind에 사설 LAN 주소를 지정하세요.'
  read -r '?엔터를 누르면 닫힙니다. '
  exit 1
fi
cd "$task_repo_dir"
"$task_python" tools/car_porting/dk_log_receiver.py --bind "$task_address" --open-browser
