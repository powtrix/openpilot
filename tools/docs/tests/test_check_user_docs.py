import importlib.util
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
MAP_PATH = REPO_ROOT / "docs" / "user" / "docs_map.json"
CHECKER_PATH = REPO_ROOT / "tools" / "docs" / "check_user_docs.py"
SPEC = importlib.util.spec_from_file_location("check_user_docs", CHECKER_PATH)
assert SPEC is not None and SPEC.loader is not None
CHECKER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECKER)

extract_override_reason = CHECKER.extract_override_reason
find_missing_docs = CHECKER.find_missing_docs
load_mapping = CHECKER.load_mapping


def test_mapping_is_valid():
  mapping = load_mapping(MAP_PATH)
  assert mapping["version"] == 1
  assert {rule["id"] for rule in mapping["rules"]} >= {
    "settings-catalog-and-web", "cruise-buttons-and-shared-control", "radar", "tesla-vehicle-integration",
  }


def test_mapped_code_requires_documentation():
  mapping = load_mapping(MAP_PATH)
  missing = find_missing_docs(["openpilot/selfdrive/controls/radard.py"], mapping)
  assert [rule["id"] for rule in missing] == ["radar"]


def test_related_document_satisfies_rule():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/controls/radard.py",
    "docs/user/ko/radar.md",
    "docs/user/en/radar.md",
  ]
  assert find_missing_docs(changed, mapping) == []


def test_one_language_does_not_satisfy_rule():
  mapping = load_mapping(MAP_PATH)
  changed = ["openpilot/selfdrive/controls/radard.py", "docs/user/ko/radar.md"]
  assert [rule["id"] for rule in find_missing_docs(changed, mapping)] == ["radar"]


def test_any_complete_shared_cruise_document_pair_satisfies_rule():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/car/cruise.py",
    "docs/user/ko/buttons-presets.md",
    "docs/user/en/buttons-presets.md",
  ]
  assert find_missing_docs(changed, mapping) == []


def test_tesla_vehicle_change_requires_bilingual_docs():
  mapping = load_mapping(MAP_PATH)
  code_path = "opendbc_repo/opendbc/car/tesla/carstate.py"

  missing = find_missing_docs([code_path, "docs/user/ko/tesla.md"], mapping)
  assert [rule["id"] for rule in missing] == ["tesla-vehicle-integration"]

  changed = [code_path, "docs/user/ko/tesla.md", "docs/user/en/tesla.md"]
  assert find_missing_docs(changed, mapping) == []


def test_independent_rules_each_require_a_document():
  mapping = load_mapping(MAP_PATH)
  changed = [
    "openpilot/selfdrive/controls/radard.py",
    "openpilot/selfdrive/carrot_settings.json",
    "docs/user/ko/radar.md",
    "docs/user/en/radar.md",
  ]
  missing = find_missing_docs(changed, mapping)
  assert [rule["id"] for rule in missing] == ["settings-catalog-and-web"]


def test_override_requires_a_concrete_reason():
  assert extract_override_reason("Docs-Not-Needed: 내부 함수만 분리했고 사용자 동작은 동일함")
  assert extract_override_reason("Docs-Not-Needed: N/A") == ""
  assert extract_override_reason("Docs-Not-Needed:") == ""


def test_validation_upload_docs_disclose_full_campaign_and_transfer_bounds():
  required = {
    REPO_ROOT / "docs/user/ko/dashcam-log-sharing.md": [
      "full rlog만 최대 3개",
      "최대 10개 캡처/서로 다른 full rlog 30개",
      "14개 캡처/42개 rlog",
      "최대 5개 캡처/750 MiB",
      "클라이언트 누적 전송량 상한은 없습니다",
      "같은 캡처 전체가 다시 전송",
      "약 6시간 간격으로 최대 7일",
      "장치별 일일 1 GiB",
      "다음 날 자동 재시도",
      "테더링 제공 휴대폰",
      "HDA1(비-HDA2)",
      "`https://adot.synology.me`",
      "표준 HTTPS 443",
      "배포 환경 변수도 이 고정을 바꿀 수 없습니다",
    ],
    REPO_ROOT / "docs/user/en/dashcam-log-sharing.md": [
      "at most three full rlogs",
      "at most 10 captures / 30 distinct full rlogs",
      "14 captures / 42 rlogs",
      "5 captures / 750 MiB",
      "no cumulative transfer cap",
      "resend the entire capture",
      "every 6 hours for up to seven days",
      "1 GiB per-device daily limit",
      "retry automatically the next day",
      "phone providing the current Wi-Fi default gateway",
      "HDA1 (not HDA2)",
      "`https://adot.synology.me`",
      "standard HTTPS port 443",
      "deployment environment variable cannot override that pin",
    ],
  }
  for path, fragments in required.items():
    source = path.read_text(encoding="utf-8")
    for fragment in fragments:
      assert fragment in source, f"{path}: {fragment}"
