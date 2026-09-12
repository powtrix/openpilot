# DK KA4 진단 전용 기록 — schema 1

이 문서는 `dkcarrot-wip`의 KA4 진단 구현과 분석 기준을 설명하는 기술 문서다. 사용자 설정 가이드나 실제 차량 문제의 해결 확인서가 아니다. 아래 후보 조건은 기록할 구간을 고르는 기준이며, 고장 판정이나 제어 변경 기준이 아니다.

## 변경 범위와 증거 수준

- 관측기는 `dkcarrot-wip` + `KIA_CARNIVAL_4TH_GEN`에서만 생성된다. 실제 `CarParams`의 순정 SCC/OP 롱컨 상태를 기록하며, 설정을 추측해서 강제로 바꾸지 않는다.
- `card`의 기존 제어 적용 전후를 읽는다. 기존 `CarControl`, 제어기 상태, CAN 배열 및 실제 출력값을 수정하지 않는다. 추가 RES/SET, 가감속, 조향, ECU 비활성화 명령을 보내지 않고 OEM 경고를 숨기지 않는다.
- 0912 릴리스는 버튼 전송 무결성 보완과 진단 보강을 구분한다. 버튼 인코딩이 수신 원본을 수정하지 않도록 하고, DK KA4 순정 SCC의 일반 RES/속도동기화도 새로운 OEM 버튼 카운터가 있을 때만 제출한다. 중복 호스트 요청을 줄이는 수정이지 ECU 수락·재출발 성공의 보장이 아니다. 진단 자체는 제어를 바꾸지 않으며, 재출발·점검 경고·조향·제동의 실차 해결 완료를 주장하지 않는다. 조향 보정·모델·가감속 명령이나 경고 마스킹을 새로 적용하지 않는다.
- 새로운 네트워크 업로드, NAS 수신 서버, 공식 서버 전송 경로를 추가하지 않는다. 별도 보존본은 장치에만 저장한다. 다른 기존 업로더의 설정·동작을 이 기능이 대신 변경하거나 차단하는 것은 아니다.
- 진단 요약의 허용 필드에는 VIN, 동글/계정 ID, GPS, 임의 Params 문자열, 원시 CAN 바이트를 넣지 않는다. 그러나 **복사하는 full rlog는 기존 원본과 동일하므로 위치·장치 식별 정보 등 개인정보를 포함할 수 있다.** 공개 저장소나 Wiki에 실제 로그·manifest를 올리지 않는다.

다음 네 단계를 구분해야 한다.

| 기록 | 의미 | 증명하지 못하는 것 |
| --- | --- | --- |
| `shadow` | 기존 측정값에 적용한 가상 조건·비교 계산 | 실제 명령 전송, 차량 반응, 개선 효과 |
| `request`, `output` | 제어 요청과 `CI.apply`가 반환한 소프트웨어 출력 | Panda 승인, 물리 CAN 전달, ECU 수락 |
| `submitted_can` | 기존 `sendcan` 제출에 들어간 주소·bus·개수와 버튼 값/카운터 요약 | ECU가 버튼을 눌림으로 인식했다는 사실, 정차 타이머 재설정 |
| `car`, 수신 CAN 및 원본 rlog | 샘플링된 차량 상태와 수신 관측 | 단독으로 특정 코드 변경이 원인이라는 인과관계 |

`raw_before`/`raw_after`도 독립적인 물리 CAN 캡처가 아니라 `CarState`가 보관한 디코딩 캐시다. 다른 코드가 캐시를 수정했을 가능성을 명시한다. 원본 `can`/`sendcan`, bus, 카운터, 타임스탬프와 함께 대조해야 한다.

v2의 `lfahda_cluster.packet_sources[].decoded_values`는 해당 parser가 실제로 구독한 bus의 팝업·상태 값을 따로 복사한다. 수정 가능한 `values` 캐시와 비교할 수 있지만, 구독하지 않는 bus의 값을 새로 만들어 내지는 않는다. OEM 측 수신과 호스트 제출/반환/거부를 비교하려면 여전히 full rlog의 원본 `can`/`sendcan`이 필요하다.

## 다섯 증상별 가설과 반증 기준

### 1. 정차 후 재출발 안 됨 — `resume`

가설을 나눠 확인한다: 계획기의 재출발 조건이 요청을 만들지 못했는지, 제어기 인터록·버튼 원본 갱신 문제로 RES 제출이 없었는지, 제출 이후 ECU가 이를 수락하지 않았는지, 또는 SCC가 이미 해제·고장 상태였는지.

- 측정: `car.standstill`, `car.cruise_standstill`, 속도·브레이크·가속페달·Auto Hold·주차브레이크, `planner.should_stop`, `planner.final_speed_mps`, `request.resume`, 제어기 전후 버튼/대기/keepalive 상태, `submitted_can.buttons`/`button_counters`.
- 수신 대조: SCC `ACCMode`, `InfoDisplay`, `SysFailState`, `TakeOverReq`, 선행차 거리·상대속도, `ADRV_0x161.ALERTS_5` 및 실제 정지→이동 관측. 모든 raw 값은 해당 `raw_before.<메시지>.values` 안에 있다.
- 가상 비교: `shadow.legacy_resume`는 활성+SCC 정차+유효한 계획 속도 배열+마지막 계획 속도 `> 0.1 m/s`, `shadow.should_stop_resume`는 같은 기본 조건에서 `shouldStop=False`를 비교한다. 둘 다 버튼 인터록·전달·ECU 수락을 재현하는 시뮬레이터가 아니다.
- 반증: 실제 `request.resume=True`와 RES 제출이 이미 존재하면 “계획기가 요청을 전혀 못 만들었다”는 설명으로 그 구간을 설명할 수 없다. 반대로 요청이 없는데 ECU 버튼 수락 실패를 원인으로 확정할 수 없다. RES 제출 후 차량이 움직이지 않는다고 정차 타이머가 재설정됐다고 해석하지 않는다.

### 2. 인게이지 시 주행보조/HDA 점검 경고 — `engage_warning`

가설은 기존 OEM 경고가 표시된 경우, 인게이지 전부터 존재한 고장, 인게이지 뒤 ECU가 새로 만든 고장, 설정/토폴로지 불일치로 다른 제어 경로를 실행한 경우로 나눈다.

- 측정: `request.enabled`, `lat_active`, `long_active`의 변화, session의 CP flags/`openpilotLongitudinalControl`/`pcmCruise`, 초기 허용 설정값.
- 수신 대조: `CCNC_0x162.FAULT_DAS/FAULT_HDA/FAULT_LFA/FAULT_SCC`, MDPS `LKA_FAULT/LFA2_FAULT`, SCC `SysFailState/TakeOverReq`, `ADRV_0x161`의 `ALERTS_1..5`·음향, `LFAHDA_CLUSTER`의 HDA 상태·팝업·음향. “출발하려면 스위치/페달”과 “주행보조 시스템 점검”을 같은 신호로 취급하지 않는다.
- 반증: 같은 고장값이 인게이지 전부터 존재하면 “그 순간 새로 발생했다”는 가설은 반박된다. 정상 순정 SCC 경로에서 0x162를 새로 제출하지 않았다면 “이 분기가 점검 표시값을 직접 썼다”는 설명은 배제할 수 있지만, OEM ECU가 다른 출력에 반응해 고장을 만들었을 가능성까지 배제할 수는 없다.
- 누락된 메시지·오래된 캐시를 `0=정상`으로 바꾸지 않는다. 정확한 계기판 문구와 동기화된 기록이 없으면 표시와 신호의 연관은 미확정으로 남긴다. 이 기능은 고장값을 가려서 확인하지 않는다.

### 3. 커브 추종·치우침 — `curve`

가설은 모델/차선 계획 자체의 위치, 정적·동적 경로 보정, 계획 곡률 대비 제어 출력 제한, 차량의 실제 추종 응답을 분리한다.

- 측정: `lateral.path_before_static_y_m`, `path_y_m`, 정적·동적 offset, 차선 확률·차선 좌표, 차선 사용 여부·폭, 계획 곡률, 실제/목표 곡률, 요청/출력 토크·각도, 제어기 포화 상태와 전후 제한값, 운전자 토크.
- 속도별 비교: 오프라인 보고서는 `< 30`, `30–< 80`, `≥ 80 km/h` 구간을 분리한다. 서로 다른 속도·차선 신뢰도·운전자 개입 구간의 단순 평균을 개선 증거로 삼지 않는다.
- 반증: 계획 경로부터 치우쳐 있고 차량이 그 경로를 따라가면 출력 제한만으로 설명할 수 없다. 계획이 안정적인데 소프트웨어 출력이 제한되고 실제 곡률 오차가 함께 커지면 계획 위치만을 원인으로 단정하지 않는다.
- 토크 제어 차량에서는 요청/출력의 각도 필드가 직접적인 조향각 명령이라는 보장이 없다. CP 제어 방식과 lateral controller 종류를 먼저 확인하고, 각도 차이는 기술 통계로만 사용한다. v2는 `lateral.steer_control_type`/`angle_semantics`, `output.torque_output_can`, `controller_before/after.limits`의 실제 토크·증감·운전자 개입 상한, `car.use_lane_line_speed_kph`, `lateral.active_lane_line`/`model_desired_curvature`를 함께 기록한다. `torque_output_can`도 소프트웨어 반환값이며 물리 ECU 수신 증명은 아니다.
- `liveDelay`는 기존 full rlog에서 상태·`validBlocks`·지연 추정치를 확인한다. 진단 때문에 `card`에 새 구독이나 지연 추정·제어 변경을 추가하지 않는다.

### 4. 커브 출구에서 조향이 늦게 풀림 — `unwind`

가설은 목표 조향이 늦게 감소하는 경우, 출력 제한·운전자 개입 복귀 상태 때문에 출력이 늦는 경우, 출력은 줄었지만 차량 각도가 늦게 변하는 경우로 나눈다.

- 측정: 요청/출력 각도·토크·곡률, `shadow.requested_angle_rate_dps`/`output_angle_rate_dps`, 측정 조향 속도, driver torque, `steering_pressed`, override·recovery·rate-limit 관련 제어기 전후 상태.
- 가상 값 중 각도 변화율은 인접 기록의 차분이다. 실제 EPS가 받은 명령 변화율이나 새로운 제어 목표가 아니다. 차분 시간 간격이 없거나 1초를 넘으면 해당 값은 누락으로 남긴다.
- `shadow.unwind_active`는 이전 요청 각도 절댓값이 `> 5°`인 상태에서 요청 절댓값이 감소하면 열리는 10초 관측 창 안에서, 측정 각도 절댓값이 요청 절댓값보다 `> 3°` 큰 조향 활성 구간이다. 같은 감소 조건이 다시 나타나면 창을 갱신한다. 요청이 0°로 바뀐 뒤 계속 0°여도 남은 측정 각도를 관찰하되, 5° 이하 요청의 작은 감소만으로 새 창을 열지는 않는다. 실제 복귀 지연 확정값은 아니다.
- 반증: 요청 자체가 계속 유지되면 출력 복귀 지연만이 원인이라는 가설은 맞지 않는다. 출력은 즉시 줄어드는데 실제 각도만 늦게 변한다면 소프트웨어 출력 ramp만으로 설명할 수 없다. 운전자 개입·데이터 노후화가 동반되면 그것을 먼저 분리한다.

### 5. 앞차 접근 시 늦은 감속·급제동 — `braking`

가설은 순정 SCC의 감속 요구가 늦는 경우, 요구는 있지만 실제 감속 응답이 늦는 경우, 선행차 선택/거리/상대속도 변화 때문에 비교 조건이 달라진 경우로 나눈다.

- 측정: 선행차 존재·track ID·거리·상대속도, SCC `aReqRaw/aReqValue`, OP 요청/출력 가속도, 실제 `car.accel_mps2`와 샘플 차분 `jerk_mps3`, SCC 모드·인터록·신호 신선도.
- v2는 `perception.model_leads`에 최대 3개 `leadsV3`의 확률·예측 시점 및 x/y/v/a 각 최대 6개 값을 추가한다. 이는 서로 다른 미래 시점의 모델 선행차 가설이며 반드시 서로 다른 차량 3대를 의미하지 않는다. 선택된 OP 선행차, 순정 SCC의 `ACC_ObjDist/ACC_ObjRelSpd`, 전체 모델 가설은 같은 센서 결과로 섞지 않는다.
- `braking_observation.window_min_accel_mps2`/`window_min_accel_mono_ns`는 상세 샘플 사이에 들어온 모든 `card` 프레임의 실측 가속도 최솟값과 시각이다. `window_start_mono_ns`가 집계 시작이며, 기록 오류 뒤에는 마지막 성공 기록 이후의 더 긴 창일 수 있다. 10 Hz 기록만으로 짧은 급감속 최댓값을 놓치지 않도록 하되, 원본 100 Hz `carState`를 대체하지 않는다. 운전자 브레이크가 동반된 실측 감속을 전부 SCC 명령의 결과로 귀속하지 않는다.
- 가상 비교 `gentle_decel_mps2`: 선행차 속도가 일정하고 가정 여유거리 8 m를 유지한다고 놓고 `-min(1.5, closing_speed² / (2 × (distance - 8)))`을 계산한다. 유효 선행차가 없거나 거리가 8 m 이하면 값이 없다. **실제 감속 명령, 권장 설정값, 안전거리 또는 정답 모델이 아니다.** 순정 SCC를 이 값으로 제어하지 않는다.
- 반증: SCC의 음의 가속도 요구가 충분히 먼저 관측되고 차량 감속만 늦으면 “감속 요구 자체를 늦게 보냈다”는 설명은 약해진다. 앞차 track이 바뀌었거나 수신값이 오래됐으면 단순 시간차로 SCC 응답 지연을 확정하지 않는다.
- 오프라인 감속 시작 시점은 기록된 가속도가 `≥ -0.1`에서 `< -0.1 m/s²`로 바뀐 인접 샘플이다. 이는 샘플링된 관측 시각이며 ECU의 정확한 내부 판단 시각이 아니다.

## 기록 형태와 후보 제한

구현: `openpilot/selfdrive/carrot/dk_vehicle_diagnostics.py`.

- 기존 `logMessage`를 통해 `event="dk_vehicle_diag"`, `schema=1`의 `session`과 `sample`을 full rlog에 남긴다. v2 추가 필드는 호환 가능한 확장이며 session의 `diagnostics_version="dk-vehicle-diag-v2"`로 구분한다. session에는 branch/commit, 실제 CP 일부 및 시작 시 허용된 수치 설정값을 기록한다. 최초 1회 및 이후 60초마다 같은 시작 메타데이터를 재공지하여 보존기가 재시작한 뒤에도 다시 받을 수 있게 한다. 재공지 때 Params를 새로 읽는 것은 아니므로 매분의 최신 설정 스냅샷으로 해석하지 않는다.
- 기본 샘플 간격은 100 ms이며, 상태 전환·후보가 있으면 최소 50 ms 간격이다. 모든 주행 프레임이 상세 샘플이 되는 것은 아니다.
- 전환 목록은 최대 32개, 제출 CAN 주소/bus 집계는 최대 64개 키로 제한한다. 건너뜀·오류·전환 누락 횟수를 `counters`에 기록한다.
- 각 서비스는 `valid`, `alive`, `mono_ns`, `age_ms`를 기록한다. raw 캐시는 `missing`, `missing_fields`, parser/bus별 `packet_sources`와 나이를 기록한다. 필드 미지원과 값 0을 구분한다.
- 일반 관측 후보 topic의 재표시는 30초, 별도 파일 보존은 같은 topic당 120초 간격이다. 아래 v2의 우선 `braking_capture`는 일반 쿨다운과 별개이며, 이유별 관측 재표시는 20초 간격이다. 따라서 후보 수와 실제 증상 발생 횟수는 같지 않다.
- `shadow.curve_active`와 `shadow.unwind_active`는 재표시 제한과 무관한 각 샘플의 지속 관측 조건이다. topic이 그 샘플에 없더라도 조건은 참일 수 있다. dwell 시간이 지난 보존 후보와 구분하며, 둘 다 실제 제어기 상태나 고장 판정이 아니다.
- v2의 커브/풀림 관측 활성 기준은 전체 `request.enabled`가 아니라 **실제 `request.lat_active`**다. AlwaysLateral처럼 전체 인게이지가 false여도 조향 활성인 구간을 누락하지 않는다. 기록 조건을 바꾼 것이며 실제 조향 활성화 조건을 바꾸지 않는다.

| topic | 주요 보존 후보 조건 | 해석 제한 |
| --- | --- | --- |
| `resume` | 물리/SCC 정차·resume·SCC InfoDisplay·출발 안내 전환, 또는 활성 정차 3초 | 실제 출발 실패 판정 아님 |
| `engage_warning` | 활성/조향활성·고장·경고/음향 등의 상태 전환, 시작 시 이미 활성 | 정상 인게이지나 안내음도 후보가 될 수 있음 |
| `curve` | 조향 활성, 속도 `> 3 m/s`, 요청 곡률 절댓값 `> 0.002 /m` 또는 요청/측정 각도 차이 `> 5°`가 0.3초 지속; 아래 토크 기반 관측도 별도 가능 | 치우침 또는 추종 실패의 증명 아님 |
| `unwind` | 조향 활성 상태, 이전 요청 절댓값 `> 5°` 뒤 절댓값 감소로 열린 10초 창 안에서 측정 절댓값 `> 요청 절댓값 + 3°`가 0.2초 지속; 아래 토크 부호 반대 관측도 별도 가능 | 고정 0° 요청 뒤 남은 각도도 관측; 토크 차량의 각도 필드 의미를 별도 확인 |
| `braking` | 브레이크 on/off 전환, 활성 상태에서 측정 감속이 `-0.3 m/s²` 아래로 전환, 또는 거리 `< 50 m`·상대속도 `< -0.8 m/s` 접근이 0.3초 지속 | 급제동·늦은 감속 자체의 판정 아님 |

실제 CP가 토크형이고 `lat_active=True`, 속도 `> 3 m/s`일 때는 각도 필드 없이도 다음을 별도로 관찰한다. `shadow.torque_saturation_active`는 정규화 출력 토크 절댓값 `≥ 0.95`, `shadow.lateral_accel_error_active`는 제어기가 기록한 목표/실측 횡가속도 차이 절댓값 `≥ 0.75 m/s²`, `shadow.torque_opposed_active`는 요청/출력 토크 각각의 절댓값 `> 0.05`이면서 부호가 반대인 조건이다. 앞의 두 조건이 각각 0.3초 지속하면 curve, 마지막 조건이 0.2초 지속하면 unwind 후보가 된다. 이 boolean은 각도 기반 `curve_active`/`unwind_active`와 분리하여 기록한다. 정상 출력 제한·필터·운전자 개입에도 나타날 수 있으므로 EPS 포화나 위험한 복귀 지연으로 자동 판정하지 않는다. `lateral.accel_error_mps2`는 목표−실측 차이이며 해당 입력 필드가 없으면 누락으로 남긴다.

### 선행차 누락·해제 직후를 놓치지 않는 v2 보존 후보

기존 일반 접근 후보는 선택된 lead가 없거나 브레이크로 즉시 제어가 해제되면 실제 급감속을 놓칠 수 있었다. v2는 다음 **관측만** 별도로 보존한다. 숫자는 제동 명령이나 권장 운전/차간거리 설정이 아니다.

| `braking_capture.reasons[].reason` | 관측 조건 | 보존 우선순위 |
| --- | --- | --- |
| `driver_brake_intervention` | 속도 `≥ 3 m/s`에서 브레이크 false→true, OP 또는 순정 cruise가 현재 활성/최근 5초 내 활성 | 1: 운전자 개입; 정상적인 수동 제동도 포함 |
| `scc_takeover_request` | 최근 3초 내 속도 `≥ 3 m/s`, 캐시 `TakeOverReq`가 다른 값에서 양수로 전환 | 2: 인수요청 관측; 차량 고장 판정 아님 |
| `hard_deceleration` | 최근 3초 내 속도 `≥ 3 m/s`, 실측 가속도 `≤ -3 m/s²`가 100 ms 지속 | 2: 강한 감속 관측; 운전자/차량/원인 구분 별도 필요 |

세 조건 모두 OP lead 존재를 요구하지 않는다. 강한 감속은 제어가 해제되었거나 수동 운전 중에도 기록한다. 프레임 수준으로 조건을 관찰하되 상세 기록은 기존 최대 20 Hz를 유지한다. 이유·전환은 유한한 버퍼에 모았다가 다음 샘플에 기록한다. `braking_observation`의 최근 활성 여부·강한 감속 지속 조건은 쿨다운과 무관한 설명 자료이며, 실제 위기/고장을 판정한 상태값이 아니다. 짧은 노이즈, 저속 정차, 평범한 브레이크 조작도 반증 자료와 함께 구분해야 한다.

위 표는 **보존 후보 조건**이다. 오프라인 도구는 커브/풀림 통계에 기록된 `shadow.curve_active`/`shadow.unwind_active`를 우선 사용한다. 이 boolean 필드가 없는 이전 기록에서만 활성+속도 `> 3 m/s`와 요청 곡률 절댓값 `> 0.002 /m` 또는 `요청 각도 × 요청 각도 변화율 < 0`을 각각 대체 기준으로 쓴다. 대체 기준은 고정 0° 요청 뒤의 잔류 각도 등을 놓칠 수 있어 명시 플래그 사용/대체 분류 표본 수를 따로 표시한다. 접근 통계는 거리 `≤ 80 m`·상대속도 `< -0.5 m/s`로 분류한다. 후보·지속 구간·기술 통계의 집계 숫자가 달라도 곧바로 기록 결함을 뜻하지 않는다.

## 장치 내 보존과 웹 읽기

구현: `openpilot/selfdrive/carrot/dk_diagnosticsd.py`, `openpilot/selfdrive/carrot/server/features/dk_diagnostics.py`.

comma 기본 저장 위치는 `/data/media/0/dk-diagnostics/<capture_id>/`이다. `LOG_ROOT`를 바꾼 환경에서는 해당 log root의 형제 `dk-diagnostics` 디렉터리를 쓴다. 기존 `/data/media/0/realdata`나 수동 업로드/북마크 보관함과 구분한다.

- 보존기 실행 중 전체 최대 10건, topic별 최대 2건, 합계 1 GiB, 최대 7일 정책을 적용한다. 한 캡처가 여러 topic에 속할 수 있다. 용량·건수 제한 때문에 7일보다 먼저 오래된 **복사본**이 지워질 수 있다. 만료 정리는 보존기가 실행될 때 적용되므로 장치 전원이 꺼져 있거나 다른 브랜치로 전환한 동안의 즉시 삭제를 보장하지 않는다.
- v2 우선 보존 후보는 `braking`에만 속한다. 같은 route의 braking 캡처 생성 후 20초 미만에 들어온 개입/인수요청/급감속은 첫 캡처의 앞뒤 구간을 유지하며 병합하고, 우선순위만 높인다. manifest의 `braking_observations`는 최대 8개이며 그 밖의 지속 자료는 원본 rlog에 남는다. 프로세스가 재시작해도 manifest의 우선순위를 유지한다.
- 일반 후보(0) → 운전자 개입(1) → 인수요청/강한 감속(2) 순으로 복사본을 보존한다. 같은 주제에 상위 우선순위 2건이 있으면 하위 후보는 새 복사본을 만들지 않는다. 같은 우선순위의 새 후보는 더 오래된 것을 대체할 수 있으며 TTL·전체 건수·용량 상한은 우선 보존에도 그대로 적용한다. 따라서 모든 사건의 영구 보존을 보장하는 기능이 아니다.
- manifest 저장 전에 중단되거나 manifest가 손상된 캡처 ID 디렉터리도 전체 건수·용량·만료 정리에 포함한다. 유효 manifest가 없는 경우 디렉터리 수정 시각을 만료 기준으로 쓰며, 손상본을 정상 분석 입력으로 취급하지 않는다.
- 이벤트 수신 당시 현재 route의 최신 logger segment와 그 앞뒤, 최대 3개 full rlog를 대상으로 한다. 0번 segment에서는 앞 구간이 없으므로 최대 2개다. 이벤트 timestamp로 정밀 탐색한 구간이라는 뜻은 아니며 선택 기준을 manifest에 남긴다.
- 완료 확인 대상은 `rlog.zst`, `rlog.bz2`, `rlog`이며 qlog로 조용히 대체하지 않는다. `rlog.lock`이 없고, inode·파일 크기·수정 시각이 서로 다른 두 검사에서 최소 5초 동안 같아야 복사한다. 복사 중 크기 변경 및 복사 후 lock을 다시 검사한다. lock 해제만으로 압축 writer의 종료를 단정하지 않기 위한 보수적 안정화 검사이며, 내용 전체의 디코딩 성공을 대신 보증하지는 않는다. 파일당 256 MiB 및 복사 후 여유 공간 5 GiB 조건을 적용한다.
- `manifest.json`을 먼저 저장한 `pending` 상태는 프로세스 재시작 후 이어서 처리한다. 확보가 끝나면 `ready`, 생성 후 180초가 지나도 이웃 구간 등이 없으면 누락 이유를 가진 `partial`로 남긴다. 재시작 직후에는 그 프로세스가 해당 pending을 처음 검사한 때부터 최소 5초를 더 기다려 파일 안정화 검사 기회를 준 뒤 partial로 확정한다. `ready`는 파일 확보 상태이지 차량 정상 판정이 아니다.
- manifest에는 선택 route/segment, 후보 이벤트·session, 실제 복사 파일·크기·SHA-256, 누락 사유, 만료 시각 및 `network_upload=false`가 들어간다.
- 진단 보존기는 원본 rlog, 기존 사용자 북마크, NAS 파일을 삭제하지 않는다. 수집 실패가 차량 제어로 전파되지 않도록 별도 프로세스와 오류 경계로 분리한다.

같은 로컬 네트워크의 Carrot Web에서 읽을 수 있는 API:

```text
GET /api/dk/diagnostics
GET /api/dk/diagnostics/<capture_id>/manifest.json
GET /api/dk/diagnostics/<capture_id>/<segment_index>-rlog.zst
```

실제 artifact 확장자는 manifest를 따른다. API는 GET/HEAD만 제공하며 수집 시작·삭제·업로드 동작이 없다. 사설 IP/localhost Host, 실제 로컬 TCP 상대, 제공된 Origin의 동일 출처를 검사한다. 인터넷의 임의 도메인으로 직접 호출하는 공개 다운로드 API가 아니다. 이 기능만으로 다른 망에 있는 NAS에 자동 전달되지는 않는다.

수동 휴대폰 전달은 웹당근 **Logs → dk 로그전달**(`/dk-logs`)에서 원본 rlog/manifest 묶음을 내려받고, 집에서 별도 dk 수신기로 전달한다. 전체 주행이나 영상 백업은 아니다. 자동 업로드·새 Params 없이 사용자 요청으로만 동작하며, 사용법과 보안·누락·용량 제한은 [DK 수동 휴대폰 로그 전달](dk_phone_log_transfer.md)에 한글/영문으로 설명한다.

## 로컬 오프라인 분석

도구: `tools/car_porting/dk_diagnostics_report.py`.

```sh
python3 tools/car_porting/dk_diagnostics_report.py /path/to/0-rlog.zst /path/to/1-rlog.zst
python3 tools/car_porting/dk_diagnostics_report.py --jsonl /path/to/diagnostics.jsonl
```

- 이미 내려받은 로컬 파일만 받는다. 원격 route 검색, 다운로드, 업로드, Params 변경, CAN 발행을 하지 않는다.
- full `openpilot.tools.lib.logreader.LogReader` 및 전체 cereal schema를 사용한다. 축소된 opendbc CAN 전용 schema로 서비스 존재 여부를 판단하지 않는다.
- 보고서는 요청/가상조건 비교, 제출 RES 개수, SCC 상태, 경고 전환, 속도별 커브·풀림 통계, 접근/감속 시작 관측, 신호의 누락·노후화 및 branch/commit 혼합 여부를 보여 준다. 원인을 자동 확정하거나 패치를 선택하지 않는다.
- v2 보고서는 braking 보존 사유·관측 시각, 프레임 창의 최소 가속도, 모델/실측 곡률, CAN 토크 소프트웨어 반환값·실제 상한, 토크 기반 지속 관측 3종의 true/false/missing 표본을 별도로 집계한다. 보존 사유의 수는 고장 또는 충돌 건수가 아니다.
- 같은 초기 설정의 60초 session 재공지는 별도 횟수로 집계하고 연속 구간을 끊지 않는다. 초기 설정이 실제로 달라진 session은 이전 표본과의 연속 비교를 초기화한다.
- 타임라인은 기본 최대 200개이며 `--timeline-limit 0..200`으로 제한한다. 파일 간·2초 초과 샘플 공백을 연속 전환으로 추정하지 않는다. 구간 시간 기준은 해당 입력의 첫 진단 샘플이며 `initData`의 부팅 시각이 아니다.
- JSONL은 줄 단위로 읽고 한 줄 1 MiB를 넘으면 거부한다. rlog는 한 segment씩 전체 LogReader가 메모리에 읽으므로 원본 segment 크기만큼 분석 메모리가 필요할 수 있다.
- 표본이 없으면 종료 코드는 2다. 이것을 “문제 없음”으로 해석하지 않는다. 품질 집계와 원본 서비스 존재 여부부터 확인한다.

## 배포 날짜 표시

설치본 루트의 `dk_release.json`은 `schema=1`, `deployed_at="YYYY-MM-DD HH:mm KST"`, 양의 정수 `diagnostics_version`을 가진다. 배포 준비 시 저장소에서 갱신하는 릴리스 표식이며, 실제 GitHub 푸시 시간을 자동 조회하는 기능이 아니다. comma3x 주행 UI는 설치된 파일을 시작할 때 한 번 읽어 `dkcarrot-wip`에서만 좌상단 날짜 아래 `MMDD 개선`(이번 배포: `0912 개선`)을 표시한다. 날짜 글자 크기 60의 80%인 48을 사용하며, 기존 `DK 배포 YYYY-MM-DD` 문구는 중복 표시하지 않는다. 현재 날짜, Git 커밋 날짜, AGNOS 업데이트 날짜 또는 그 장치의 설치 완료 시각이 아니다.

메타데이터가 없거나 형식이 잘못되면 임의 날짜를 대신 표시하지 않는다. 별도 설정은 없다. 날짜를 숨기고 시간만 표시하면 시계 아래에, 시계·날짜를 모두 숨기면 좌상단에 개선 표식만 표시한다. 이 표시와 로컬 테스트 통과만으로 실차 문제 해결·디바이스 업데이트 완료를 판정하지 않는다.
