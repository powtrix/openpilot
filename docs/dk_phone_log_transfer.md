# DK 수동 휴대폰 로그 전달 / Manual phone log relay

## 목적과 범위 / Scope

`dkcarrot-wip`의 진단 보관 자료를 휴대폰의 다운로드 폴더에 저장한 뒤 집의 컴퓨터로 전달한다. 별도 Android APK, 백그라운드 권한, 자동 실행, 외부 서비스, 포트포워딩은 필요하지 않다. 차량 제어, Params, 원본 rlog, 기존 자동 업로드 설정을 변경하지 않는다. 인터넷 서버로 보내는 기능이 아니다.

This is a foreground, user-initiated file relay. It does not install a phone app, run background services, upload to an external server, change controls/Params, or remove original logs. No external port forwarding is required.

현재 대상은 `/data/media/0/dk-diagnostics`의 **자동 선별된 진단 캡처**다. 최대 10건·1 GiB·7일 보관이며, 용량/건수에 따라 더 빨리 교체된다. 전체 주행 백업이 아니고 영상/qcamera는 포함하지 않는다. 선택된 캡처의 원본 full rlog와 원본 manifest를 보존한다. manifest에는 기록 당시 수집된 session의 브랜치·커밋·설정 등과 누락 이유가 들어간다. `ready`는 파일 확보 완료, `partial`은 일부 구간 누락이며 차량 정상 여부를 뜻하지 않는다. `pending`과 확보된 파일이 없는 캡처는 받지 않는다.

The payload is retained diagnostic-window **full rlogs and their original manifests**, not all routes or video. Session metadata reflects the recorded session, not a fresh settings read. Partial captures remain explicitly partial. Missing evidence cannot be recovered by transfer.

## 차량에서 받기 / Save in the car

1. 안전하게 정차한 뒤 comma를 본인의 갤럭시 핫스팟에 연결한다. 휴대폰 브라우저에서 해당 디바이스의 웹당근이 열려야 한다.
2. 웹당근 **Logs / 로그 → dk 로그전달**을 누른다. 직접 주소는 `http://<디바이스 IP>:7000/dk-logs`이다. 고정된 IP를 가정하지 않는다.
3. 기록 시각, 주제, 확보/누락 상태와 대략적인 용량을 확인하고 **선택 로그를 휴대폰에 저장**을 누른다.
4. 서버가 묶음을 준비하면 브라우저가 `dk-logs-<transfer_id>.dklog.zip`을 다운로드한다. 페이지의 ‘요청’은 성공 확인이 아니다. 브라우저 다운로드 목록에서 완료를 확인할 때까지 comma 전원과 핫스팟을 유지한다. 중단된 다운로드는 다시 시작한다. 첫 버전은 바이트 단위 이어받기를 제공하지 않는다.

On your own hotspot while parked, open Carrot Web → Logs → dk Log transfer, select captures, and save the ZIP. Keep power/network until the browser confirms completion. No JS Blob buffers the archive in phone memory. Interrupted exports must be restarted.

## 집에서 받기 / Receive at home

맥에서는 `tools/car_porting/dk_log_receiver.command`를 더블클릭해 수신기를 시작할 수 있다. 저장소의 `.venv` Python을 사용하며 현재 기본 네트워크의 사설 IPv4로 실행하고 수신 페이지를 연다. 키가 포함된 휴대폰용 주소를 페이지에서 복사해 사용할 수 있다. VPN 때문에 주소를 찾지 못하면 아래 직접 실행 방식을 사용한다. Python 3.11 이상이 필요하다.

On macOS, double-click `tools/car_porting/dk_log_receiver.command` to start the receiver and open its page. It uses this repository's `.venv` Python and the current default network's private IPv4. If a VPN prevents address discovery, specify the LAN address manually below. Python 3.11+ is required.

저장소 루트에서 맥미니의 실제 사설 LAN 주소로 수신기를 실행한다. `<맥미니 LAN IP>`는 휴대폰과 같은 집 네트워크에서 접근할 수 있는 주소로 바꾼다. NAS에서 사용하려면 Python 실행 환경과 이 소스 모듈을 별도로 준비해야 하며, 현재 NAS에 설치됐다고 가정하지 않는다.

```sh
.venv/bin/python tools/car_porting/dk_log_receiver.py --bind <맥미니 LAN IP> --port 8766 --output local_comma3x/dk-phone-inbox
```

1. 출력된 `http://<맥미니 LAN IP>:8766/#token=...` 주소를 집 Wi-Fi에 연결된 휴대폰에서 연다. 토큰이 포함된 주소는 외부에 공유하지 않는다.
2. 다운로드 폴더의 `.dklog.zip`을 선택하고 수신 페이지의 전달 버튼을 누른다.
3. **업로드 진행률 100%와 검증 완료는 다르다.** 파일명/크기/검증값과 manifest 검증을 마친 수신 완료 응답을 확인한다. 손상·불완전 자료는 완료로 처리하지 않는다.
4. 수신된 자료는 지정한 폴더의 transfer ID 디렉터리에 보관한다. 수신 확인 전에는 휴대폰의 ZIP을 삭제하지 않는다. 같은 transfer ID/내용을 다시 보내도 기존 결과를 덮어쓰지 않는다.
5. 수신기는 실행 중에만 동작한다. 터미널에서 Ctrl+C로 종료한다. 상주 서비스나 부팅 자동 실행을 설치하지 않는다.

Start the receiver on an explicit private LAN IP, open its token-bearing link on the phone, choose the downloaded file, and wait for **verification success**, not only 100% network progress. Keep the phone copy until receipt. The receiver runs only while explicitly started; stop it with Ctrl+C.

수신 후 분석 요청 시 `local_comma3x/dk-phone-inbox` 아래의 최신 완료 자료를 확인하면 된다. 분석에는 기존 full cereal 스키마를 사용한다. 수신기는 원인 분석이나 코드 수정을 자동으로 실행하지 않는다.

여기서 검증 완료는 묶음 구조와 복사 무결성(SHA-256)이 확인됐다는 뜻이다. 원본 rlog 전체의 디코딩 성공이나 차량 정상 판정을 뜻하지 않는다. Verification confirms archive structure and copy integrity, not full rlog decoding or vehicle safety.

## 안전·개인정보 / Safety and privacy

- 로그에는 위치, 장치 식별정보 등 개인정보가 들어갈 수 있다. 이 ZIP은 암호화된 보관 형식이 아니다. 본인만 사용하는 기기/다운로드 폴더에 보관한다.
- 초기 LAN 수신기는 HTTP다. 토큰은 접근 제어일 뿐 전송 암호화가 아니다. 본인의 신뢰할 수 있는 핫스팟/집 Wi-Fi에서만 사용하고, 공유기에서 포트를 공개하거나 공용 Wi-Fi에 노출하지 않는다.
- 수신기는 매 실행마다 임의 토큰을 만들며 주소의 fragment로 전달한다. 업로드는 토큰·동일 출처·실제 로컬 접속자·Host 검사 후 허용한다. 외부 자산, 외부 업로드 주소, 자동 클라우드 전송을 넣지 않는다.
- 차량 묶음 생성/다운로드는 한 번에 하나다. 보관 정책의 5 GiB 여유 공간 외에 최대 묶음 크기를 확보할 수 있어야 한다. 부족하면 기존 로그를 지우지 않고 실패한다.
- 수신기는 크기/개수 제한, 여유 공간 확인, 임시 저장, 검증 후 완료 디렉터리 확정을 적용한다. 경로 탈출, 링크, 중복 멤버, 허용 목록 밖 파일과 손상된 검증값을 거부한다. `extractall`을 사용하지 않는다.
- 이 구현으로 실차 수집의 완전성이나 다섯 주행 증상의 해결이 증명되는 것은 아니다. 갤럭시에서 핫스팟 접근 및 다운로드/파일 선택이 실제로 되는지는 최초 1회 확인해야 한다.

ZIPs and the initial LAN HTTP transport are not encrypted. Tokens do not provide encryption. Use only trusted private networks, never public port forwarding. Resource limits and source-preserving error handling are intentional. The relay does not prove symptom resolution or completeness of the original recording.
