# r409 저장 설정 감사: 표기 차이와 주행 실패 분리

r406의 원 `runtime_audit.json`은 `header_exact=false`다. 유일한 차이는 `follow_config.known_structure_sectors_json` 문자열 안의 JSON 공백이다. 새 감사는 이 필드만 엄격하게 해석하여 값·형식·배열 순서가 같음을 확인했다. 다른 header는 그대로 비교하며 중복 키·비유한 수·깨진 schema를 거절한다. 원 기대값·trace·감사·주행 결과를 덮어쓰지 않았다.

Windows 최초 실행은 pytest 미설치로 실행되지 않았으며 `verify_v1`에 보존했다. 기존 WSL 환경에서 순수 회귀 **16통과** 뒤 저장 감사1회를 실행했다(`verify_wsl_v1`). ROS·Gazebo·Jetson은 시작하지 않았다. [후속 감사](report.json)에 입력4개와 감사 스크립트 SHA256을 기록했다.

결과는 설정 의미 동등·trace 정상 종료/기록오류0·후보 부모 발행1건이다. **독립 보호층/MCU 적용이나 실제 주행 개선의 증명이 아니다.** 원 감사의 `candidate_effect_demonstrated=true`는 부모 발행만 셌던 모호한 필드이므로 후속에서는 발행 관측과 주행 성과를 구분했다. r406의 미완주·정체·원 평가 실패는 그대로다.

검사 예:

```bash
python3 -m pytest -q -p no:cacheprovider artifacts/validation/2026-11-13/runtime_config_audit_r409_v1/test_audit_saved.py
```

`audit_saved.py --source <r406 폴더> --output <새 결과 경로>`는 저장 입력만 읽으며 기존 결과 파일을 덮어쓰지 않는다. 동일 자료를 다시 실행할 필요는 없다.
