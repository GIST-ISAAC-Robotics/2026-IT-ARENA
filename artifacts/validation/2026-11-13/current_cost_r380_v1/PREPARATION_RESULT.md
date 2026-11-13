# r380 로컬 준비 확인

2026-11-13. 원격 실행 없이 준비를 마쳤다. 기존 r375/r377 및 공통 소스는 수정하지 않았다.

- `build_bundle.py`: 266파일·83,731,976바이트 ZIP, 각 압축 항목 SHA256 일치. 명시 후보 7단계와 11/12 r266 `candidate_recheck/` 8파일을 포함한다.
- 번들 SHA256: `122f13672bdac590285bc4bf4bc15be0341158dac0849d9f65a7c916c8530a44`.
- 원 폴더와 별도 `local_extract/`에 푼 번들 각각에서 Python import 사전 검사 통과. 전체 266파일 해시·7단계 실제 import 경로 일치, rebase 호출 0회.
- `local_extract/`는 해당 폴더의 `.gitignore`로, 원시 JSONL/번들 ZIP은 기존 저장소 규칙으로 Git 제외를 확인했다. 원시 센서 입력과 상세 판단 행을 공개 요약으로 복사하지 않는다.
- Jetson/WSL 접속·native 빌드·계측·센서/주행 실행은 0회다. `native_v1`의 실제 원격 import와 계산 시간은 실행 담당 배정 뒤 확인한다.

근거: [번들 영수증](bundle_receipt.json), [전파일 manifest](source_manifest.json), [원 경로 import](local_import_preflight.json), [독립 추출 import](isolated_import_preflight.json), [실행 인계](README.md).

독립 추출 import는 설치 환경과 분리된 **경로 완결성** 확인이다. Jetson 네이티브 ABI·성능 확인으로 확대하지 않는다. 결과에 시간 수치를 적는 것은 이후 `jetson/report.json`을 회수한 뒤다.
