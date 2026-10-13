# G1 trace 원천 시각 감사 최초판 — 의미 오류 보존

관련 17회귀는 통과했지만, 첫 감사기는 비동기 trace의 pose 시각을 마치 모든 채널의 수신 시각처럼 취급해 MCU 상태 +1 ms 3건을 원천 오류로 표시했다. `validate_g1_traffic.py`의 `now = pose['time']`를 확인한 결과 이 추론은 성립하지 않는다. 파일/스냅샷/출력은 그대로 보존하며 [v2](../g1_trace_timing_v2/README.md)에서 원문 복사 일치와 채널 시각 차이를 분리했다.

`report.json`의 `raw_source_consistent=false`를 실물 MCU 시계 고장 증거로 사용하지 않는다. 실행 담당은 shell exit 0을 보고했지만 해당 소스는 이 결과에서 Python exit 2를 반환하도록 되어 있어 최초 실제 Python exit 메타데이터는 미확인으로 남긴다. 후속은 `set -eo pipefail` 실행 파일을 사용했다. 기존 G1 주행/사후 HOLD 감사 결과는 변경하지 않았다.
