# r536 단1회 재생 helper

**준비본이다. Jetson 재사용 전달 전에는 실행하지 않는다.** Windows에서는 fixture/AST와 `--help`만 검증했다. ROS·ARM 성능은 미측정이다.

root가 배정한 최신 설치의 activation을 먼저 적용하고, 이 `stage/`와 원 r523 `reconstructed_inputs.json.gz`를 별도 시험 경로에 둔다. `--installed-root`는 `source_manifest.json`과 `install/`을 포함하는 최신 bundle root다. 설치 manifest의 SHA를 명시하고 새 결과 디렉터리 하나를 사용한다.

```bash
ROS_DOMAIN_ID=158 ROS_LOCALHOST_ONLY=1 python3 -B /test/stage/run_once.py \
  --installed-root /selected/latest-bundle \
  --source-manifest-sha256 MANIFEST_SHA256 \
  --profile /test/stage/profile.compact.json \
  --input /test/reconstructed_inputs.json.gz \
  --output /test/result_once \
  --dds-library /selected/libarena_dds_probe_v1.so \
  --execute-once
```

위 경로는 선택값 자리표시자다. 도구가 SSH·설치·activation을 대신하지 않는다. 기존 r523의 프레임/토픽·20초 입력·계약 검사·합성 RGB/detector·STOP/자식 회수 절차를 유지한다. SHM16은 이 프로세스와 자식 환경에만 적용한다. DDS probe SHA는 r523과 동일한 고정값을 요구한다.

`profile.compact.json`은 기존 replay profile에 **명시 `g1_trace_detail=compact_v1`**만 더했다. 외부 profile도 받지만 compact/complete-plan-trace-off/원 simulation frame 계약이 다르면 시작하지 않는다. trace 경로만 새 결과 폴더로 지정한다. source manifest·입력·stage hash를 확인하고 기존 결과 폴더가 있으면 거절한다.

`observation.py`는 원 control/collect/prepared/publish/ack/guard callback을 각각 한 번 호출한다. 별도 drain, proof bootstrap, TraceOff 교체가 없다. 원 header의 정수 ns와 전달 token을 작은 scalar로 메모리에 보관하고 종료 후 기록한다. 기존 status·TimingProbe를 유지하며 입력 전체를 복제·기록하지 않는다.

핵심 결과는 `summary.json`의 양수 최대 공백·0목표 총시간/구간·원 명령 stale 나이와 current 회수→permit→publish 경계다. `scalar_guard.jsonl`의 원 command 수신으로 guard 전달을 구분한다. `start_receipt.json`은 시작 한 번, `execution.json`은 자식 회수·원 소스 불변이다. scalar drop/error가 있으면 완전한 종단 판정에 사용하지 않는다. helper 자체 PID0은 종료 후 root가 확인하며 파일 내부에서 자기 종료를 증명했다고 표시하지 않는다.
