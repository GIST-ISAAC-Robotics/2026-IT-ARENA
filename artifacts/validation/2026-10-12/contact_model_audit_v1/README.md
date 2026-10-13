# 제동·접촉 모델 후처리 감사

10/12 `scheduled_surface_v1`의 두 실행을 **재실행하지 않고** 읽는다. 물성/구동/보호선과 원시 자료는 변경하지 않는다.

## 범위

- 실행 SDF의 질량·바퀴 관성·축·마찰 방향·15:1 토크 환산과 기록값을 대조한다.
- 0목표 적용 후 동일 시각의 힘/자세/바퀴 속도를 비교한다. 힘은 world 좌표이며 접촉 양쪽의 부호를 구분한다. 마찰은 원형 원뿔이 아닌 **방향별 피라미드 한계**로 검사한다.
- 접촉 누락/동시각 충돌은 0힘으로 채우지 않는다. 진단은 약 20 ms이므로 pre/post-physics 값을 섞어 에너지 보존을 단정하지 않는다.
- 평면의 접촉력 적분과 차체 수평 운동량 변화를 대조하되, 차체 기준점은 전체 질량중심이 아니므로 근사 진단으로 표시한다.
- 설치 버전과 일치하는 공개 upstream 소스에서 WheelSlip→Physics→DART 경로를 확인한다. 바이너리 재빌드/계측 없이 런타임 내부 파라미터를 직접 읽었다고 주장하지 않는다.

## 최소 추가 검사 조건

기존 자료에서 `|F_t| > mu*F_n`이 방향별로 나타나면, 설치 DART의 **작은 수학적 LCP 입력**으로 마찰 한계를 계산하는 순서를 검사한다. 이는 차량/월드 실행이 아니며 차량의 실제 solver 분기를 계측한 것도 아니다. 기본 solver나 차량 파라미터를 바꾸지 않는다. GPT가 분석·코드를 맡고 GPT-6 Luna가 검사 실행을 맡는다.

결과는 `analysis.json`, 실행/소스 근거는 `manifest.json`에 보존한다. 실패 산출물은 덮어쓰지 않는다. Gazebo 새 주행·G1 재시험·외부 게시·Jetson/전원 변경은 이 범위에 없다.

## 마감 결과와 재현

- 최종판은 [analysis_final.json](analysis_final.json), 설치 DART 작은 수학 시험은 [solver_probe.json](solver_probe.json), 관련 회귀는 [tests_final.log](tests_final.log)의 **16개 통과**다. `analysis.json`은 최초 후처리이며 [수정/실패 경위](ANALYSIS_REVIEW.md)를 함께 읽는다.
- 상세 해석은 [G1-b 보고서](../../../../docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md#1012-구동접촉-정합성-감사--큰-슬립과-접촉-계산-근사의-구분) 한 곳에 둔다. 마찰 한계 초과는 모델 합격이 아니며 결과 JSON의 무결성 checks와 구분한다.
- `capture_sources.py`/`capture_world_source.py`는 버전별 공개 원문과 SHA를 처음 수집한 도구다. 기존 폴더에 다시 실행하지 않는다. `.gitattributes`의 날짜별 `-text` 정책으로 원문 바이트를 유지한다.
- Windows–WSL 인자 확장을 피하기 위해 설치 라이브러리 시험은 `run_probe.sh`, 최종 마감 감사는 `run_finalize_v2.sh`로 분리했다. 기존 결과가 있으면 덮어쓰지 않는다. 임시 `/tmp` 실행 파일은 마감 시 남아 있지 않아 해당 바이너리 해시는 확보하지 못했다. 소스/configure/build/ldd/결과와 현재 설치 라이브러리 해시를 보존하되 과거 실행 파일 보존으로 확대하지 않는다.

저장소 루트에서 새 출력 파일로 후처리만 재계산할 수 있다. 원시 JSONL은 기존 Git 제외 정책에 따라 로컬에 필요하다.

```bash
python3 -B scripts/analyze_contact_model.py \
  --base artifacts/validation/2026-10-12/contact_model_audit_v1 \
  --output /tmp/contact_model_audit_new.json
python3 -m pytest -q tests/test_contact_model_analysis.py tests/test_scheduled_surface_analysis.py
```
