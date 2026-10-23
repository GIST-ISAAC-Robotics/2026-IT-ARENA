# C1 발행 배열과 운동 보정의 연결 대조 v1

2026-10-23. PC 합성 17조건과 고정 제조사 C++ 함수의 제한 실행이다. 실물/ROS/Gazebo 주행, 드라이버 설치, 운영 설정 변경은 없다.

- [report.json](report.json): 조건·원래 점 ID 대조·오차·체크. 17조건 검사 통과, 입력 소스와 제조사 원문 해시 불변.
- [source_manifest.json](source_manifest.json), `source/`: 시험 당시 분석기/보정기/시험 코드. 마지막에 추가한 방위축 회귀는 아래 `checks_v2/`에 별도 보존한다.
- `*.input`, `*.stdout`, `*.stderr`, `*.exit`: 실제 C++ probe 입력/출력. `*.npz`: 합성 원래 점·취득 시각·진실 교차점·발행 배열·평가용 원본 인덱스.
- [compile.stderr](compile.stderr), [compile_exit.txt](compile_exit.txt): GCC/UBSan 컴파일 exit 0, stderr 없음. 각 실행 stderr도 비어 있다.
- [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md): 공개 원문 함수와 라이선스. 원본 캐시는 이전 `sllidar_contract_v1/upstream`이며 분석기는 네트워크/장치를 열지 않는다. 알려진 SHA와 일치하지 않으면 컴파일 전에 거절한다.

재현(WSL, 출력 디렉터리는 새 이름이어야 함):

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=src/arena_autonomy:scripts \
python3 scripts/audit_c1_motion_pipeline.py \
  --upstream artifacts/validation/2026-10-23/sllidar_contract_v1/upstream \
  --output artifacts/validation/2026-10-23/c1_motion_pipeline_new
```

처음 관련 회귀는 71통과/2.05초(exit 0)였으며 원시 터미널 출력만 남았다. 이후 방위축 회귀를 추가한 [checks_v2](checks_v2/)는 stdout·소스 snapshot·해시를 별도로 보존한다. 이전 17조건 산출물을 새 시험 소스로 덮어쓰지 않는다.

자세한 해석은 [C1 시각 계약 §6](../../../../docs/sensors/C1_DRIVER_TIME_CONTRACT_2026_10_23.md#6-움직이는-장면에서-제조사-발행과-보정을-연결한-대조)에 모았다. 진단 검사 통과는 잘못된 시간 보정의 합격이 아니며, 알고리즘/시계의 실차 검증도 아니다.
