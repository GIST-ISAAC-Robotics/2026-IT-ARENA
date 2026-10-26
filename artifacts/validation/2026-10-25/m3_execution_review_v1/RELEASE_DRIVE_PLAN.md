# 기본 G1 복귀 옵션 — 반려된 주행 등록안 (미실행)

2026-10-25, run13 분해와 run14 저장 입력 대조 뒤 작성. **후속 GPT 검토에서 반려했다. 아래 명령은 실행하지 않는다.** 마지막 반환 위치와 두 번의 부재만으로는 탈락·가림과 통과를 구분하지 못한다. 후속 구현·반례는 `run15_release_gate/HANDOFF.md`와 `run16_release_gate_review/`에서 이어가며 이 안의 원래 조건·기대 결과는 역사로 보존한다. **GPT 독립 검토 전에는 실행하지 않는다.** run13 실패(진행·최종 간격, 옆차 약 69.8 mm)는 그대로 보존한다.

## 질문과 변경

run13에서 옆차가 앞으로 지나간 뒤에도 측방 시험이 끝나지 않아 0.35 m/s 상한과 약 0.7 s 정지-출발 주기가 계속됐다. 원인은 후방 마스크 경계의 꼬리 흔들림 띠 때문에 주행 중 곡선 명령이 지지되지 않고, 시험 해제가 중앙 후보 완료(횡위치 5 mm 이내)에만 묶인 것이다([분해](run13_analysis/README.md)).

차이는 기본 false `--m3-release-to-baseline` **하나만** 켜는 것이다. 마지막으로 보인 옆차 반환이 모두 자차 앞면보다 앞에 있었고, 그 뒤 서로 다른 fresh 스캔 2회 동안 측방 위협이 없으면 시험을 끝내고 활성 전과 같은 기본 G1 출력으로 돌아간다. 뒤·옆에서 사라진 차는 후방 마스크에 숨었을 수 있으므로 해제하지 않는다. 관측·간격·보호 기준, 0.35 m/s 시험 상한(활성 중), 꼬리 띠 계약은 바꾸지 않는다.

이 옵션은 꼬리 띠 관측 문제를 해결하지 않고, 통과 뒤 중앙 복귀를 기본 G1에 맡겨 우회한다. 뒤 사각 미검사, 앞쪽 소멸과 탈락·가림의 미구별, 측정 여유 없는 0.10 m 임계, 해제·재활성 반복 가능성은 [E3 §8.10.3](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#8103-기본-false-release_to_baseline--상태기-해제-조건만-보완)의 검토 쟁점이다. 승인 시 임계 여유를 보강할지 먼저 정한다.

## 고정 조건과 명령

run13과 같은 `slow_lead`·seed 0·도로 내 왼쪽 옆차·`corner_distance_candidate`·`per_ray_candidate`·회랑 보완 ON·최근 관측 OFF·`--m3-command-fallback` ON. 차량·센서·물성·구동·속도·보호층·간격·재개 횟수·잔디 규칙 동일. 관련 회귀·빌드·설치본 확인(run14) 뒤, 소스가 run14와 같을 때 1회만 실행한다.

```bash
python3 -B scripts/validate_g1_traffic.py --case slow_lead --seed 0 --mode m3 --m3-command-fallback --m3-release-to-baseline --record-follow-decisions --record-braking-diagnostics --boundary-evidence corner_distance_candidate --occupancy-timing per_ray_candidate --corridor-fallback --actor-overlay config/tests/e3_onroad_encounter_v1.json --output artifacts/validation/2026-10-25/m3_execution_review_v1/run15_release_closed_loop/run
```

종료 뒤 판단 재생·노면 평가 각 1회. 결과 파일이 생긴 뒤 종료 영수증이 모호하면 기존 세션·로그로 회수하고 중복 실행하지 않는다. 실패해도 같은 설정을 반복하지 않는다.

## 판정 (사전 등록)

- 원본 평가기의 15 cm·차체 도로 내·진행 2.5 m·최종 앞차 간격 1.2 m 판정을 그대로 기록하고 덮어쓰지 않는다.
- 별도 M3 판정: 시험기·입력 무결성, 명시 STOP·밀집 정지, 정상 종료·잔존 0, 소스 불변, 판단 재생 일치, 모드 표식, 접촉 0, `surface_contact_eval` `any` 해석 3바퀴 이상 이탈 0·기타 지면 0·벽 0(두 바퀴 잔디 시간·3초 초과 여부 별도), 측방 최소 sampled/swept 간격을 0.097 m 가정 예산과 대조(미달은 실패로 기록).
- 해제 고유 확인: 해제 시각에 정답 축으로 옆차 뒤면이 자차 앞면보다 앞이었는지(사후 평가), 해제 뒤 출력과 같은 판단의 기본 G1 출력 일치, 재활성 횟수, 해제 뒤 옆차·앞차 간격. 해제가 일어나지 않으면 그 사유를 분해한다.
- 예상: 통과 뒤 정지-출발 주기 해소와 진행 증가. 옆차 최소 간격 약 70 mm(배치 평행 간격)는 이 변경으로 바뀌지 않을 것으로 예상하며 해결로 표현하지 않는다. 최종 앞차 간격은 기본 G1의 E3 결과(1.712 m 실패)처럼 여전히 실패할 수 있다.
- 시뮬레이션 실시간 비율 약 0.09에서의 결과이므로 PC 실시간 판단 비용·Jetson·실차 제동·후방 숨은 차량·전체 코스 성공을 입증하지 않는다.
