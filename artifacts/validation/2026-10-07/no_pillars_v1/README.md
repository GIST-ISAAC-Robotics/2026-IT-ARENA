# 기본 코스 임시 기둥 제외와 짧은 비교 — 2026-10-07

사용자 결정에 따라 기본 `track:=official`에서 팀이 추가한 신호등 바닥 기둥/발판 `tl_post`와 마커 지지 링크 `team_gantry_support_{0,20,30,45}`를 제거했다. 공식 원본 ZIP은 그대로이며, 실제 설치 형상은 제작 사진/도면 공개 후 다시 확인한다. **현실에도 기둥이 없다는 확인이 아니다.**

## 보존과 재현

- `before_supports.zip`: 제거 직전 기본 월드 전체·공식 파생 프로필·생성기 2개. SHA-256 `3790b410c6c8428c8e24fbc1ad427aeb9bf888caada0ee828b5f08059be51fb4`. 당시 실패 및 성공 기록과 함께 기둥 스트레스 사례로 보존한다.
- `experimental_before.zip`: 공유 생성기 입력 해시 갱신 전의 옛 실험 파생본 `world.sdf`·`scene.json`·`provenance.json`. 옛 실험 프로필에는 기둥을 계속 생성하며, 이번 재생성의 형상 변경은 없다.
- `audit_geometry.py` / `geometry_audit.json`: 정확히 5개 링크(충돌체 29개)를 제외한 XML 전체 동일성, 지도·메시·PNG·중심선 등 27개 파일 바이트 동일성, scene의 공식 형상/마커/그리드 메타데이터 동일성 검사.
- `build_official.log`, `build_experimental.log`, `pytest_geometry.log`: 생성 및 관련 회귀 기록. 실행본은 install symlink가 현재 source 월드를 가리키는 것을 확인했다.

기둥 스트레스 사본을 다시 만들려면 **현재 기본 프로필을 되돌리지 말고** 보존 ZIP의 `official_v2026.09.14.yaml`만 별도 경로로 추출한 뒤 아래처럼 다른 출력 폴더를 지정한다. 이 프로필은 `gantry_supports.enabled: true`, `traffic_light.ground_post_enabled` 생략(기존 기본 true)이므로 옛 지지 형상을 재생성한다. 저장 ZIP 자체를 직접 펼치면 당시 월드·자료를 그대로 사용할 수 있다.

```bash
python3 scripts/build_official_track.py --profile <별도_추출한_프로필.yaml> --destination build/pillar_stress_world
```

## 시험 범위

`before_g1_slow_s0` / `after_g1_slow_s0`: 동일 G1 제어기 `fddc808b…`, 평가기 v4, 1.0 m/s 상한·등속 앞차 0.35 m/s·시작 직선·seed 0. 기존 실행기를 사용했으며 출력별 source snapshot과 runtime world를 보존한다. 제거 전후 1회씩의 기능 비교이지 통계적 우열/고속/곡선/전체 랩/실차 보증은 아니다.

두 실행 모두 유효·행동/정지/종료·사후 감사 통과, 앞차 이동 중 정체 0초, 진행 4.939 m, 접촉·노면 이탈 표본 0이다. 제거 전/후 최소 간격은 0.588/0.581 m, 끝 간격은 0.651/0.644 m였다. 두 source snapshot의 차이는 `world.sdf`와 `scene.json`뿐이다. 관련 형상/경계 회귀 115개·계측/평가 회귀 69개 통과(서로 별도 검사 집합).

별도 `previous_braking_analysis.json`은 기존 v3/v4 R0 정지 앞차 실행의 감속 구간 비교다. `scripts/analyze_g1b_braking.py`로 MCU 목표·모터축 환산 속도와 차체 XY 위치 차분 속도를 대조한다. 엔코더와 차체의 차이 자체로 마찰계수/접촉 솔버 등 최종 원인을 단정하지 않는다. 후속 상세 결과의 기준 문서는 [G1-b 보고서](../../../../docs/autonomy/LOCAL_FOLLOW_G1B_2026_10_07.md)다.

원시 고빈도 기록은 로컬에 보존하고 Git 제외한다. 기존 실패 판정·과거 실행 출력은 덮어쓰지 않는다.

## 후속 제동 시험

- `after_r0_stopped_s0`·`after_r0_stopped_s0_repeat`: 기둥 없는 동일 조건 R0 정지 앞차 두 회. 실행·정지·정상 종료는 유효, 접촉은 0이지만 최소 간격 0.100599/0.088727 m로 G1의 0.15 m 조건에는 미달한다. `passed=true`를 G1 행동 합격으로 해석하지 않는다.
- `after_g1_stopped_s0`: G1 정지 앞차. 최소 0.386283 m·HOLD 6.472초, 행동·정지·종료·사후 감사 통과.
- `--record-braking-diagnostics`를 켜서 R0 두 회와 G1 정지 시험에서 원천 시각·휠/차체/토크를 추가 기록했다. 제어/판정 변경은 없다. 원시 `braking_diagnostics.jsonl`은 Git 제외하고 해시는 report에 남긴다.
- `repeated_braking_analysis.json`: R0 두 회의 감속 시작 후 0.5초 분석. 첫 실행/반복의 최대 평균 휠–차체 차이는 0.0086/0.7011 m/s, 최대 좌우 타이어 표면 속도 차이는 0.0243/1.4061 m/s였다. 계측상 운동 차이이며 실제 원인/실차 고장을 확정하지 않는다. `first_no_pillar_braking_analysis.json`은 진단 요약 기능 추가 전 첫 실행 분석이다.
- `final_audit.py`·`final_audit.json`: 다섯 실행의 소스·원시 SHA·주행/정지/종료 구분 검증. `run_final_checks.sh`는 추가된 형상/분석 검사와 기존 v8 회귀를 같은 집합으로 실행한다.

최종 `pytest_final.log`: **517 passed, 46.59초**, `pytest_final.exitcode=0`. 기존 115/69 검사와 합산하지 않는다. 모든 시험 프로세스는 종료했으며, 이전 R0 접촉의 최종 물리 원인은 미해결로 남긴다.
