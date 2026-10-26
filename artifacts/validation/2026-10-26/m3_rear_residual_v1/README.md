# M3 후방 가능 영역: 정보 한계와 기록 장부 보수성의 분리 (10/26 첫 묶음)

**최신:** GPT 인계 뒤 run7/8에서 캐시 수정판을 재실행했다. [전후 비교](cachefix_comparison.json)의 해제 시각·속도·조향은 동일하고 진단 1행만 달라졌다. 원래 [HANDOFF.md](HANDOFF.md)는 인계 당시 기록으로 보존한다. 현재 설계 판단은 [E3 §8.28](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#828-1026-gpt-인계-검토--전제-채택-보류와-실제-가림의-영향)을 따른다. run9 순차 비용은 미실행이다.

해석과 결정 항목은 [E3 §8.27](../../../../docs/autonomy/MULTI_ACTOR_E3_2026_10_24.md#827-1026--후방-가능-영역-정보-한계와-기록-장부-보수성의-분리)에 있다. 이 폴더는 실행 근거만 둔다.

**범위:** 저장 판단 입력(run17/21/19/32/35)의 개루프 재호출과 이동 자차 합성 장면. 운영 소스·노드·launch·기본값·보호층·STOP·간격/물성은 바꾸지 않았고 ROS/Gazebo 주행은 없다. 정답 자세·actor 위치는 광선 생성(합성)과 사후 평가에만 썼다. 아래 시제품은 이 폴더 안의 미연결 모듈이다.

## 스크립트

| 파일 | 내용 |
|---|---|
| [`decompose_release_residual.py`](decompose_release_residual.py) | 현행 `release_gate`를 관찰만 하여 미확인 칸을 원인(마스크·무응답·가림·광선쌍 무효)×위치(후면 평면 뒤/앞)로 분해. 관찰/비관찰 출력 해시 동일 확인 |
| [`release_premises.py`](release_premises.py) | 전제별 관문(`ReleasePremise`: P-follow·P-size·P-base·만료 무시 진단)과 측면 합집합 상태기 `UnionReleaseTrial`. 모든 전제 False면 현행과 같은 계산 |
| [`compare_release_premises.py`](compare_release_premises.py) | 저장 입력에서 판본별 전체 출력·명령·해제·비용 비교. `strict`=현행 전체 출력 동일을 단언 |
| [`analyze_run2.py`](analyze_run2.py) | run2의 명령 단위 대조와 해제 시각 사후 actor 위치 |
| [`synthetic_world.py`](synthetic_world.py), [`counterexamples.py`](counterexamples.py) | 광선별 취득 시각에 자차·물체를 두고 운동 보정 원점을 만드는 직선로 합성과 반례 17개 불변식 |
| [`measure_tail_band.py`](measure_tail_band.py) | 저장 장면의 마스크 경계 광선이 후면 평면을 지나는 횡위치로 꼬리 띠 폭 측정 |

## 실행과 결과

| 실행 | 명령 | 결과 |
|---|---|---|
| [run1_decompose](run1_decompose/) | [`run1_decompose.sh`](run1_decompose.sh) | exit 0, 5입력 관찰 출력 차이 0·입력/소스 불변. 위협 존재 구간 차단 칸 대부분 `mask_behind` |
| [run2_premises](run2_premises/) | [`run2_premises.sh`](run2_premises.sh), [plan](run2_plan.json) | exit 0, 5입력×10판본. `strict`=현행 전체 출력 동일. [`run2_analysis.json`](run2_analysis.json)(명령 대조·첫 해제 사후 위치), [`run2_second_release_posthoc.json`](run2_second_release_posthoc.json)(run32/35 두 번째 해제 때 잔디 옆차가 보이는 상태로 옆 뒤쪽 0.185 m). 병렬 실행이라 비용은 참고값 |
| [run3_counterexamples](run3_counterexamples/) | [`run3_counterexamples.sh`](run3_counterexamples.sh) | exit 0, 9사례×6판단기, 불변식 17/17 |
| [run4_cost](run4_cost/) | [`run4_cost.sh`](run4_cost.sh), [plan](run4_plan.json) | exit 0, 단일 프로세스 순차 비용(실행 전 부하 [기록](run4_load_before.txt)). Claude Haiku 5.5 보조가 실행·회수(자체 보고 `claude-haiku-5-5`) |
| [run5_syntax_error](run5_syntax_error/) | — | **실패 보존.** 임시 스크립트를 옮길 때 `'\n'`이 실제 줄바꿈으로 바뀌어 구문 오류(exit 1), 계산 전 중단 |
| [run6_tail_band](run6_tail_band/) | 스크립트 사본 `script.py` | exit 0, 꼬리 띠 폭 표본 |
| [run7 수정판 저장 재생](cachefix_runs/run7_premises_cachefix/summary.json) | 기존 run2 계획·8프로세스 | exit 0, 5입력 6,165판단 엄격/현행 동일. 다른 전제의 명령·해제 시각도 수정 전과 같음 |
| [run8 수정판 합성 반례](cachefix_runs/run8_counterexamples_cachefix/summary.json) | 기존 run3 조건·12프로세스 | exit 0, 17불변식 통과 |
| run9 순차 비용 | — | 미채택 후보의 추가 비용 재측정은 보류. 결함판 비용은 과거 근거로만 보존 |
| [run10 전체 형태 반례](cachefix_runs/run10_shape_review/result.json) | `review_geometry.py` | **exit 1:** 우회전 겹침 예상 실패. 전체 형태의 스캔 동일성과 반대 좌회전 겹침은 원본에서 확인. 후처리 그림은 별도 exit 0 |
| [run11 가림 분리](cachefix_runs/run11_visibility_strict/summary.json) | `review_visibility.py` | exit 0, 24조합 성격 확인. 실제 장착 변경이나 주행 완료 아님 |
| [run12 완전 시야의 거절 분해](cachefix_runs/run12_full_view_audit/result.json) | `audit_full_view.py` | exit 0, 좌우 원래 출력 동일·오른쪽의 광선 배열 경계 거절 확인 |
| run13 원문 시각 반례 | `review_scan_seam.py` 이전 판본 | **exit 1:** `source_first_s` 불일치 fixture 오류, 계산 전 실패 원문과 소스 보존 |
| [run14 배열 경계 분리](cachefix_runs/run14_scan_seam_fixturefix/summary.json) | `review_scan_seam.py` fixture 수정판 | exit 0, 18조건. 완전 시야 좌우 6.6초 복귀, 현 마스크 4사례 출력 동일, 숨은 물체 유지 |
| [run15 원문 순환 계약](cachefix_runs/run15_cycle_contract_regression/stdout.txt) | `test_scan_cycle_candidate.py` 초기 판본 | exit 0, 25검사. 기본 미연결·시각/마스크/거리 여유 보존 |
| [run16 원문 계약·반환 종류](cachefix_runs/run16_cycle_contract_complete/stdout.txt) | 같은 회귀 보충 | exit 0, 30검사 |
| [run17 원문 계약 통합](cachefix_runs/run17_cycle_metadata_comparison/summary.json) | `verify_cycle_integration.py` | exit 0, 16대조가 run11 OFF/run14 ON과 전부 동일 |
| [run18 물체 간 분리 배치](cachefix_runs/run18_disjoint_shape_review/result.json) | `review_geometry.py` 수정 배치 | exit 0, 통과차와 횡 5 mm 분리. 후방 가림에 숨는 전체 단면·좌회전 표본 겹침. 도식 후처리 exit 0 |
| [run19 float32 각도 계약](cachefix_runs/run19_cycle_float32_contract/stdout.txt) | `test_scan_cycle_candidate.py` 보충 | exit 0, 39검사. 한바퀴 누적 반올림 허용·광선 누락/중복 끝점 거절 |

호출별 재생 행과 상세 `rows.json`은 기존 상세로그 정책에 따라 로컬 보존·Git 제외한다. `summary.json`, 실패 판정, 원본 해시, 실행 스크립트와 소스 ZIP은 보존한다. [결함판 복원 ZIP](sources_original_cache_defect.zip)은 run2/4 기록 해시와 정확히 같고, [GPT 검토 시작 소스](sources_gpt_review_v1.zip)와 [형상 시험 실패 소스](sources_geometry_failed_right_prediction.zip), [시각 fixture 실패 소스](sources_seam_fixture_failure.zip), [run15 계약 소스](sources_run15_contract.zip), [run16–17 계약 후보](sources_cycle_candidate_v1.zip)는 별도다.

최종 float32 보충·분리 배치 소스는 [마감 소스 80파일](sources_gpt_wrap_v2.zip)에 파일별 SHA-256과 함께 보존했다. [마감 검사](wrap_audit.json)는 Python 구문·새 문서의 상대 경로·운영 M3 소스 불변을 확인한 결과이며 추가 주행시험이 아니다. 이후 재개는 [GPT 인계](GPT_HANDOFF.md)를 따른다.

## 개발 중 스모크와 버린 판정식

정식 실행 전 `~/arena_scratch`에서 run17 스모크를 여러 번 돌렸고 정식 근거로 쓰지 않는다. 처음의 꼬리 띠 크기 판정(표본 근처에 원판이 T에 닿기만 해도 차단)은 후면 평면 바로 뒤 원판까지 세어 효과가 없었고, 다음 판본(표본 반경으로 후면 평면 근처를 판정)은 2 cm 격자 정렬에 따라 1–3칸이 깜박여 해제 시각이 표본화 운에 좌우됐다(run17 12.82 s). 최종 판본은 뒤 모서리 옆 영역 Z를 현재 장면에서 1 mm 격자로 직접 판정한다. 상한을 넓혀 기록별 장부를 돌린 탐색 진단도 호출당 p95 13–19 s로 실용성이 없어 중단했다(보존하지 않은 임시 출력).
