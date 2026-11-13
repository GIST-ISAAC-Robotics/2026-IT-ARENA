# r410 경계 이웃 후보: 검사 통과, 주행 개선 미확인

WSL에서25개 순수검사와 같은 고유 저장 입력254개의 baseline/candidate 대조를1회 수행했다. 최초37계획의 원 출력이 같고 최초 직접대상 이전 출력·관측 상태 차이는 없었다.233동결입력/manifest 불변,32.55초·exit0·강제종료/잔여process group0을 확인했다. Terra 실행 담당이 시작 전 model capacity 오류로 종료하여 GPT 주 에이전트가 실행했다.

crop 때문에 실제 finite 이웃만 잃은 전체31입력에서 기존 나머지 검사로 넘어갈 수 있게 됐다. 그러나 reference25초 이후 고유88입력의 양수 계획은 **2→1**로 늘지 않았다. 원 `follow_unbounded_missing`77개는50개로 줄었지만 `follow_partial_history_expired`24개와 `follow_known_obstacle_latched`11개 등이 나타났다. 유효 관측을 기억하지 않고 early return한 뒤 오래된 history에 도달하는 문제가 남으므로 이 수정만 공통 코드에 채택하거나 새 주행을 반복하지 않는다.

직접대상 이후 전체출력45개와 base partial 상태105개가 바뀌었다. 바깥 unknown이 계속되는 입력의 출력 변화5개도 이전 상태 변화와 함께 남겼으며, 해당 경계 자체를 새로 허용한 것으로 해석하지 않는다. 첫37개 이후에는 원 자식 계산이 누락돼 원 결과와48개가 다르다. 이는 같은 저장 입력 순서의 제한 대조이고 원 차량 궤적 재현·실제 보호층 통과·주행 개선은 아니다.

근거: [보고서](verify_native_v1/report.json), `comparison_rows.jsonl`, 양쪽 전체 output/state JSONL, [실행 영수증](execution_v1.json). 기존 실패·staged 파일·manifest는 보존한다. 다음은 새 관측의 갱신과 기존 근거리 소실 기억을 분리해 확인하는 r412이며, 임계값을 늘리거나 실제 끊긴 센서를 정상 수신으로 취급하는 수정은 하지 않는다.
