# CF 교체 후 모터·엔코더 결합 시험

2026-11-12 GPT 직접 실행. [상세 보고서](../../../../docs/sensors/MD20A_MOTOR_HARDWARE_2026_11_12.md#공통-gndcf-교체-후-저출력-결합-재시험)를 기준으로 한다.

약5%·500 ms×4회에서 −116/−120/−123/−124카운트, PCNT/software 일치·invalid/drop0이다. 사용자가 회전과 정지를 확인했다. 전체 수신55,197프레임의 CRC/순번은 정상이나, 마지막 구동281.538초 뒤 대기 중 추가1854에지·invalid3·ring drop270이 있다. 이후 사용자가 엔코더·모터 쪽 연결을 제거했다고 보고했으므로 전원/신호 해체 영향을 의심하되 원인을 확정하지 않는다. 전체 세션을 무잡음 통과로 표시하지 않는다.

[원문 감사](independent_audit.json)·[구동/후반 구간 분리 감사](edge_timing_audit.json)·[최초 전구간 연속성 감사 실패](edge_timing_v1_failure.json)·[사용자 관찰](user_observations.json)·[무전력 사전 검사](preflight.json)·[종료](closed.json)·[복구](restore_summary.json)·[회수 해시](receipt.json). 원 앱1 MiB 복구/readback 일치·inactive·포트 소유자0으로 종료했다. 원시 UART/Flash는 Git 제외 build에 보존한다.
