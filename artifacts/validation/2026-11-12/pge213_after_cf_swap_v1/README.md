# 공통 GND 연결·CF 교체 후 엔코더 손 회전

2026-11-12 사용자 요청으로 GPT가 직접 실행했다. [상세 보고서](../../../../docs/sensors/PGE213_ENCODER_HARDWARE_2026_11_12.md#공통-gndcf-교체-후-손-회전-재시험)를 기준으로 한다.

첫 방향 −30, 반대 방향 +34카운트이며 PCNT/software 일치·64개 에지 연속·CRC/순번 공백/invalid/drop/송신 오류0이다. 최종 정지40.64초에서 계수 변화0이다. 눈대중 회전이므로 PPR/정확도/고속 품질은 미확정이다. 모터는 무전력이고 A측 전압 별도 실측은 생략했다.

[집계](summary.json)·[원문 독립 감사](independent_audit.json)·[배포/백업](deployment_summary.json)·[원 앱 복구](restore_summary.json)·[회수 해시](receipt.json). 원 UART·Flash 백업은 Git 제외 build에 보존한다.
