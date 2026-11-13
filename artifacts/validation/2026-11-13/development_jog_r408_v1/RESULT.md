# r408 공통 조작 도구 이관

기존 `scripts/development_jog.py`에 staged 보완을 바이트 동일하게 이관했다. 기존 virtual 상태와 현행 runtime host 관측 상태를 구분하고, `ESTOP_LATCHED`·세대/시계 경계·유효 feedback을 확인한다. 종료0발행 실패와 STOP요청을 분리하며 종료0속도는 요청 조향을 유지한다. 자율주행 실행 루프에는 추가하지 않았다.

staged37순수검사 후 공통 기존13+신규37, **50통과·skip0**을 확인했다. 공통 신규 검사는 import 위치만 staged에서 실제 script로 바꿨다. `integration_v1/integration.json`은 이관 시점, `integration_v1/verify_v1/result.json`은 후속 검증 결과다. 원 r169와 staged manifest는 보존했다.

실제 ROS·UART·장치·`--send`는 이번에 실행하지 않았다. 실차 방향·중립·출력·정지는 조립 후 확인한다. 기본 미리보기만 제공하며 새 CLI·WASD·자동 ARM/CLEAR/RESET은 없다. 기본0.15m/s·0.5초, 상한0.2m/s는 개발 시험값이다. 명시 ARM·유효 센서·독립 보호층·현재 host 출력 준비가 별도로 필요하다.
