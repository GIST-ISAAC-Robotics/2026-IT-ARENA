# 실제 모터축 카운트의 운동 입력 r397

P0-2/P0-4: 실제 offer의 motor_feedback_v1을 기존 McuStatusClock으로 해석한 뒤 단일 모터축 JointState에 필요한 값을 만드는 준비다. 첫 표본·invalid·무관측은 속도를 만들지 않으며 두 유효 표본의 같은 카운트는 실제 관측된0속도로 구분한다. 원 MCU 취득 간격으로 평균속도를 계산하고 ROS stamp는 기존 최소 수신offset 추정 그대로다. 시각 교정/장착방향/실차 구동 검증이 아니다.

분해능28·2.5×81/17감속·65mm는 명시 MotorEncoderSpec을 통해 사용한다. 부호·최대간격·나이는 호출자가 명시한다. boot/clock epoch/제어세대 변경 후 첫 표본부터 다시 시작한다. 무효·큰 점프를 유효0이나 이전속도 유지로 바꾸지 않는다. 아직 공통연결/실물장치/ROS발행은 하지 않았다. 실제 MCU 카운트가 없을 때 기존 운동history를 가짜로 채우지 않는다.

verify_v1:본19+offer33=52검사 통과·5파일 전후불변. 독립 코드검토 후 공통 모듈/경로독립 검사로 이관했다. receipt 기준 age와 실제 processing 지연은 다르므로 후자는 원stamp를 유지하는 downstream freshness가 검사한다. ROS/실물발행은 아직 후속이다.
