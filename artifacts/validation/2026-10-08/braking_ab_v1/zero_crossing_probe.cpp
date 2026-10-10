// 원래 SingleMotor 식의 백색상자 특성 확인. 차량/모터 실제 운동 재현이 아니다.
// 0목표에서 양의 관절속도로 제동 토크를 만든 뒤 관측 속도를 강제로 음수로 바꾼다.
// 강제한 관측값은 런타임 제어기에 공급하지 않는다. 플러그인/설정 변경 없음.
#include "arena_gazebo/drivetrain.hpp"
#include <iomanip>
#include <iostream>

int main()
{
  arena::SingleMotor motor;
  arena::DriveState state;
  for (int i = 0; i < 1000; ++i) state = motor.Step(0., 30., 30., .001);
  std::cout << std::setprecision(12);
  std::cout << "{\"scope\":\"prescribed wheel speed sign change; zero target; not vehicle dynamics or real braking\",\"samples\":[";
  int positivePower = 0;
  for (int i = 0; i < 100; ++i)
  {
    state = motor.Step(0., -5., -5., .001);
    double power = state.motorTorque*state.motorSpeed;
    if (power > 0.) ++positivePower;
    if (i) std::cout << ',';
    std::cout << "{\"t_s\":" << (i+1)*.001 << ",\"target_mps\":0,\"wheel_rad_s\":-5,\"motor_torque_nm\":"
              << state.motorTorque << ",\"motor_mechanical_power_w\":" << power << '}';
  }
  std::cout << "],\"positive_power_samples\":" << positivePower << ",\"sample_period_s\":0.001}\n";
}
