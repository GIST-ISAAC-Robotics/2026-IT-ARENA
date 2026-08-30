// 설치된 Gazebo Math의 속도 제한기만으로 정지 흔들림을 재현합니다.
#include <gz/math/SpeedLimiter.hh>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

int main()
{
  for (const bool jerkEnabled : {true, false})
  {
    gz::math::SpeedLimiter limiter;
    limiter.SetMinVelocity(-2.0);
    limiter.SetMaxVelocity(2.0);
    limiter.SetMinAcceleration(-4.0);
    limiter.SetMaxAcceleration(4.0);
    if (jerkEnabled)
    {
      limiter.SetMinJerk(-20.0);
      limiter.SetMaxJerk(20.0);
    }
    double previous = 0.0, older = 0.0, minimum = 1.0, maximum = -1.0;
    for (int tick = 0; tick < 13000; ++tick)
    {
      double velocity = tick < 7000 ? 0.16 : 0.0;
      limiter.Limit(velocity, previous, older, std::chrono::milliseconds(1));
      older = previous;
      previous = velocity;
      if (tick >= 11000)
      {
        minimum = std::min(minimum, velocity);
        maximum = std::max(maximum, velocity);
      }
    }
    std::cout << (jerkEnabled ? "jerk_20" : "acceleration_only")
              << " last_2s_range=" << minimum << "," << maximum
              << " final=" << previous << '\n';
  }
}
