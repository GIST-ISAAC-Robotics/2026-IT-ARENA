// PC 전용 후속 계약 검사. 이 r495 작업에서는 컴파일/실행하지 않았다.
#include "board_profile.hpp"
#include <cassert>
#include <limits>

int main() {
  RuntimeBoardProfile p;
  p.maximum_speed_mps=.1;
  // native 실행 전 fixture 수정: 기본0 조향 한도가 valid() 검사를 가리지 않게 한다.
  // .2rad는 합성 receiver 계약값이며 실장 조향값의 채택이 아니다.
  p.servo.command_limit=.2;
  auto original=p.receiver_config();
  assert(original.max_wheel_mps==5.0);
  p.max_wheel_mps=13./3.6;
  auto selected=p.receiver_config();
  assert(selected.max_wheel_mps==13./3.6);
  assert(selected.max_speed_mps==.1);
  assert(selected.valid());
  p.max_wheel_mps=.09;
  assert(!p.receiver_config().valid());
  p.max_wheel_mps=std::numeric_limits<double>::quiet_NaN();
  assert(!p.receiver_config().valid());
}
