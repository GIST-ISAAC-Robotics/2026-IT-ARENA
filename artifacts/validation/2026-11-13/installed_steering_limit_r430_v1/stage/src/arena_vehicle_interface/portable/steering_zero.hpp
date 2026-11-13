#pragma once
// 저장 프로필을 검증한 호출자가 채운다. EEPROM/토크/UART/ARM 기능 없음.
#include <algorithm>
#include <cmath>

namespace arena_steering {
struct ZeroGoal {
  bool valid=false,clamped=false;
  unsigned position=0;
  double relative_rad=0;
  const char* reason="calibration_pending";
};
struct ZeroProfile {
  int zero=-1,minimum=-1,maximum=-1,direction=0;
  unsigned counts_per_turn=0;
  bool straight_verified=false,direction_verified=false,endpoints_verified=false;
  bool position_mode_verified=false,scale_verified=false;
  bool valid()const {
    return straight_verified && direction_verified && endpoints_verified && position_mode_verified &&
      scale_verified && counts_per_turn==4096 && (direction==1 || direction==-1) &&
      0<=minimum && minimum<zero && zero<maximum && maximum<=4095;
  }
  double raw(double servo_relative_rad)const {
    return zero+direction*servo_relative_rad*counts_per_turn/6.2831853071795864769;
  }
  bool covers(double minimum_relative_rad,double maximum_relative_rad)const {
    if(!valid()||!std::isfinite(minimum_relative_rad)||!std::isfinite(maximum_relative_rad) ||
       minimum_relative_rad>0||maximum_relative_rad<0)return false;
    const double a=raw(minimum_relative_rad),b=raw(maximum_relative_rad);
    return std::min(a,b)>=minimum && std::max(a,b)<=maximum;
  }
  ZeroGoal goal(double relative_rad)const {
    if(!valid())return {};
    if(!std::isfinite(relative_rad))return {false,false,0,0,"invalid_angle"};
    const double count=raw(relative_rad);
    if(!std::isfinite(count))return {false,false,0,0,"invalid_angle"};
    // %4096나 최단 회전 경로는 사용하지 않는다. 실장 안전 구간 안쪽만 사용한다.
    const double bounded=std::clamp(count,static_cast<double>(minimum),static_cast<double>(maximum));
    const unsigned position=static_cast<unsigned>(std::lround(bounded));
    return {true,count!=bounded,position,
      (static_cast<double>(position)-zero)*6.2831853071795864769/(direction*static_cast<double>(counts_per_turn)),
      count!=bounded?"mounted_limit_clamped":"relative_offset"};
  }
};
}  // namespace arena_steering
