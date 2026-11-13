#pragma once
// 장착 후 측정할 차량 조향각(rad) ↔ 서보 위치(count)의 유한 보간표.
// GPIO/토크 허가/자동 중립 이동은 하지 않는다. 서보 자체 회전각과 차량 조향각은 다르다.
#include <array>
#include <cmath>
#include <cstddef>

namespace arena_steering {
struct Sample { double steering_rad=0.; unsigned position=0; };
struct Goal { bool valid=false; unsigned position=0; const char* reason="calibration_missing"; };

class Mapping {
 public:
  static constexpr std::size_t capacity=17;
  Mapping()=default;
  Mapping(const Sample* samples,std::size_t size) {
    if(samples==nullptr || size<3 || size>capacity)return;
    bool zero=false;int direction=0;
    for(std::size_t i=0;i<size;++i){
      const auto s=samples[i];
      if(!std::isfinite(s.steering_rad) || std::abs(s.steering_rad)>=1.5707963267948966 || s.position>4095)return;
      zero=zero || s.steering_rad==0.;
      if(i){
        if(!(s.steering_rad>samples[i-1].steering_rad) || s.position==samples[i-1].position)return;
        const int next=s.position>samples[i-1].position ? 1:-1;
        if(direction && next!=direction)return;
        direction=next;
      }
      samples_[i]=s;
    }
    if(!zero || !(samples[0].steering_rad<0 && samples[size-1].steering_rad>0))return;
    size_=size;
  }
  bool valid()const{return size_>=3;}
  bool covers(double minimum,double maximum)const{
    return valid() && std::isfinite(minimum) && std::isfinite(maximum) && minimum<=maximum &&
      minimum>=samples_[0].steering_rad && maximum<=samples_[size_-1].steering_rad;
  }
  Goal goal(double angle)const{
    if(!valid())return {};
    if(!std::isfinite(angle))return {false,0,"invalid_angle"};
    if(!covers(angle,angle))return {false,0,"outside_measured_range"};
    for(std::size_t i=0;i<size_;++i){
      if(angle==samples_[i].steering_rad)return {true,samples_[i].position,"measured_sample"};
      if(i && angle<samples_[i].steering_rad){
        const auto a=samples_[i-1],b=samples_[i];
        const double fraction=(angle-a.steering_rad)/(b.steering_rad-a.steering_rad);
        const double value=static_cast<double>(a.position)+fraction*(static_cast<double>(b.position)-a.position);
        return {true,static_cast<unsigned>(std::lround(value)),"interpolated"};
      }
    }
    return {false,0,"invalid_range"};
  }
 private:
  std::array<Sample,capacity> samples_{};
  std::size_t size_=0;
};
}  // namespace arena_steering
