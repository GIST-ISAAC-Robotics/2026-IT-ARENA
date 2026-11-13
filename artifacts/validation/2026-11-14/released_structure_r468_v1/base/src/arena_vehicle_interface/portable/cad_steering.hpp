#pragma once
// CAD 기하 전용. 실장 교정/전류/간섭/마찰 모델이나 출력 허가가 아니다.
#include <algorithm>
#include <cmath>
#include <cstddef>

namespace arena_steering {
struct CadSample { double servo_rad, left_rad, right_rad; };
struct CadGoal {
  bool valid=false, clamped=false;
  double servo_rad=0, left_rad=0, right_rad=0, equivalent_rad=0;
};
inline double EquivalentAngle(double left,double right) {
  if(left==0 && right==0)return 0;
  const double a=std::tan(left),b=std::tan(right);
  return std::atan(2*a*b/(a+b));
}
class CadMapping {
 public:
  // 데이터는 객체보다 오래 살아야 한다. 표는 서보 오름차순, 좌/우각 내림차순.
  CadMapping(const CadSample* samples,std::size_t size): samples_(samples),size_(size) {
    if(!samples || size<3 || size>96)return;
    bool zero=false;
    for(std::size_t i=0;i<size;++i){
      const auto p=samples[i];
      if(!std::isfinite(p.servo_rad)||!std::isfinite(p.left_rad)||!std::isfinite(p.right_rad) ||
         std::abs(p.servo_rad)>=1.4||std::abs(p.left_rad)>=1.4||std::abs(p.right_rad)>=1.4)return;
      if(p.servo_rad==0){if(p.left_rad!=0 || p.right_rad!=0)return;zero=true;}
      else if(p.left_rad*p.servo_rad>=0 || p.right_rad*p.servo_rad>=0)return;
      if(i && (!(p.servo_rad>samples[i-1].servo_rad) || !(p.left_rad<samples[i-1].left_rad) ||
               !(p.right_rad<samples[i-1].right_rad)))return;
    }
    valid_=zero && samples[0].servo_rad<0 && samples[size-1].servo_rad>0;
  }
  bool valid()const{return valid_;}
  CadGoal relative(double servo)const {
    if(!valid_ || !std::isfinite(servo))return {};
    const double s=std::clamp(servo,samples_[0].servo_rad,samples_[size_-1].servo_rad);
    std::size_t hi=1;
    while(hi<size_-1 && samples_[hi].servo_rad<s)++hi;
    const auto a=samples_[hi-1],b=samples_[hi];
    const double t=(s-a.servo_rad)/(b.servo_rad-a.servo_rad);
    const double l=a.left_rad+t*(b.left_rad-a.left_rad),r=a.right_rad+t*(b.right_rad-a.right_rad);
    return {true,s!=servo,s,l,r,EquivalentAngle(l,r)};
  }
  CadGoal central(double requested,double wheelbase,double track,double wheel_limit)const {
    if(!valid_ || !std::isfinite(requested)||!std::isfinite(wheelbase)||!std::isfinite(track) ||
       !std::isfinite(wheel_limit)||wheelbase<=0||track<=0||wheel_limit<=0||wheel_limit>=1.4)return {};
    const double target=std::clamp(requested,
      EquivalentAngle(samples_[size_-1].left_rad,samples_[size_-1].right_rad),
      EquivalentAngle(samples_[0].left_rad,samples_[0].right_rad));
    if(target==0)return {true,requested!=0,0,0,0,0};
    // 같은 표의 단조 구간을 역산한다. 독립 좌/우 클립으로 링크 곡선을 바꾸지 않는다.
    std::size_t hi=1;
    while(hi<size_-1 && EquivalentAngle(samples_[hi].left_rad,samples_[hi].right_rad)>target)++hi;
    double lo=samples_[hi-1].servo_rad,up=samples_[hi].servo_rad;
    for(int i=0;i<40;++i){const double mid=(lo+up)/2;
      if(relative(mid).equivalent_rad>target)lo=mid;else up=mid;}
    auto goal=relative((lo+up)/2);
    bool clipped=requested!=target;
    if(std::max(std::abs(goal.left_rad),std::abs(goal.right_rad))>wheel_limit){
      double inside=0,outside=goal.servo_rad;
      for(int i=0;i<40;++i){const double mid=(inside+outside)/2;const auto p=relative(mid);
        if(std::max(std::abs(p.left_rad),std::abs(p.right_rad))<=wheel_limit)inside=mid;else outside=mid;}
      goal=relative(inside);clipped=true;
    }
    goal.clamped=clipped;return goal;
  }
 private:
  const CadSample* samples_=nullptr;
  std::size_t size_=0;
  bool valid_=false;
};
}  // namespace arena_steering
