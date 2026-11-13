#pragma once
#include <cstdint>

// 샘플 시각은 호출자가 실제로 읽는다. 이 클래스는 다음 5 ms 예약만 소유한다.
class FixedTick {
 public:
  explicit FixedTick(std::int64_t first_us):next_(first_us){}
  bool poll(std::int64_t now){
    if(now<next_)return false;
    const auto late=now-next_;
    if(late>maximum_lateness_)maximum_lateness_=late;
    const auto skipped=late/5000;
    missed_+=static_cast<std::uint64_t>(skipped);
    next_+=(skipped+1)*5000;
    return true;
  }
  std::uint64_t missed()const{return missed_;}
  std::int64_t maximum_lateness()const{return maximum_lateness_;}
 private:
  std::int64_t next_,maximum_lateness_=0;
  std::uint64_t missed_=0;
};
