#pragma once
#include <cstdint>

// 내부 풀업·접점 LOW 허용. 정지 해제는 입력 해제와 별개인 기존 CLEAR/ARM 계약이다.
class StopSwitchInput {
 public:
  template<class Endpoint>
  void sample(int raw_level,std::int64_t now,Endpoint& endpoint) {
    const bool held=raw_level!=0;
    if (!initialized_ || held!=held_) {
      endpoint.physical_stop(held,now);
      initialized_=true;held_=held;++changes_;
    }
  }
  bool held() const { return held_; }
  std::uint64_t changes() const { return changes_; }
 private:
  bool initialized_=false,held_=true;
  std::uint64_t changes_=0;
};
