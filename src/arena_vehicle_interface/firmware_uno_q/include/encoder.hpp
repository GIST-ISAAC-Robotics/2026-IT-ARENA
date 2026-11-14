#pragma once
#include <cstdint>
namespace unoq {
class EncoderCounter {
 public:
  bool sample(std::uint32_t raw,std::uint32_t modulus,int sign) {
    if(modulus!=65520 || raw>=modulus || (sign!=1 && sign!=-1))return false;
    if(valid_){
      std::int64_t delta=std::int64_t(raw)-previous_;
      if(delta>std::int64_t(modulus/2))delta-=modulus;
      if(delta< -std::int64_t(modulus/2))delta+=modulus;
      delta*=sign;
      if((delta>0 && count_>INT64_MAX-delta)||(delta<0 && count_<INT64_MIN-delta))return false;
      count_+=delta;
    }
    previous_=raw;valid_=true;return true;
  }
  bool valid()const{return valid_;}
  std::int64_t count()const{return count_;}
 private:
  bool valid_=false;std::uint32_t previous_=0;std::int64_t count_=0;
};
}
