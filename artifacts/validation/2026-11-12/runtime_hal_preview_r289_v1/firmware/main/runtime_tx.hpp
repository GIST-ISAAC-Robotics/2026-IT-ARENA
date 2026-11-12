#pragma once
// IA 계약/68 B IMU/원시 관측의 UART 프레임 경계를 지킨다. 단일 소유 작업.
#include "actuation_motor_endpoint.hpp"
#include <array>
#include <cstring>

class RuntimeTx {
 public:
  bool imu(const void* data,std::size_t size) {
    if(!data || size!=68 || count_==queue_.size()){++imu_dropped_;return false;}
    std::memcpy(queue_[(head_+count_)%queue_.size()].data(),data,68);++count_;return true;
  }
  bool observation(const std::uint8_t* data,std::size_t size) {
    // 시작된 프레임을 덮지 않는다. 보관 공간은 1개이며 밀리면 다음 주기 원시 관측을 기다린다.
    if(!data || !size || size>observation_.size() || observation_size_){++observation_dropped_;return false;}
    std::memcpy(observation_.data(),data,size);observation_size_=size;return true;
  }
  template<class Endpoint,class Sink>
  bool pump(std::int64_t now,Endpoint& endpoint,Sink&& sink) {
    if(failed_)return false;
    if(observation_offset_)return send_observation(sink);
    if(imu_offset_)return send_imu(sink);
    endpoint.pump(now,sink,false); // 이 preview는 준비/ARM/구동을 허가하지 않는다.
    if(endpoint.tx().fault()!=arena_actuation::TxFault::kNone)return fail();
    if(endpoint.tx().frame_in_flight())return true;
    if(observation_size_ && (prefer_observation_ || !count_))return send_observation(sink);
    if(count_)return send_imu(sink);
    return true;
  }
  std::uint64_t imu_dropped()const{return imu_dropped_;}
  std::uint64_t observation_dropped()const{return observation_dropped_;}
 private:
  bool fail(){failed_=true;return false;}
  template<class Sink> bool send_imu(Sink& sink){
    const auto n=sink(queue_[head_].data()+imu_offset_,68-imu_offset_);
    if(n>68-imu_offset_)return fail();
    imu_offset_+=n;
    if(imu_offset_==68){imu_offset_=0;head_=(head_+1)%queue_.size();--count_;prefer_observation_=true;}
    return true;
  }
  template<class Sink> bool send_observation(Sink& sink){
    const auto n=sink(observation_.data()+observation_offset_,observation_size_-observation_offset_);
    if(n>observation_size_-observation_offset_)return fail();
    observation_offset_+=n;
    if(observation_offset_==observation_size_){observation_offset_=observation_size_=0;prefer_observation_=false;}
    return true;
  }
  std::array<std::array<std::uint8_t,68>,64> queue_{};
  std::array<std::uint8_t,1024> observation_{};
  std::size_t head_=0,count_=0,imu_offset_=0,observation_offset_=0,observation_size_=0;
  std::uint64_t imu_dropped_=0,observation_dropped_=0;
  bool failed_=false,prefer_observation_=false;
};
