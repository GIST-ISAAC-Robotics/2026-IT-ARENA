#pragma once
#include "actuation_receiver.hpp"
#include <cstring>
namespace unoq {
inline std::uint64_t read_u64(const std::uint8_t* p) {
  std::uint64_t out=0; for(unsigned i=0;i<8;++i)out|=std::uint64_t(p[i])<<(8*i);return out;
}
inline std::uint32_t read_u32(const std::uint8_t* p) {
  std::uint32_t out=0; for(unsigned i=0;i<4;++i)out|=std::uint32_t(p[i])<<(8*i);return out;
}
inline bool identity(const std::uint8_t* p,const arena_actuation::ActuationReceiverCore& r) {
  if(r.boot().length!=8 || read_u64(p+12)!=std::uint64_t(r.generation()))return false;
  for(unsigned i=0;i<8;++i)if(p[4+i]!=r.boot().code_points[i])return false;
  return true;
}
inline const arena_actuation::Lease* lease(const arena_actuation::ActuationReceiverCore& r,std::uint64_t id) {
  for(std::size_t i=0;i<r.lease_count();++i)
    if(id==std::uint64_t(r.lease_at(i).id))return &r.lease_at(i);
  return nullptr;
}
class ExternalServo {
 public:
  // ESV1 <4s8sQQQIBq; sequence is monotonic for the whole MCU boot.
  bool accept(const std::uint8_t* p,std::size_t n,std::int64_t now,
              const arena_actuation::ActuationReceiverCore& r) {
    if(n!=49 || std::memcmp(p,"ESV1",4)!=0 || p[40]>1 || now<0) {
      invalidate();++malformed;return false;
    }
    const auto seq=read_u64(p+28);const auto age=read_u32(p+36);
    const auto* l=lease(r,read_u64(p+20));
    if(!identity(p,r) || !l || now<l->issued_us || now>=l->expires_us ||
       (seen_ && seq<=seq_) || age>=100000 || seq>std::uint64_t(INT64_MAX)) {
      ++rejected;return false; // stale false cannot invalidate a newer true, nor refresh it.
    }
    const auto bound=l->issued_us+100000-std::int64_t(age);
    const auto expiry=bound<l->expires_us?bound:l->expires_us;
    if(now>=expiry){++rejected;return false;}
    const auto applied=read_u64(p+41);
    if(applied>std::uint64_t(INT64_MAX) && applied!=UINT64_MAX){invalidate();++malformed;return false;}
    seen_=true;seq_=seq;generation_=r.generation();deadline_=expiry;
    ready_=p[40]==1;applied_=applied==UINT64_MAX?-1:static_cast<std::int64_t>(applied);
    ++accepted;return true;
  }
  void transition(std::int64_t old_generation,bool old_armed,
                  const arena_actuation::ActuationReceiverCore& r) {
    if(old_generation==r.generation())return;
    // Only the actual explicit ARM transition can carry pre-ARM evidence.
    const bool arm=!old_armed && r.armed() && r.generation()==old_generation+1 &&
      std::strcmp(r.reason(),"explicit_arm")==0 && generation_==old_generation;
    if(arm)generation_=r.generation();else invalidate();
  }
  bool fresh(std::int64_t now,const arena_actuation::ActuationReceiverCore& r) const {
    return ready_ && now>=0 && now<deadline_ && generation_==r.generation();
  }
  bool applied(std::int64_t now,const arena_actuation::ActuationReceiverCore& r)const {
    return fresh(now,r) && r.has_owner() && r.drive_seen() && applied_==r.accepted_seq();
  }
  void invalidate(){ready_=false;deadline_=0;applied_=-1;generation_=-1;}
  std::int64_t deadline()const{return deadline_;}
  std::int64_t applied_seq()const{return applied_;}
  std::uint64_t accepted=0,rejected=0,malformed=0;
 private:
  bool seen_=false,ready_=false;
  std::uint64_t seq_=0;
  std::int64_t generation_=-1,deadline_=0,applied_=-1;
};
}
