// PC 합성 회귀. 테스트용 PI 값은 차량 프로필/보정값이 아니다.
#include "external_servo.hpp"
#include "bench.hpp"
#include "encoder.hpp"
#include "endpoint.hpp"
#include "profile.hpp"
#include "md20a_forward_output.hpp"
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>
namespace aa=arena_actuation;
const std::uint32_t boot[]={'0','1','2','3','a','b','c','d'};
void put(std::uint8_t* p,std::uint64_t x,unsigned n=8){for(unsigned i=0;i<n;++i)p[i]=(x>>(8*i))&255;}
aa::Command command(const char* kind,const aa::ActuationReceiverCore& r,const aa::Lease& l,int seq=1){
  char data[700];
  std::snprintf(data,sizeof(data),"{\"v\":1,\"kind\":\"%s\",\"boot\":\"0123abcd\",\"generation\":%lld,\"lease\":%lld,\"client\":\"test\",\"seq\":%d%s}",kind,(long long)r.generation(),(long long)l.id,seq,
    std::strcmp(kind,"DRIVE")==0?",\"speed_mps\":0.5,\"steering_rad\":0.1,\"source_age_us\":0,\"ttl_us\":100000":"");
  aa::CommandParser parser;aa::Command out;
  assert(parser.parse(reinterpret_cast<const std::uint8_t*>(data),std::strlen(data),aa::PayloadLayout::kBareCommand,out)==aa::ParseStatus::kObject);
  return out;
}
struct Fixture{
  unoq::Profile p;aa::ActuationReceiverCore r{p.config(),boot,8};unoq::ExternalServo s;unoq::Bench b;
  std::int64_t now=220000;
  Fixture(){for(std::int64_t t=0;t<=now;t+=5000){assert(r.update_motor_encoder(t,aa::IntInput::of(t/5000),aa::IntInput::of(t),aa::IntInput::of(0)));r.tick(t);}}
  aa::Lease offer(){auto o=r.offer(now);assert(o.ok);return o.lease;}
  std::vector<std::uint8_t> esv(std::uint64_t seq=1,bool ready=true,std::uint32_t age=0,std::int64_t applied=-1){
    std::vector<std::uint8_t> v(49);std::memcpy(v.data(),"ESV10123abcd",12);put(v.data()+12,r.generation());put(v.data()+20,offer().id);
    put(v.data()+28,seq);put(v.data()+36,age,4);v[40]=ready;put(v.data()+41,applied);return v;
  }
  std::vector<std::uint8_t> ben(unsigned action,std::uint64_t seq,unsigned duty=0,unsigned duration=0){
    std::vector<std::uint8_t> v(44);std::memcpy(v.data(),"BNV10123abcd",12);put(v.data()+12,r.generation());put(v.data()+20,offer().id);put(v.data()+28,seq);
    v[36]=action;put(v.data()+38,duty,2);put(v.data()+40,duration,4);return v;
  }
  bool accept(const std::vector<std::uint8_t>& v){return s.accept(v.data(),v.size(),now,r);}
  bool bench(const std::vector<std::uint8_t>& v,bool enabled=true,bool ready=true){return b.accept(v.data(),v.size(),now,r,enabled,ready);}
  void control(const char* kind,int seq=1){auto l=offer();auto g=r.generation();bool armed=r.armed();assert(r.receive(command(kind,r,l,seq),now).accepted);s.transition(g,armed,r);}
};
void servo_boundaries(){
  Fixture f;auto good=f.esv();assert(f.accept(good));assert(f.s.fresh(f.now,f.r));auto deadline=f.s.deadline();
  auto oldfalse=good;oldfalse[40]=0;assert(!f.accept(oldfalse));assert(f.s.fresh(f.now,f.r));
  assert(!f.s.fresh(deadline,f.r));assert(f.s.fresh(deadline-1,f.r));
  auto wrong=f.esv(2);wrong[4]='f';assert(!f.accept(wrong));assert(f.s.fresh(f.now,f.r));
  wrong=f.esv(2);put(wrong.data()+12,f.r.generation()+1);assert(!f.accept(wrong));
  wrong=f.esv(UINT64_MAX);assert(!f.accept(wrong));
  wrong=f.esv(2,true,100000);assert(!f.accept(wrong));
  wrong=f.esv(2,true,99999);++f.now;assert(!f.accept(wrong)); // observation expiry exact
  auto freshfalse=f.esv(2,false);assert(f.accept(freshfalse));assert(!f.s.fresh(f.now,f.r));
  assert(!f.accept(good)); // replay cannot release false
  auto next=f.esv(3);assert(f.accept(next));auto invalid=next;invalid[40]=2;
  assert(!f.accept(invalid));assert(!f.s.fresh(f.now,f.r));
  next=f.esv(4);put(next.data()+41,UINT64_MAX-1);assert(!f.accept(next)); // negative other than -1
  next=f.esv(4);assert(f.accept(next));auto expired=f.esv(5,false);f.now+=100000;
  assert(!f.accept(expired));assert(!f.s.fresh(f.now,f.r));
  Fixture reboot;assert(reboot.accept(reboot.esv(0))); // sequence may restart only in new MCU object
  auto oldboot=good;oldboot[4]='9';assert(!reboot.accept(oldboot));
}
void arm_and_applied(){
  Fixture f;auto ready=f.esv(1);assert(f.accept(ready));auto deadline=f.s.deadline();
  f.control("ARM");assert(f.s.fresh(f.now,f.r));assert(f.s.deadline()==deadline);assert(!f.s.applied(f.now,f.r));
  auto oldgen=ready;assert(!f.accept(oldgen));assert(f.s.fresh(f.now,f.r));
  f.control("DRIVE",2);assert(!f.s.applied(f.now,f.r));
  assert(f.accept(f.esv(2,true,0,2)));assert(f.s.applied(f.now,f.r));
  f.control("DRIVE",3);assert(!f.s.applied(f.now,f.r));
  assert(f.accept(f.esv(3,true,0,3)));assert(f.s.applied(f.now,f.r));
  f.control("DISARM",4);assert(!f.s.fresh(f.now,f.r));
  assert(!f.accept(f.esv(1))); // generation transition does not reset monotonic status seq
  f.control("ARM",5);assert(!f.s.fresh(f.now,f.r)); // no evidence to carry
  auto g=f.r.generation();auto armed=f.r.armed();
  f.r.report_control_fault(aa::ControlFault::kOutputUnavailable,f.now);f.s.transition(g,armed,f.r);assert(!f.s.fresh(f.now,f.r));
  f.control("CLEAR",6);assert(!f.s.fresh(f.now,f.r));
}
void bench_boundaries(){
  Fixture f;assert(f.b.sequence()==-1);assert(!f.b.armed());
  assert(!f.bench(f.ben(1,1),false));assert(!f.bench(f.ben(1,1),true,false));
  assert(f.bench(f.ben(1,1)));assert(f.b.armed());assert(!f.s.fresh(f.now,f.r)); // motor-only no servo
  assert(f.bench(f.ben(2,2,150,5000000)));assert(f.b.running());assert(f.b.duty()==.15);
  const auto end=f.b.stop_at();assert(!f.bench(f.ben(2,3,150,5000000)));assert(f.b.stop_at()==end);
  f.now+=50000;assert(f.bench(f.ben(4,3)));assert(f.b.stop_at()==end);
  assert(!f.bench(f.ben(4,3))); // repeat cannot extend host deadline
  f.b.tick(f.b.deadline(),f.r,true);assert(!f.b.armed());assert(f.b.duty()==0);
  Fixture bad;assert(bad.bench(bad.ben(1,1)));assert(!bad.bench(bad.ben(2,2,151,5000000)));assert(!bad.b.armed());
  assert(bad.bench(bad.ben(1,3)));assert(!bad.bench(bad.ben(2,4,150,5000001)));assert(!bad.b.armed());
  Fixture once;assert(once.bench(once.ben(1,1)));assert(once.bench(once.ben(2,2,50,50000)));
  once.now+=40000;assert(once.bench(once.ben(4,3)));assert(once.b.deadline()==270000);
  assert(!once.b.handle_output_deadline(269999,once.r,true)); // control-gap fault path before local expiry
  assert(once.b.handle_output_deadline(270000,once.r,true));assert(!once.b.armed());
  assert(once.r.state()==aa::ReceiverState::kDisarmed);assert(std::strcmp(once.b.reason(),"duration_complete")==0);
  assert(!once.bench(once.ben(2,4,50,50000))); // no auto rearm
  Fixture stop;assert(stop.bench(stop.ben(1,1)));auto s=stop.ben(3,0);s[4]='x';assert(stop.bench(s));assert(!stop.b.armed());
}
void encoder_boundaries(){
  unoq::EncoderCounter c;assert(c.sample(65519,65520,1));assert(c.sample(0,65520,1));assert(c.count()==1);
  assert(c.sample(65519,65520,1));assert(c.count()==0);assert(c.sample(65518,65520,-1));assert(c.count()==1);
  assert(!c.sample(65520,65520,1));assert(!c.sample(0,65536,1));assert(!c.sample(0,65520,0));
  unoq::EncoderCounter full;assert(full.sample(0,65520,1));
  for(std::uint32_t i=1;i<=6000;++i)assert(full.sample((i*118)%65520,65520,1));
  assert(full.count()==708000); // 5900 edges/s, many complete wraps, 20 ms samples
}
void framing(){
  Fixture f;auto p=f.esv();std::uint8_t framed[100];auto n=aa::encode_frame(p.data(),p.size(),framed,sizeof(framed));assert(n==57);
  aa::FrameLink link(30000);link.connect(f.now);unsigned delivered=0;
  auto receive=[&](const std::uint8_t* b,std::size_t z,aa::FrameOrigin origin){assert(origin==aa::FrameOrigin::kLive);++delivered;assert(f.s.accept(b,z,f.now,f.r));};
  for(std::size_t i=0;i<n-1;++i)link.feed(f.now,framed+i,1,receive);
  assert(delivered==0);link.feed(f.now,framed+n-1,1,receive);assert(delivered==1);
  framed[n-1]^=1;link.feed(f.now,framed,n,receive);assert(delivered==1);assert(link.framing_errors()>0);
  // Main applies the same framing-error reaction before its next control output.
  f.s.invalidate();assert(!f.s.fresh(f.now,f.r));
  // Independent Python host struct/CRC fixture; no shared encoder implementation.
  std::memcpy(p.data()+4,"1234abcd",8);put(p.data()+12,0);put(p.data()+20,1);p[40]=0;
  n=aa::encode_frame(p.data(),p.size(),framed,sizeof(framed));
  char hex[115]{};for(std::size_t i=0;i<n;++i)std::snprintf(hex+i*2,3,"%02x",framed[i]);
  assert(std::strcmp(hex,"494131004553563131323334616263640000000000000000010000000000000001000000000000000000000000ffffffffffffffffa6bb51ff")==0);
}
void installed_output(){
  Fixture f;aa::MotorControlCandidate cycle(f.r,.6,1.2,.05,0.,.15);
  cycle.tick(220000,false);assert(f.accept(f.esv()));f.control("ARM");f.now+=5000;
  cycle.tick(f.now,true);f.control("DRIVE",2);f.now+=5000;
  auto inhibited=cycle.tick(f.now,true,!f.s.applied(f.now,f.r));assert(inhibited.motor.effort==0);
  assert(f.r.state()==aa::ReceiverState::kActive);assert(f.accept(f.esv(2,true,0,2)));f.now+=5000;
  auto active=cycle.tick(f.now,true,!f.s.applied(f.now,f.r));
  auto mapped=aa::md20a_forward_command(active,f.r,{.15,false},true);assert(mapped.status==aa::Md20aMapStatus::kActive);assert(mapped.duty_ratio>0 && mapped.duty_ratio<=.15);
  f.s.invalidate();f.now+=5000;assert(cycle.tick(f.now,false).motor.effort==0);assert(f.r.state()==aa::ReceiverState::kFault);
}
void unconfigured_endpoint(){
  unoq::Profile p;aa::TxConfig tc;tc.offer_period_us=20000;tc.max_consecutive_events=2;
  unoq::Endpoint e(p.config(),boot,tc,0,0,0,false);e.connect(0);
  for(std::int64_t t=0;t<=220000;t+=5000){assert(e.feedback(t,t/5000,0));e.tick(t,false,false);}
  assert(e.receiver().state()==aa::ReceiverState::kDisarmed); // host sends explicit startup STOP; MCU never auto ARMs
  assert(!p.installed());p.mode=unoq::Mode::Installed;assert(!p.installed());
}
int main(){servo_boundaries();arm_and_applied();bench_boundaries();encoder_boundaries();framing();installed_output();unconfigured_endpoint();std::puts("UNO Q firmware portable boundaries PASS");}
