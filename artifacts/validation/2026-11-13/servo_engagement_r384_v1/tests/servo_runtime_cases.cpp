#include "runtime_servo.hpp"
#include "cad_steering_table.hpp"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace as=arena_servo;
namespace st=arena_steering;
#define REQUIRE(c) do { if(!(c)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#c);std::exit(1);} }while(false)

as::RuntimeServoConfig config() {
  // 합성 값이며 장착/주행 프로필이 아니다.
  as::RuntimeServoConfig c;
  c.id=1;c.goal_speed=120;c.goal_acceleration=10;
  c.position_period_us=100000;c.observation_max_age_us=200000;
  c.command_limit=.4;c.wheelbase=.128;c.track=.1239;c.wheel_limit=.5235987755982988;
  c.zero={2000,1000,3000,1,4096,true,true,true,true,true};return c;
}
std::vector<std::uint8_t> reply(unsigned size,unsigned value=0,unsigned error=0) {
  std::vector<std::uint8_t> out={255,255,1,static_cast<std::uint8_t>(size+2),static_cast<std::uint8_t>(error)};
  for(unsigned i=0;i<size;++i)out.push_back(static_cast<std::uint8_t>(value>>(8*i)));
  unsigned sum=0;for(std::size_t i=2;i<out.size();++i)sum+=out[i];
  out.push_back(static_cast<std::uint8_t>(~sum));return out;
}
struct Fixture {
  st::CadMapping map{st::kCadSamples,sizeof(st::kCadSamples)/sizeof(st::kCadSamples[0])};
  as::RuntimeServoConfig cfg;
  as::RuntimeServo servo;
  as::RuntimeServoSnapshot snap{};
  std::int64_t now=0;
  std::size_t capacity=64;
  std::vector<std::uint8_t> bytes;
  std::vector<std::vector<std::uint8_t>> requests;
  explicit Fixture(as::RuntimeServoConfig c=config()):cfg(c),servo(map,cfg){}
  void step(const std::vector<std::uint8_t>& rx={}) {
    servo.step(now,snap,rx.data(),rx.size(),[&](const std::uint8_t* p,std::size_t n)->std::size_t {
      n=std::min(n,capacity);bytes.insert(bytes.end(),p,p+n);
      while(bytes.size()>=4 && bytes.size()>=static_cast<std::size_t>(bytes[3])+4){
        const auto size=static_cast<std::size_t>(bytes[3])+4;
        requests.emplace_back(bytes.begin(),bytes.begin()+size);bytes.erase(bytes.begin(),bytes.begin()+size);
      }
      return n;
    });now+=1000;
  }
  void ack(unsigned size=0,unsigned value=0,unsigned error=0){step(reply(size,value,error));}
  const std::vector<std::uint8_t>& last()const{REQUIRE(!requests.empty());return requests.back();}
  unsigned address()const{return last()[5];}
  unsigned goal()const{REQUIRE(address()==41);return last()[7]|(unsigned(last()[8])<<8);}
  std::size_t writes()const {
    std::size_t n=0;for(const auto& p:requests){if(p[4]==3)++n;}return n;
  }
  void probe(){step();REQUIRE(address()==33);ack(1,0);REQUIRE(address()==56);ack(2,2100);
    REQUIRE(servo.state()==as::RuntimeServoState::Standby && servo.ready(now));REQUIRE(writes()==0);}
  void begin_engage(){snap.armed=true;snap.generation=1;step();REQUIRE(address()==56);}
  void engage_to(unsigned stage){probe();begin_engage();if(stage>=1){ack(2,2100);REQUIRE(goal()==2100);}
    if(stage>=2){ack();REQUIRE(address()==40 && last()[6]==1);}}
  void engage(){engage_to(2);REQUIRE(!servo.engaged());ack();REQUIRE(servo.engaged());}
  unsigned cad(double steering)const{return st::ResolveCadServoGoal(map,cfg.zero,steering,cfg.command_limit,
    cfg.wheelbase,cfg.track,cfg.wheel_limit).servo.position;}
};

void basic() {
  Fixture f;f.probe();
  for(int i=0;i<3;++i){f.now+=f.cfg.position_period_us;f.step();REQUIRE(f.address()==56);f.ack(2,2100);}
  REQUIRE(f.writes()==0);f.begin_engage();REQUIRE(!f.servo.engaged());
  f.ack(2,2100);REQUIRE(f.goal()==2100 && !f.servo.engaged());
  f.ack();REQUIRE(f.address()==40 && f.last()[6]==1 && !f.servo.engaged());f.ack();
  REQUIRE(f.servo.engaged());const auto writes=f.writes();
  for(int i=0;i<5;++i){f.step();}
  REQUIRE(f.writes()==writes); // 첫 DRIVE 전 중앙 목표 없음
  f.snap.drive_seen=true;f.snap.steering_rad=.2;f.step();
  REQUIRE(f.goal()==f.cad(.2) && f.servo.engaged()); // snapshot에 speed 조건이 없으므로 DRIVE(0)도 동일
  f.ack();REQUIRE(f.writes()==writes+1);
  f.snap.steering_rad=0;f.step();REQUIRE(f.goal()==2000); // 명시 DRIVE의 중앙만 전송
}
void latest() {
  Fixture f;f.engage();f.snap.drive_seen=true;f.snap.steering_rad=.1;f.step();
  const auto first=f.goal();const auto count=f.writes();
  f.snap.steering_rad=.2;f.step();f.snap.steering_rad=-.3;f.step();
  REQUIRE(f.writes()==count && f.goal()==first);f.ack();
  REQUIRE(f.writes()==count+1 && f.goal()==f.cad(-.3));f.ack();
  const auto count2=f.writes();f.capacity=0;f.snap.steering_rad=.1;f.step();
  f.snap.steering_rad=.3;f.step();f.capacity=64;f.step();
  REQUIRE(f.writes()==count2+1 && f.goal()==f.cad(.3));
  f.ack();f.capacity=0;f.snap.steering_rad=.1;f.step();
  f.snap.steering_rad=.3;f.step(); // ACK된 목표로 돌아오면 미송신 요청 자체를 버린다.
  f.now+=as::Transaction::timeout_us;f.capacity=64;f.step();
  REQUIRE(f.servo.engaged() && f.writes()==count2+1);
}
void backpressure_timeout() {
  Fixture f;f.engage();f.snap.drive_seen=true;f.snap.steering_rad=.1;f.capacity=0;f.step();
  const auto deadline=f.servo.transaction_deadline();
  for(int i=0;i<10;++i){f.snap.steering_rad=i%2?.1:.2;f.step();REQUIRE(f.servo.transaction_deadline()==deadline);}
  f.now=deadline;f.snap.steering_rad=.3;f.step();
  REQUIRE(f.servo.state()==as::RuntimeServoState::Fault && f.servo.transaction_error()==as::Error::Deadline);
}
void boundary(unsigned stage,bool generation) {
  Fixture f;f.engage_to(stage);const auto writes=f.writes();
  if(generation)++f.snap.generation;else {f.snap.armed=false;f.snap.drive_seen=false;}
  f.ack(stage==0?2:0,2100);
  REQUIRE(!f.servo.engaged() && f.writes()==writes);
  if(generation){REQUIRE(f.address()==56);f.ack(2,2200);REQUIRE(f.goal()==2200);}
  else {for(int i=0;i<3;++i){f.step();}REQUIRE(f.writes()==writes);
    REQUIRE(f.servo.state()==as::RuntimeServoState::Standby);}
}
void unsent(unsigned stage) {
  Fixture f;f.engage_to(stage); // stage0 read 진행, stage1 hold 진행
  f.capacity=0;f.ack(stage==0?2:0,2100);
  const auto writes=f.writes();REQUIRE(f.servo.transaction_phase()==as::Phase::Sending);
  f.snap.armed=false;f.capacity=64;f.step();
  REQUIRE(f.writes()==writes && f.bytes.empty() && !f.servo.engaged());
}
void partial() {
  Fixture f;f.engage_to(0);f.capacity=2;f.ack(2,2100);REQUIRE(f.bytes.size()==2);
  f.snap.armed=false;f.capacity=64;f.step();REQUIRE(f.goal()==2100);
  const auto writes=f.writes();f.ack();REQUIRE(!f.servo.engaged() && f.writes()==writes);
}
void timeout(unsigned stage) {
  Fixture f;f.engage_to(stage);const auto writes=f.writes();
  f.now=f.servo.transaction_deadline();f.step();
  REQUIRE(f.servo.state()==as::RuntimeServoState::Fault && !f.servo.ready(f.now) && !f.servo.engaged());
  REQUIRE(f.servo.transaction_error()==as::Error::Deadline);
  f.ack(stage==0?2:0,2100);REQUIRE(f.writes()==writes);
  // 외부 drain/무송신 확보를 합성으로 가정한다. 원 ARM 세대는 쓰기 재개 금지.
  f.now+=as::Transaction::timeout_us;REQUIRE(f.servo.reset_after_drain(f.now));f.step();
  REQUIRE(f.address()==33);const auto requests=f.requests.size();
  f.ack();REQUIRE(f.servo.transaction_phase()==as::Phase::Waiting && f.requests.size()==requests);
  f.ack(1,0);REQUIRE(f.address()==56);f.ack();REQUIRE(f.servo.transaction_phase()==as::Phase::Waiting);
  f.ack(2,2100);REQUIRE(f.servo.ready(f.now) && !f.servo.engaged());
  REQUIRE(f.writes()==writes);f.step();REQUIRE(f.writes()==writes);
  f.snap.armed=false;f.step();f.snap.armed=true;++f.snap.generation;f.step();REQUIRE(f.address()==56);
}
void invalid(const std::string& kind) {
  auto cfg=config();
  if(kind=="profile")cfg.zero.straight_verified=false;
  if(kind=="default")cfg=as::RuntimeServoConfig{};
  if(kind=="speed")cfg.goal_speed=401;
  if(kind=="range")cfg.zero.minimum=1900;
  Fixture f(cfg);
  if(kind=="profile"||kind=="default"||kind=="speed"||kind=="range"){
    f.step();REQUIRE(f.requests.empty() && !f.servo.ready(f.now));return;
  }
  if(kind=="mode"){f.step();f.ack(1,1);}
  else if(kind=="position"){f.step();f.ack(1,0);f.ack(2,999);}
  else if(kind=="engage_position"){f.engage_to(0);f.ack(2,3001);}
  else if(kind=="status"){f.step();f.ack(1,0,1);}
  else if(kind=="goal"){f.engage();f.snap.drive_seen=true;f.snap.steering_rad=.5;f.step();}
  else if(kind=="clock"){f.probe();f.now=-1;f.step();}
  else {REQUIRE(false);}
  REQUIRE(f.servo.state()==as::RuntimeServoState::Fault && !f.servo.ready(f.now) && !f.servo.engaged());
}
void freshness() {
  Fixture f;f.probe();const auto observed=f.servo.position_observed_us();
  REQUIRE(f.servo.ready(observed+f.cfg.observation_max_age_us-1));
  REQUIRE(!f.servo.ready(observed+f.cfg.observation_max_age_us));
  REQUIRE(!f.servo.ready(observed-1));
}
void stopped_goal() {
  Fixture f;f.engage();f.snap.drive_seen=true;f.snap.steering_rad=.2;f.step();
  const auto writes=f.writes();f.snap.armed=false;f.snap.drive_seen=false;++f.snap.generation;
  f.ack();REQUIRE(!f.servo.engaged() && f.writes()==writes);
  f.now+=f.cfg.position_period_us;f.step();REQUIRE(f.address()==56);
  f.ack(2,2100);REQUIRE(f.writes()==writes); // 중앙 및 torque(0) 없음
}
int main(int argc,char** argv) {
  if(argc!=2)return 2;
  const std::string name=argv[1];
  if(name=="basic")basic();else if(name=="latest")latest();else if(name=="partial")partial();
  else if(name=="backpressure_timeout")backpressure_timeout();
  else if(name=="freshness")freshness();else if(name=="stopped_goal")stopped_goal();
  else if(name.rfind("stop",0)==0)boundary(unsigned(name.back()-'0'),false);
  else if(name.rfind("generation",0)==0)boundary(unsigned(name.back()-'0'),true);
  else if(name.rfind("unsent",0)==0)unsent(unsigned(name.back()-'0'));
  else if(name.rfind("timeout",0)==0)timeout(unsigned(name.back()-'0'));
  else if(name.rfind("invalid_",0)==0)invalid(name.substr(8));else return 2;
  std::printf("%s: PASS\n",name.c_str());return 0;
}
