// portable 결합 합성 검사. ESP-IDF main/LEDCPWM/실제 시각/포트는 실행하지 않는다.
#include "board_profile.hpp"
#include "actuation_motor_endpoint.hpp"
#include "runtime_tx.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <initializer_list>

namespace aa=arena_actuation;
namespace as=arena_servo;
namespace st=arena_steering;
#define REQUIRE(c) do {if(!(c)){std::fprintf(stderr,"line %d: %s\n",__LINE__,#c);std::exit(1);}}while(false)

RuntimeBoardProfile synthetic_profile(){
  RuntimeBoardProfile p;
  p.encoder_direction=-1;p.motor_direction_confirmed=true;p.forward_dir_high=true;
  p.maximum_duty=.4;p.kp=.6;p.ki=1.2;p.maximum_speed_mps=2.;p.feedback_period_us=20000;
  p.servo.id=1;p.servo.goal_speed=120;p.servo.goal_acceleration=10;
  p.servo.position_period_us=100000;p.servo.observation_max_age_us=200000;
  p.servo.command_limit=.4;p.servo.wheelbase=.128;p.servo.track=.1239;p.servo.wheel_limit=.5235987755982988;
  p.servo.zero={2000,1000,3000,1,4096,true,true,true,true,true};return p;
}
aa::TxConfig tx_config(){aa::TxConfig c;c.offer_period_us=20000;c.max_consecutive_events=2;
  c.offer_status_v1=true;c.offer_motor_feedback_v1=true;return c;}
constexpr std::uint32_t boot[]={'r','3','8','7'};
std::vector<std::uint8_t> response(unsigned size,unsigned value=0,unsigned status=0){
  std::vector<std::uint8_t> r={255,255,1,static_cast<std::uint8_t>(size+2),static_cast<std::uint8_t>(status)};
  for(unsigned i=0;i<size;++i){r.push_back(static_cast<std::uint8_t>(value>>(8*i)));}
  unsigned sum=0;for(std::size_t i=2;i<r.size();++i){sum+=r[i];}
  r.push_back(static_cast<std::uint8_t>(~sum));return r;
}
std::int64_t integer_field(const std::string& value,const char* key){
  const auto where=value.find(std::string("\"")+key+"\":");REQUIRE(where!=std::string::npos);
  return std::strtoll(value.c_str()+where+std::strlen(key)+3,nullptr,10);
}
struct Fixture {
  st::CadMapping map{st::kCadSamples,sizeof(st::kCadSamples)/sizeof(st::kCadSamples[0])};
  RuntimeBoardProfile profile;
  aa::ActuationMotorEndpoint endpoint;
  as::RuntimeServo servo;
  RuntimeTx tx;
  aa::FrameDecoder host_decoder;
  std::vector<std::vector<std::uint8_t>> requests;
  std::vector<std::uint8_t> servo_bytes;
  std::vector<std::string> offers;
  aa::MotorCyclePreview cycle{};
  aa::Md20aForwardCommand command{0.,false,aa::Md20aMapStatus::kInhibited};
  std::int64_t now=0,feedback_seq=0,last_feedback=-20000,lease=0,lease_issued=0,offer_generation=0;
  unsigned request_seq=0;
  explicit Fixture(RuntimeBoardProfile p=synthetic_profile()):profile(p),
    endpoint(profile.endpoint_receiver_config(map),boot,4,30000,aa::PayloadLayout::kEnvelope,tx_config(),
             profile.kp,profile.ki,.05,0.,profile.maximum_duty),servo(map,profile.servo){endpoint.connect(0);}
  bool ready()const{return profile.valid(map)&&servo.ready(now);}
  void feedback(std::int64_t raw_count=0){
    REQUIRE(endpoint.motor_feedback(now,aa::IntInput::of(feedback_seq++),aa::IntInput::of(now),
      aa::IntInput::of(raw_count*profile.encoder_direction)));last_feedback=now;
  }
  void host_output(){
    REQUIRE(tx.pump_bounded(now,endpoint,[&](const std::uint8_t* p,std::size_t n)->std::size_t{
      const auto take=std::min(n,aa::kMaxInputPerCall);
      REQUIRE(host_decoder.feed(p,take,[&](const std::uint8_t* payload,std::size_t size){
        const std::string text(reinterpret_cast<const char*>(payload),size);
        if(text.find("\"type\":\"offer\"")!=std::string::npos){
          offers.push_back(text);lease=integer_field(text,"lease");lease_issued=integer_field(text,"lease_issued_us");
          offer_generation=integer_field(text,"generation");
        }
      }));return take;
    },ready()));
  }
  void step(const std::vector<std::uint8_t>& rx={},bool control=true,bool pump=true){
    endpoint.check_deadlines(now,ready());endpoint.poll(now,ready());
    const auto& r=endpoint.receiver();
    servo.step(now,{r.armed(),r.generation(),r.drive_seen(),r.steering()},rx.data(),rx.size(),
      [&](const std::uint8_t* bytes,std::size_t n)->std::size_t{
        servo_bytes.insert(servo_bytes.end(),bytes,bytes+n);
        while(servo_bytes.size()>=4 && servo_bytes.size()>=static_cast<std::size_t>(servo_bytes[3])+4){
          const auto size=static_cast<std::size_t>(servo_bytes[3])+4;
          requests.emplace_back(servo_bytes.begin(),servo_bytes.begin()+size);
          servo_bytes.erase(servo_bytes.begin(),servo_bytes.begin()+size);
        }return n;
      });
    if(control){cycle=endpoint.control_tick(now,ready(),!servo.engaged());
      command=aa::md20a_forward_command(cycle,endpoint.receiver(),profile.motor_profile(),ready()&&servo.engaged());}
    if(!ready()||!endpoint.receiver().armed()||!servo.engaged()||endpoint.receiver().target_speed()==0)
      command.duty_ratio=0.;
    if(pump)host_output();
  }
  const std::vector<std::uint8_t>& last()const{REQUIRE(!requests.empty());return requests.back();}
  unsigned address()const{return last()[5];}
  unsigned goal()const{REQUIRE(address()==41);return last()[7]|(unsigned(last()[8])<<8);}
  std::size_t writes()const{std::size_t n=0;for(const auto& p:requests){if(p[4]==3)++n;}return n;}
  void send(const char* kind,double speed=0.,double steering=0.,std::int64_t ttl=100000){
    char data[512];++request_seq;
    int n;
    if(std::strcmp(kind,"STOP")==0){
      n=std::snprintf(data,sizeof(data),"{\"type\":\"command\",\"request\":\"r%u\",\"command\":{\"v\":1,\"kind\":\"STOP\"}}",request_seq);
    }else{
      char drive[180]{};
      if(std::strcmp(kind,"DRIVE")==0)std::snprintf(drive,sizeof(drive),
        ",\"speed_mps\":%.17g,\"steering_rad\":%.17g,\"source_age_us\":0,\"ttl_us\":%lld",speed,steering,static_cast<long long>(ttl));
      n=std::snprintf(data,sizeof(data),"{\"type\":\"command\",\"request\":\"r%u\",\"command\":{\"v\":1,\"kind\":\"%s\","
        "\"boot\":\"r387\",\"generation\":%lld,\"lease\":%lld,\"client\":\"review\",\"seq\":%u%s}}",
        request_seq,kind,static_cast<long long>(offer_generation),static_cast<long long>(lease),request_seq,drive);
    }
    REQUIRE(n>0&&static_cast<std::size_t>(n)<sizeof(data));
    std::vector<std::uint8_t> wire(static_cast<std::size_t>(n)+aa::kFrameOverhead);
    REQUIRE(aa::encode_frame(reinterpret_cast<const std::uint8_t*>(data),n,wire.data(),wire.size())==wire.size());
    for(std::size_t at=0;at<wire.size();){const auto size=std::min(aa::kMaxInputPerCall,wire.size()-at);
      endpoint.feed(now,wire.data()+at,size,ready());at+=size;}
  }
  void prepare(){
    for(now=0;now<=240000;now+=5000){
      if(now-last_feedback>=profile.feedback_period_us)feedback();
      std::vector<std::uint8_t> rx;
      if(servo.transaction_phase()==as::Phase::Waiting){
        if(address()==33)rx=response(1,0);else if(address()==56)rx=response(2,2100);else REQUIRE(false);
      }
      step(rx);
    }
    now=245000;REQUIRE(ready()&&!servo.engaged()&&writes()==0);
    REQUIRE(!offers.empty()&&offers.back().find("\"output_ready\":true")!=std::string::npos);
  }
  void arm(){send("ARM");step();REQUIRE(endpoint.receiver().state()==aa::ReceiverState::kArmed);
    REQUIRE(address()==56&&!servo.engaged());now+=5000;}
  void finish_engage(){
    step(response(2,2100));REQUIRE(goal()==2100&&!servo.engaged());now+=5000;
    step(response(0));REQUIRE(address()==40&&last()[6]==1&&!servo.engaged());now+=5000;
    if(now-last_feedback>=profile.feedback_period_us)feedback();
    step(response(0));REQUIRE(servo.engaged());now+=5000;
  }
  void active(){prepare();arm();finish_engage();send("DRIVE",.5,.2);step();
    REQUIRE(endpoint.receiver().state()==aa::ReceiverState::kActive&&command.duty_ratio>0);now+=5000;}
};

void default_invalid(){Fixture f(installed_board_profile());f.step();REQUIRE(!f.ready()&&f.requests.empty());
  REQUIRE(!f.profile.valid(f.map) && !f.profile.receiver_config().valid());
  REQUIRE(f.profile.endpoint_receiver_config(f.map).valid());
  REQUIRE(!f.offers.empty() && f.offers.back().find("\"output_ready\":false")!=std::string::npos);
  f.now+=5000;f.send("ARM");f.step();
  REQUIRE(!f.endpoint.receiver().armed()&&f.command.duty_ratio==0&&f.requests.empty());}
void engagement(){Fixture f;f.prepare();f.arm();const auto held=f.writes();
  f.send("DRIVE",.5,.2);f.step(response(2,2100));REQUIRE(f.goal()==2100);
  REQUIRE(f.command.duty_ratio==0&&std::strcmp(f.cycle.reason,"actuator_preparing")==0);f.now+=5000;
  f.step(response(0));REQUIRE(f.command.duty_ratio==0&&!f.servo.engaged());f.now+=5000;
  f.feedback();f.step(response(0));REQUIRE(f.servo.engaged()&&f.command.duty_ratio==.303);
  REQUIRE(f.writes()==held+3); // hold, torque, 실제 DRIVE의 CAD goal
  f.now+=5000;f.send("DRIVE",0.,-.2);f.step(response(0));
  REQUIRE(f.endpoint.receiver().state()==aa::ReceiverState::kArmed&&f.endpoint.receiver().drive_seen());
  REQUIRE(f.command.duty_ratio==0&&f.address()==41&&f.goal()!=2100&&f.servo.engaged());}
void arm_no_center(){Fixture f;f.prepare();f.arm();f.finish_engage();const auto writes=f.writes();
  for(int i=0;i<3;++i){f.step();f.now+=5000;}REQUIRE(f.writes()==writes&&writes==2&&f.command.duty_ratio==0);}
void stop_late_ack(){Fixture f;f.prepare();f.arm();f.step(response(2,2100));f.now+=5000;
  f.step(response(0));REQUIRE(f.address()==40);const auto writes=f.writes();f.now+=5000;
  f.send("STOP");f.step(response(0));REQUIRE(f.endpoint.receiver().state()==aa::ReceiverState::kEstop);
  REQUIRE(!f.servo.engaged()&&f.writes()==writes&&f.command.duty_ratio==0);}
void expired_late_ack(){Fixture f;f.prepare();f.arm();
  const auto ttl=f.now+13000-f.lease_issued;f.send("DRIVE",.5,.2,ttl);f.step(response(2,2100));f.now+=5000;
  f.step(response(0));REQUIRE(f.address()==40);f.now+=5000;f.feedback();f.step();
  const auto writes=f.writes();f.now=f.endpoint.receiver().deadline()+1;
  f.step(response(0),false,false); // 5 ms tick/새 offer 없이 deadline+1 us의 ACK
  REQUIRE(std::strcmp(f.endpoint.receiver().reason(),"command_expired")==0);
  REQUIRE(!f.servo.engaged()&&f.writes()==writes&&f.command.duty_ratio==0);}
void control_gap(){Fixture f;f.prepare();f.arm();const auto start=f.now-5000;
  // servo ACK나 제어 tick 없이 check_deadlines만 반복한다. feedback은 계속 새것이다.
  for(f.now=start+1000;f.now<=start+51000;f.now+=1000){
    if(f.now-f.last_feedback>=20000){f.feedback();}
    f.endpoint.check_deadlines(f.now,f.ready());
  }
  REQUIRE(std::strcmp(f.endpoint.receiver().reason(),"control_deadline_missed")==0);
}
void negative_feedback(){Fixture f;f.active();f.step(response(0));f.now+=5000;
  while(f.now-f.last_feedback<20000){f.step();f.now+=5000;}
  f.feedback(12); // 프로필 방향 -1 적용 후 음수 속도
  f.step();REQUIRE(f.endpoint.receiver().mean_mps()<0&&f.command.duty_ratio>0&&f.command.dir_high);
  REQUIRE(f.command.duty_ratio<=f.profile.maximum_duty);
}
void servo_fault_recovery(){Fixture f;f.active();const auto writes=f.writes();
  f.step(response(0,0,1));REQUIRE(!f.ready()&&f.command.duty_ratio==0);
  REQUIRE(std::strcmp(f.endpoint.receiver().reason(),"output_unavailable")==0);
  REQUIRE(!f.servo.engaged());f.now+=5000;
  for(int i=0;i<6;++i){if(f.now-f.last_feedback>=20000){f.feedback();}f.step();f.now+=5000;}
  REQUIRE(f.servo.reset_after_drain(f.now));f.step();REQUIRE(f.address()==33);f.now+=5000;
  f.step(response(1,0));f.now+=5000;
  if(f.now-f.last_feedback>=20000){f.feedback();}
  f.step(response(2,2100));
  REQUIRE(f.ready()&&!f.servo.engaged()&&f.writes()==writes);
  REQUIRE(f.endpoint.receiver().state()==aa::ReceiverState::kFault&&f.command.duty_ratio==0);f.now+=5000;
  f.send("DRIVE",.5,.2);f.step();REQUIRE(!f.endpoint.receiver().armed());f.now+=5000;
  f.send("CLEAR");f.step();REQUIRE(f.endpoint.receiver().state()==aa::ReceiverState::kDisarmed);f.now+=5000;
  f.send("ARM");f.step();REQUIRE(f.endpoint.receiver().state()==aa::ReceiverState::kArmed&&!f.servo.engaged());
}
void profile_gains(){Fixture f;auto p=f.profile;p.ki=0;REQUIRE(!p.valid(f.map));
  p=f.profile;p.kp=0;REQUIRE(!p.valid(f.map));p=f.profile;p.encoder_direction=0;REQUIRE(!p.valid(f.map));}
void profile_geometry(){Fixture f;auto p=f.profile;p.servo.command_limit=.8;REQUIRE(!p.valid(f.map));
  p=f.profile;p.servo.wheel_limit=.2;REQUIRE(!p.valid(f.map));}
void quantization(){Fixture f;const auto c=f.profile.receiver_config();
  const double metres=6.283185307179586*c.wheel_radius_m/(c.ticks_per_rev*c.gear_ratio);
  REQUIRE(metres/.02>.03&&metres/.02<.031);
  for(double value:{0.,.0001,.01,.1,.4,1.}){const auto raw=static_cast<unsigned>(std::floor(value*1023.));
    REQUIRE(raw<=1023&&static_cast<double>(raw)/1024.<=value);}
}
int main(int argc,char** argv){
  if(argc!=2)return 2;
  const std::string name=argv[1];
  if(name=="default")default_invalid();else if(name=="engagement")engagement();else if(name=="arm_no_center")arm_no_center();
  else if(name=="stop_late_ack")stop_late_ack();else if(name=="expired_late_ack")expired_late_ack();
  else if(name=="control_gap")control_gap();else if(name=="negative_feedback")negative_feedback();
  else if(name=="servo_fault_recovery")servo_fault_recovery();else if(name=="profile_gains")profile_gains();
  else if(name=="profile_geometry")profile_geometry();else if(name=="quantization")quantization();else return 2;
  std::printf("%s: PASS\n",name.c_str());return 0;}
