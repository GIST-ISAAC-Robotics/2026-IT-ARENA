#include "hal.hpp"
#include "profile.hpp"
#include "endpoint.hpp"
#include "external_servo.hpp"
#include "bench.hpp"
#include "encoder.hpp"
#include "runtime_tx.hpp"
#include "md20a_forward_output.hpp"
#include ARENA_PROFILE_HEADER
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <zephyr/kernel.h>

namespace aa=arena_actuation;
static RuntimeTx tx;
static unoq::ExternalServo servo;
static unoq::Bench bench;
static aa::FrameLink input(30000);
static std::uint8_t wire[aa::kMaxFrame],rx[256],observation[1024];
static char json[1000];

int main(){
  if(!unoq::hal_init()){
    unoq::fail_hal();for(;;)k_msleep(10);
  }
  const auto boot=unoq::boot_random();std::uint32_t identity[8];char boot_text[9]{};
  std::snprintf(boot_text,sizeof(boot_text),"%08x",unsigned(boot));
  for(unsigned i=0;i<8;++i)identity[i]=boot_text[i];
  static const auto profile=selected_profile();
  aa::TxConfig tc;tc.offer_status_v1=true;tc.offer_motor_feedback_v1=true;
  tc.offer_period_us=40000;tc.diag_period_us=1000000;tc.max_consecutive_events=2;tc.line_ns_per_byte=10000;
  static unoq::Endpoint endpoint(profile.config(),identity,tc,profile.kp,profile.ki,profile.maximum_duty,profile.installed());
  endpoint.connect(unoq::now_us());input.connect(unoq::now_us());
  unoq::imu_start(boot);
  std::int64_t next_control=0,next_feedback=0,next_report=0,feedback_seq=0,counts=0;
  std::int64_t last_accepted=-2,last_generation=-1,bench_count=0,bench_progress_at=0;
  std::int64_t stationary_since=-1;
  std::int64_t observation_seq=-1,capture_us=0;
  std::uint32_t raw=0,modulus=0;bool was_bench_running=false;
  unoq::EncoderCounter encoder;
  std::uint64_t errors_seen=0;double last_duty=0;
  auto hardware_ready=[&](){return !unoq::hal_failed() && encoder.valid() && endpoint.receiver().fresh_feedback(unoq::now_us());};
  auto runtime_ready=[&](){return profile.installed()&&hardware_ready()&&unoq::imu_ready()&&servo.fresh(unoq::now_us(),endpoint.receiver());};
  auto sink=[](const std::uint8_t* p,std::size_t n){return unoq::uart_write(p,n);};
  auto frame=[&](const std::uint8_t* p,std::size_t n,aa::FrameOrigin origin){
    const auto now=unoq::now_us();const auto old_generation=endpoint.receiver().generation();const bool old_armed=endpoint.receiver().armed();
    if(n>=4 && std::memcmp(p,"ESV1",4)==0){
      if(origin==aa::FrameOrigin::kLive)servo.accept(p,n,now,endpoint.receiver());else servo.invalidate();
    }else if(n>=4 && std::memcmp(p,"BNV1",4)==0){
      const bool stationary=stationary_since>=0 && now-stationary_since>=200000;
      const bool starting=n==44 && p[36]==1;
      if(origin==aa::FrameOrigin::kLive)
        bench.accept(p,n,now,endpoint.receiver(),profile.mode==unoq::Mode::Bench,
                     hardware_ready()&&(!starting||stationary));
      else bench.stop("expired_frame");
    }else{
      const auto bytes=aa::encode_frame(p,n,wire,sizeof(wire));
      for(std::size_t offset=0;offset<bytes;){
        auto size=std::min<std::size_t>(256,bytes-offset);
        endpoint.feed(now,wire+offset,size,runtime_ready());offset+=size;
      }
    }
    servo.transition(old_generation,old_armed,endpoint.receiver());
    if(endpoint.receiver().generation()!=old_generation){bench.stop("session_changed");next_report=0;}
    if(endpoint.receiver().accepted_seq()!=last_accepted)next_report=0;
  };
  for(;;){
    auto now=unoq::now_us();
    if(unoq::guard_tripped()){
      servo.invalidate();
      if(!bench.handle_output_deadline(now,endpoint.receiver(),hardware_ready())){
        bench.stop("deadline_guard");endpoint.fault(now);
      }
      unoq::motor_zero();last_duty=0;
    }
    const auto old_generation=endpoint.receiver().generation();const bool old_armed=endpoint.receiver().armed();
    endpoint.check_deadlines(now,runtime_ready());
    servo.transition(old_generation,old_armed,endpoint.receiver());
    const auto size=unoq::uart_read(rx,sizeof(rx));
    input.feed(now,rx,size,frame);input.poll(now,frame);endpoint.poll(unoq::now_us(),runtime_ready());
    if(input.framing_errors()!=errors_seen){
      errors_seen=input.framing_errors();servo.invalidate();bench.stop("framing_error");unoq::motor_zero();last_duty=0;
    }
    now=unoq::now_us();
    if(now>=next_feedback){
      if(!unoq::encoder_read(raw,modulus))unoq::fail_hal();
      else {
        const int sign=profile.installed()?profile.encoder_direction:1; // bench exposes raw timer sign.
        if(!encoder.sample(raw,modulus,sign))unoq::fail_hal();
        counts=encoder.count();
        if(!endpoint.feedback(now,feedback_seq++,counts)){
          bench.stop("encoder_invalid");unoq::motor_zero();
        }else if(!unoq::hal_failed()){observation_seq=feedback_seq-1;capture_us=now;}
      }
      if(endpoint.receiver().fresh_feedback(now)&&std::fabs(endpoint.receiver().mean_mps())<.03){
        if(stationary_since<0)stationary_since=now;
      }else stationary_since=-1;
      next_feedback=now+profile.feedback_period_us;
    }
    if(now>=next_control){
      const bool ready=runtime_ready();
      auto cycle=endpoint.tick(now,ready,!servo.applied(now,endpoint.receiver()));
      bench.tick(now,endpoint.receiver(),hardware_ready());
      if(bench.running()){
        if(!was_bench_running){bench_count=counts;bench_progress_at=now;}
        if(counts!=bench_count){bench_count=counts;bench_progress_at=now;}
        if(now-bench_progress_at>=300000)bench.stop("encoder_no_progress");
      }
      was_bench_running=bench.running();
      if(unoq::hal_failed()){
        endpoint.fault(now);bench.stop("hal_error");servo.invalidate();unoq::motor_zero();last_duty=0;
      }else if(bench.running()){
        last_duty=bench.duty();unoq::motor_set(last_duty,bench.dir(),std::min(now+50000,bench.deadline()));
      }else if(profile.installed()){
        const auto cmd=aa::md20a_forward_command(cycle,endpoint.receiver(),{profile.maximum_duty,profile.forward_dir_high},ready);
        last_duty=cmd.status==aa::Md20aMapStatus::kActive?cmd.duty_ratio:0.;
        const auto deadline=std::min({now+50000,endpoint.receiver().deadline(),servo.deadline()});
        unoq::motor_set(last_duty,cmd.dir_high,deadline);
      }else {unoq::motor_zero();last_duty=0;}
      if(!unoq::hal_failed() && !unoq::watchdog_progress())unoq::fail_hal();
      next_control=now+5000;
    }
    for(unsigned i=0;i<8;++i){unoq::ImuFrame f;if(!unoq::imu_take(f))break;tx.imu(&f,sizeof(f));}
    const auto& r=endpoint.receiver();
    if(now>=next_report || r.generation()!=last_generation || r.accepted_seq()!=last_accepted){
      const char* mode=profile.mode==unoq::Mode::Bench?"bench":profile.installed()?"installed":"unconfigured";
      const int n=std::snprintf(json,sizeof(json),
        "{\"type\":\"uno_q\",\"v\":1,\"boot\":\"%s\",\"generation\":%lld,\"accepted_seq\":%lld,\"state\":\"%s\",\"reason\":\"%s\","
        "\"bench_state\":\"%s\",\"bench_seq\":%lld,\"bench_reason\":\"%s\",\"mode\":\"%s\",\"profile_sha256\":\"%s\","
        "\"observation_seq\":%lld,\"capture_us\":%lld,\"motor_counts\":%lld,\"raw_count\":%u,"
        "\"pwm_duty_ppm\":%u,\"hal_fault\":%s,\"imu_ready\":%s,\"servo_ready\":%s,\"servo_applied_seq\":%lld,"
        "\"uart_errors\":%u,\"imu_dropped\":%u,\"tx_imu_dropped\":%llu}",
        boot_text,(long long)r.generation(),(long long)r.accepted_seq(),aa::tx_detail::state_name(r.state()),r.reason(),
        bench.running()?"running":bench.armed()?"armed":"disarmed",(long long)bench.sequence(),bench.reason(),mode,selected_profile_sha256,
        (long long)observation_seq,(long long)capture_us,
        (long long)counts,unsigned(raw),unsigned(last_duty*1000000),unoq::hal_failed()?"true":"false",unoq::imu_ready()?"true":"false",
        servo.fresh(now,r)?"true":"false",(long long)servo.applied_seq(),unsigned(unoq::uart_errors()),unsigned(unoq::imu_dropped()),
        (unsigned long long)tx.imu_dropped());
      if(n>0 && std::size_t(n)<sizeof(json)){
        const auto framed=aa::encode_frame(reinterpret_cast<const std::uint8_t*>(json),n,observation,sizeof(observation));
        if(tx.observation(observation,framed)){next_report=now+20000;last_generation=r.generation();last_accepted=r.accepted_seq();}
      }
    }
    if(!tx.pump_bounded(now,endpoint,sink,runtime_ready()))unoq::fail_hal();
    k_usleep(250);
  }
}
