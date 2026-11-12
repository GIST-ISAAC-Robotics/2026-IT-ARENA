// 합성 입력만 사용한다. GPIO/모터/센서를 열거나 실제 ARM을 보내지 않는다.
#include "fixture_host.hpp"
#include "firmware/main/stop_switch.hpp"
static void require(bool value,const char* message){if(!value)throw message;}
int main(){
  try {
    {Fixture f;StopSwitchInput sw;f.endpoint.connect(0);sw.sample(1,0,f.endpoint);
      require(sw.held() && f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"open boot not latched");
      const auto generation=f.endpoint.receiver().generation();
      for(int t=1000;t<=200000;t+=1000)sw.sample(1,t,f.endpoint);
      require(f.endpoint.receiver().generation()==generation && sw.changes()==1,"held input churned generation");
      sw.sample(0,205000,f.endpoint);
      require(!sw.held() && f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"release auto cleared");}
    {Fixture f;StopSwitchInput sw;f.endpoint.connect(0);sw.sample(0,0,f.endpoint);
      require(!sw.held() && f.endpoint.receiver().state()==aa::ReceiverState::kDisarmed,"closed boot altered state");}
    {Fixture f;StopSwitchInput sw;require(active(f.start()),"synthetic drive did not start");
      sw.sample(0,225000,f.endpoint);sw.sample(1,226000,f.endpoint);
      require(f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"open not immediate latch");
      f.feedback(230000);require(halted(f.endpoint.control_tick(230000,true)),"latched cycle not zero/brake");
      f.send(235000,"CLEAR");f.feedback(235000);f.endpoint.control_tick(235000,true);
      require(f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"held CLEAR accepted");
      sw.sample(0,236000,f.endpoint);sw.sample(1,237000,f.endpoint);sw.sample(0,238000,f.endpoint);
      f.send(240000,"ARM");f.feedback(240000);require(halted(f.endpoint.control_tick(240000,true)),"bounce auto armed");
      require(f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"release skipped CLEAR");
      f.send(245000,"CLEAR");f.feedback(245000);f.endpoint.control_tick(245000,true);
      require(f.endpoint.receiver().state()==aa::ReceiverState::kDisarmed,"explicit CLEAR failed");
      f.send(250000,"ARM");f.feedback(250000);f.endpoint.control_tick(250000,true);
      f.send(255000,"DRIVE",.5);f.feedback(255000);
      require(active(f.endpoint.control_tick(255000,true)),"explicit rearm failed");}
    {Fixture f;StopSwitchInput sw;f.endpoint.connect(0);sw.sample(-1,0,f.endpoint);
      require(sw.held() && f.endpoint.receiver().state()==aa::ReceiverState::kEstop,"invalid read allowed");}
    std::puts("PASS: open/closed boot, held generation, active stop, held CLEAR, release/bounce, explicit rearm, invalid input");
    return 0;
  } catch(const char* reason){std::fprintf(stderr,"FAIL: %s\n",reason);return 1;}
}
