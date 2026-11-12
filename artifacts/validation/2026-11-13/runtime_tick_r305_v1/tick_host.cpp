#include "fixed_tick.hpp"
#include <cstdio>
#include <stdexcept>
static void require(bool v){if(!v)throw std::runtime_error("tick schedule failure");}
int main(){
  FixedTick t(100);
  require(!t.poll(99));require(t.poll(100));require(!t.poll(100));
  require(!t.poll(5099));require(t.poll(5107));require(!t.poll(5108));
  require(t.poll(16100));require(t.missed()==1);require(t.maximum_lateness()==6000);
  require(!t.poll(16100));require(!t.poll(20099));require(t.poll(20100));
  // 같은 task 깨움과 관측 비용에서 완료+5ms는 약166Hz로 밀린다.
  FixedTick phase(0);std::int64_t completion_due=0;unsigned fixed=0,drifting=0;
  for(std::int64_t now=0;now<30000000;now+=1000){
    if(phase.poll(now+100)){++fixed;require(!phase.poll(now+101));}
    if(now+100>=completion_due){++drifting;completion_due=now+104+5000;}
  }
  require(fixed==6000 && drifting==5000 && phase.missed()==0);
  FixedTick stall(0);require(stall.poll(0));require(stall.poll(100000));
  require(stall.missed()==19 && !stall.poll(100000) && stall.poll(105000));
  std::printf("fixed=%u completion_relative=%u; early,late,skipped,no-catchup checks passed\n",fixed,drifting);
}
