#include "cad_steering_table.hpp"
#include "cad_servo_goal.hpp"
#include "steering_mapping.hpp"
#include "st3215_transaction.hpp"
#include <limits>
#include <stdexcept>
#include <vector>

void Require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
int main(){
  using namespace arena_steering;
  CadMapping cad(kCadSamples,kCadSampleCount);
  Require(cad.valid(),"96 row source table valid");
  for(const auto& row:kCadSamples){
    const auto value=cad.relative(row.servo_rad);
    Require(value.valid && std::abs(value.left_rad-row.left_rad)<1e-12 &&
      std::abs(value.right_rad-row.right_rad)<1e-12,"CAD knot preservation");
  }
  const double limit=std::atan(.128/(.128/std::tan(.45)+.1239/2));
  for(int i=-500;i<=500;++i){
    const double requested=limit*i/500;
    const auto goal=cad.central(requested,.128,.1239,.45);
    Require(goal.valid && std::abs(goal.equivalent_rad-requested)<1e-11,"inverse equivalent angle");
    Require(std::max(std::abs(goal.left_rad),std::abs(goal.right_rad))<=.45,"inner cap unchanged");
  }
  Require(cad.central(100,.128,.1239,.45).clamped,"central saturation");
  Require(cad.relative(100).clamped,"no extrapolation");
  for(const double sign:{-1.,1.}){
    const auto end=cad.central(sign,.128,.1239,.5235987755982988);
    const auto row=sign>0?kCadSamples[0]:kCadSamples[kCadSampleCount-1];
    Require(std::abs(end.servo_rad-row.servo_rad)<1e-11,"full CAD endpoint, no legacy Ackermann cap");
    Require(std::abs(end.left_rad-row.left_rad)<1e-11 &&
      std::abs(end.right_rad-row.right_rad)<1e-11,"left/right endpoint relation preserved");
  }
  Require(!cad.central(std::numeric_limits<double>::quiet_NaN(),.128,.1239,.45).valid,"NaN rejected");
  Require(!ZeroProfile{}.goal(0).valid,"pending calibration has no target");
  ZeroProfile zero{2000,1000,3000,1,4096,true,true,true,true,true};
  Require(zero.goal(0).position==2000,"software straight offset");
  zero.direction=-1;
  Require(zero.goal(.1).position<2000 && zero.goal(.1).relative_rad>0,"reverse mount sign");
  auto target=ResolveCadServoGoal(cad,zero,.1,limit,.128,.1239,.45);
  Require(target.valid,"CAD to checked software offset");
  Require(!ResolveCadServoGoal(cad,zero,limit+.01,limit,.128,.1239,.45).valid,"wire command cap remains strict");
  const double full_limit=std::min(cad.central(1,.128,.1239,.5235987755982988).equivalent_rad,
    -cad.central(-1,.128,.1239,.5235987755982988).equivalent_rad);
  for(int direction:{-1,1}){
    zero.direction=direction;
    for(int i=-100;i<=100;++i)
      Require(ResolveCadServoGoal(cad,zero,full_limit*i/100,full_limit,.128,.1239,.5235987755982988).valid,
        "full design commands round inward without false rejection");
  }
  zero.zero=4090;zero.minimum=0;zero.maximum=4095;
  Require(!zero.covers(-.3,.3),"near-wrap range unavailable");
  Require(!ResolveCadServoGoal(cad,zero,0,limit,.128,.1239,.45).valid,"insufficient full range refuses goal");
  zero.zero=0;Require(!zero.valid(),"boundary zero retained in storage but not bidirectional runtime profile");
  // r319 실측 경로는 별도 그대로 유지한다. 아래 값은 합성값이다.
  Sample measured[]={{-.3,1700},{0,2000},{.3,2300}};
  Mapping mapping(measured,3);
  Require(mapping.goal(.1).valid && mapping.goal(.1).position==2100,"r319 measured path unchanged");
  Require(!mapping.goal(.4).valid,"r319 outside measured range still rejects");
  arena_servo::Transaction tx;
  Require(tx.goal(1000,1,target.servo.position,200,10),"existing RAM-only serializer");
  std::vector<unsigned char> packet;
  tx.pump(1001,[&](const unsigned char* bytes,std::size_t size){
    packet.insert(packet.end(),bytes,bytes+size);return size;});
  Require(packet.size()==14 && packet[5]==41 &&
    (packet[7]|(unsigned(packet[8])<<8))==target.servo.position,"RAM target payload");
}
