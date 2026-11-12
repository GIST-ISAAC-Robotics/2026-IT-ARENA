#include "st3215_transaction.hpp"
#include <cstdio>
#include <vector>
#include <string>
using arena_servo::Transaction;
using arena_servo::Phase;
using arena_servo::Error;
using arena_servo::Response;

std::vector<std::uint8_t> response(unsigned id,std::vector<std::uint8_t> data,unsigned error=0){
  std::vector<std::uint8_t> out={255,255,static_cast<std::uint8_t>(id),static_cast<std::uint8_t>(data.size()+2),static_cast<std::uint8_t>(error)};
  out.insert(out.end(),data.begin(),data.end());unsigned sum=0;
  for(std::size_t i=2;i<out.size();++i)sum+=out[i];out.push_back(static_cast<std::uint8_t>(~sum));return out;
}
std::vector<std::uint8_t> send(Transaction& t,std::int64_t now,std::size_t chunk=99){
  std::vector<std::uint8_t> out;
  for(int i=0;i<20 && t.phase()==Phase::Sending;++i)t.pump(now+i,[&](const std::uint8_t* p,std::size_t n){
    n=std::min(n,chunk);out.insert(out.end(),p,p+n);return n;
  });return out;
}
bool run(const std::string& name){
  Transaction t;Response r;
  if(name=="position_fragments"){
    if(!t.read_position(1000,1))return false;
    auto wire=send(t,1001,1);
    if(wire!=std::vector<std::uint8_t>{255,255,1,4,2,56,2,190})return false;
    auto rx=response(1,{0,8});for(std::size_t i=0;i<rx.size();++i)t.feed(2000+i,&rx[i],1);
    return t.take(r) && !t.take(r) && r.started_us==1000 && r.completed_us==2007 && r.data[1]==8 && r.size==2;
  }
  if(name=="echo_then_reply"){
    t.read_position(0,1);auto wire=send(t,1);auto rx=response(1,{10,8});
    t.feed(100,wire.data(),wire.size());if(t.phase()!=Phase::Waiting)return false;
    t.feed(200,rx.data(),rx.size());return t.take(r) && r.data[0]==10;
  }
  if(name=="noise_bad_crc_other_id"){
    t.read_position(0,1);send(t,1);auto wrong=response(2,{1,2});auto bad=response(1,{3,4});bad.back()^=1;
    std::vector<std::uint8_t> rx={255,12,255,255,1,255,0,255};
    rx.insert(rx.end(),wrong.begin(),wrong.end());rx.insert(rx.end(),bad.begin(),bad.end());auto good=response(1,{5,6});rx.insert(rx.end(),good.begin(),good.end());
    t.feed(500,rx.data(),rx.size());return t.take(r) && r.data[0]==5 && r.data[1]==6;
  }
  if(name=="partial_tx_deadline"){
    t.read_position(0,1);int ticks=0;
    for(std::int64_t now=0;now<=30000;now+=5000){t.pump(now,[](const std::uint8_t*,std::size_t){return std::size_t(0);});++ticks;}
    return ticks==7 && t.error()==Error::Deadline && !t.read_mode(30001,1);
  }
  if(name=="no_reply_deadline"){
    t.read_position(0,1);send(t,1);t.tick(29999);if(t.phase()!=Phase::Waiting)return false;
    auto good=response(1,{5,6});t.feed(30000,good.data(),good.size());return t.error()==Error::Deadline && !t.take(r);
  }
  if(name=="cancel_requires_new_link"){
    t.read_position(0,1);send(t,1);t.cancel();auto good=response(1,{5,6});t.feed(100,good.data(),good.size());
    if(t.take(r) || t.read_position(101,1))return false;
    return t.reset_after_drain(200) && t.read_position(201,1);
  }
  if(name=="goal_volatile_encoding"){
    if(t.goal(0,1,4096,200,10) || t.goal(0,1,2000,0,10) || t.goal(0,1,2000,401,10))return false;
    if(!t.goal(0,1,2048,200,10))return false;
    auto wire=send(t,1);unsigned sum=0;for(std::size_t i=2;i<wire.size();++i)sum+=wire[i];
    return wire.size()==14 && wire[5]==41 && wire[6]==10 && wire[7]==0 && wire[8]==8 && wire[9]==0 && wire[10]==0 && wire[11]==200 && wire[12]==0 && (sum&255)==255;
  }
  if(name=="torque_ack"){
    if(!t.torque(0,1,false))return false;auto wire=send(t,1);auto rx=response(1,{});t.feed(100,rx.data(),rx.size());
    return wire[5]==40 && wire[6]==0 && t.take(r) && r.size==0;
  }
  if(name=="servo_error"){
    t.read_mode(0,1);send(t,1);auto rx=response(1,{0},4);t.feed(100,rx.data(),rx.size());return t.error()==Error::ServoStatus && !t.take(r);
  }
  if(name=="busy_keeps_request"){
    t.read_position(0,1);if(t.torque(1,1,true))return false;auto wire=send(t,2);return wire[4]==2 && wire[5]==56;
  }
  if(name=="clock_and_sink"){
    t.read_mode(100,1);t.tick(99);if(t.error()!=Error::InvalidClock || t.reset_after_drain(99))return false;
    t.reset_after_drain(200);t.read_mode(201,1);
    t.pump(202,[](const std::uint8_t*,std::size_t n){return n+1;});return t.error()==Error::SinkContract;
  }
  return false;
}
int main(){
  const char* names[]={"position_fragments","echo_then_reply","noise_bad_crc_other_id","partial_tx_deadline","no_reply_deadline","cancel_requires_new_link","goal_volatile_encoding","torque_ack","servo_error","busy_keeps_request","clock_and_sink"};
  for(const auto name:names){const bool ok=run(name);std::printf("%s %s\n",ok?"PASS":"FAIL",name);if(!ok)return 1;}
}
