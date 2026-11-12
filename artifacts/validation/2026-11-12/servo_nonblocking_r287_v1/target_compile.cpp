// Target object only. No hardware I/O or firmware entrypoint.
#include "st3215_transaction.hpp"
static arena_servo::Transaction transaction;
extern "C" bool compile_servo_begin(std::int64_t now,unsigned mode,unsigned id,unsigned position,unsigned speed,unsigned acceleration){
  if(mode==0)return transaction.read_position(now,id);
  if(mode==1)return transaction.read_mode(now,id);
  if(mode==2)return transaction.torque(now,id,false);
  if(mode==3)return transaction.torque(now,id,true);
  return transaction.goal(now,id,position,speed,acceleration);
}
extern "C" void compile_servo_send(std::int64_t now,std::uint8_t* bytes,std::size_t capacity){
  transaction.pump(now,[&](const std::uint8_t* p,std::size_t size){
    const auto n=std::min(size,capacity);std::memcpy(bytes,p,n);return n;
  });
}
extern "C" int compile_servo_receive(std::int64_t now,const std::uint8_t* bytes,std::size_t size){
  transaction.feed(now,bytes,size);
  return int(transaction.phase());
}
extern "C" bool compile_servo_take(arena_servo::Response* out){return transaction.take(*out);}
extern "C" void compile_servo_cancel(){transaction.cancel();}
extern "C" bool compile_servo_reset(std::int64_t now){return transaction.reset_after_drain(now);}
