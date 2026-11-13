// 합성 입력의 준비/허가 분리. 장치 출력과 실제 시간 간격은 검증하지 않는다.
#include "motor_cycle_cases.hpp"
#include "runtime_tx.hpp"
#include <cassert>
#include <cstdio>
#include <cstring>

namespace aa = arena_actuation;
using arena_smoke::CycleFixture;
static arena_smoke::Scratch scratch;

struct FakeTx {
  aa::TxFault fault() const { return aa::TxFault::kNone; }
  bool frame_in_flight() const { return false; }
};
struct FakeEndpoint {
  bool observed = false;
  unsigned calls = 0;
  FakeTx tx_;
  template<class Sink> void pump(std::int64_t, Sink&&, bool ready) {
    observed=ready; ++calls;
  }
  const FakeTx& tx() const { return tx_; }
};

int main() {
  {
    CycleFixture f(scratch);
    assert(!f.receiver.drive_seen());
    f.prepare(); assert(f.arm());
    assert(!f.receiver.drive_seen());
    assert(f.drive(225000,0));
    assert(f.receiver.drive_seen() && f.receiver.state()==aa::ReceiverState::kArmed);
    const auto angle=f.receiver.steering();
    assert(f.stop(225000));
    assert(!f.receiver.drive_seen() && f.receiver.steering()==angle);
    assert(f.control("CLEAR",225000));
    assert(f.control("ARM",225000));
    assert(!f.receiver.drive_seen());
  }
  {
    CycleFixture f(scratch); assert(f.start());
    const auto first=f.cycle.tick(225000,true);
    assert(first.motor.effort>0 && f.cycle.integral()>0);
    for(std::int64_t now=230000;now<=260000;now+=5000) {
      assert(f.feedback(now));
      const auto out=f.cycle.tick(now,true,true);
      assert(out.state==aa::ReceiverState::kActive && out.motor.effort==0);
      assert(out.motor.status==aa::PiStatus::kDisabled && f.cycle.integral()==0);
      assert(std::strcmp(out.reason,"actuator_preparing")==0);
    }
    assert(f.feedback(265000));
    const auto resumed=f.cycle.tick(265000,true,false);
    assert(resumed.motor.effort==first.motor.effort && f.receiver.drive_seen());
  }
  {
    CycleFixture f(scratch); assert(f.start());
    for(std::int64_t now=225000;now<325000;now+=5000) {
      assert(f.feedback(now));
      assert(f.cycle.tick(now,true,true).state==aa::ReceiverState::kActive);
    }
    assert(f.feedback(325000));
    const auto expired=f.cycle.tick(325000,true,true);
    assert(arena_smoke::cycle_fault(expired,"command_expired"));
    assert(!f.receiver.drive_seen());
  }
  {
    CycleFixture f(scratch); assert(f.start());
    assert(arena_smoke::cycle_fault(f.cycle.tick(225000,false,true),"output_unavailable"));
  }
  {
    RuntimeTx mux; FakeEndpoint endpoint;
    const auto sink=[](const std::uint8_t*,std::size_t)->std::size_t { return 0; };
    assert(mux.pump(0,endpoint,sink)); assert(endpoint.calls==1 && !endpoint.observed);
    assert(mux.pump_bounded(1,endpoint,sink,true)); assert(endpoint.calls==2 && endpoint.observed);
    assert(mux.pump_bounded(2,endpoint,sink)); assert(endpoint.calls==3 && !endpoint.observed);
  }
  std::puts("drive_generation, preparation_reset, original_expiry, output_loss, ready_forward:PASS");
}
