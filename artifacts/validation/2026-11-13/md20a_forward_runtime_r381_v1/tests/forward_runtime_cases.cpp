// 합성 receiver/PI 연결만 검사한다. GPIO/포트/장치를 열지 않는다.
#include "motor_cycle_cases.hpp"
#include "actuation_motor_endpoint.hpp"
#include "md20a_forward_output.hpp"
#include <initializer_list>

namespace aa = arena_actuation;
namespace sm = arena_smoke;
#define CHECK(condition) do { if (!(condition)) { \
  std::fprintf(stderr, "line %d: %s\n", __LINE__, #condition); return false; } } while (false)

struct ForwardFixture {
  aa::ActuationReceiverCore receiver;
  aa::MotorControlCandidate cycle;
  sm::Scratch& scratch;
  aa::Md20aForwardProfile profile;
  explicit ForwardFixture(sm::Scratch& s, bool direction = false)
      : receiver(sm::motor_config(), sm::kCycleBoot, 10),
        cycle(receiver, .6, 1.2, .05, 0.0, .4), scratch(s), profile{.4, direction} {}
  bool feedback(std::int64_t now, std::int64_t count = 0) {
    return receiver.update_motor_encoder(now, aa::IntInput::of(now / 5000),
        aa::IntInput::of(now), aa::IntInput::of(count));
  }
  bool control(const char* kind, std::int64_t now, unsigned seq = 1) {
    const auto offer = receiver.offer(now);
    return offer.ok && sm::command(scratch, kind, receiver, offer.lease, seq) &&
           receiver.receive(scratch.command, now).accepted;
  }
  bool drive(std::int64_t now, double speed = .5, unsigned seq = 2) {
    const auto offer = receiver.offer(now);
    if (!offer.ok) return false;
    const int size = std::snprintf(scratch.text, sizeof(scratch.text),
        "{\"v\":1,\"kind\":\"DRIVE\",\"boot\":\"smoke-only\",\"generation\":%lld,\"lease\":%lld,"
        "\"client\":\"smoke-only\",\"seq\":%u,\"speed_mps\":%.17g,\"steering_rad\":0.2,"
        "\"source_age_us\":0,\"ttl_us\":100000}", static_cast<long long>(receiver.generation()),
        static_cast<long long>(offer.lease.id), seq, speed);
    return size > 0 && static_cast<std::size_t>(size) < sizeof(scratch.text) &&
        sm::parse(scratch, scratch.text) && receiver.receive(scratch.command, now).accepted;
  }
  bool start() {
    for (std::int64_t now = 0; now <= 215000; now += 5000) { feedback(now); cycle.tick(now); }
    if (!control("ARM", 220000)) return false;
    if (cycle.tick(220000, true).state != aa::ReceiverState::kArmed) return false;
    return drive(225000);
  }
  aa::Md20aForwardCommand mapped(const aa::MotorCyclePreview& p, bool ready = true) const {
    return aa::md20a_forward_command(p, receiver, profile, ready);
  }
};

bool forward_and_overspeed(sm::Scratch& s, bool direction) {
  ForwardFixture f(s, direction);
  CHECK(f.start());
  // 부호가 교정된 피드백의 역회전에도 전진 duty를 계산한다. DIR는 바뀌지 않는다.
  CHECK(f.feedback(225000, -8));
  const auto reverse = f.cycle.tick(225000, true);
  CHECK(reverse.state == aa::ReceiverState::kActive && reverse.motor.effort == .4);
  CHECK(f.mapped(reverse).status == aa::Md20aMapStatus::kActive);
  CHECK(f.mapped(reverse).dir_high == direction && f.mapped(reverse).duty_ratio == .4);
  const auto generation = f.receiver.generation();
  // +12 counts / 5 ms: 합성 속도가 양수 목표를 초과한다.
  CHECK(f.feedback(230000, 4));
  const auto slowing = f.cycle.tick(230000, true);
  CHECK(slowing.state == aa::ReceiverState::kActive && slowing.motor.status == aa::PiStatus::kNormal);
  CHECK(slowing.motor.effort == 0.0 && !slowing.motor.brake_requested);
  CHECK(f.mapped(slowing).status == aa::Md20aMapStatus::kActive);
  CHECK(f.mapped(slowing).duty_ratio == 0.0 && f.mapped(slowing).dir_high == direction);
  CHECK(f.receiver.generation() == generation && f.receiver.target_speed() == .5);
  CHECK(f.feedback(235000, 4));
  const auto recovered = f.cycle.tick(235000, true);
  CHECK(recovered.state == aa::ReceiverState::kActive && recovered.motor.effort > 0.0);
  CHECK(f.mapped(recovered).dir_high == direction);
  CHECK(f.drive(240000, 0.0, 3));
  const auto zero = f.cycle.tick(240000, true);
  CHECK(zero.state == aa::ReceiverState::kArmed && zero.motor.status == aa::PiStatus::kZeroTarget);
  CHECK(f.cycle.integral() == 0.0 && f.mapped(zero).status == aa::Md20aMapStatus::kInhibited);
  CHECK(f.drive(245000, .5, 4));
  const auto resumed = f.cycle.tick(245000, true);
  CHECK(resumed.state == aa::ReceiverState::kActive && resumed.motor.effort == .303);
  return true;
}

bool mapping_contract(sm::Scratch& s) {
  ForwardFixture f(s);
  aa::MotorCyclePreview fake;
  fake.state = aa::ReceiverState::kActive;
  fake.motor = { .2, false, aa::PiStatus::kNormal };
  CHECK(f.mapped(fake, true).duty_ratio == 0.0);  // bool 하나는 ARM/DRIVE를 만들지 않는다.
  CHECK(f.start());
  const auto p = f.cycle.tick(225000, true);
  CHECK(f.mapped(p).duty_ratio == .303);
  CHECK(f.mapped(p, false).status == aa::Md20aMapStatus::kInhibited);
  CHECK(aa::md20a_forward_command(p, f.receiver, f.profile).duty_ratio == 0.0);
  for (const double effort : {-0.1, .5, std::numeric_limits<double>::quiet_NaN()}) {
    auto bad = p; bad.motor.effort = effort;
    CHECK(f.mapped(bad).status == aa::Md20aMapStatus::kInvalidIntent);
    CHECK(f.mapped(bad).duty_ratio == 0.0 && !f.mapped(bad).dir_high);
  }
  for (const auto status : {aa::PiStatus::kDisabled, aa::PiStatus::kZeroTarget,
                           aa::PiStatus::kInvalidInput, aa::PiStatus::kArithmetic}) {
    auto bad = p; bad.motor.status = status;
    CHECK(f.mapped(bad).status == aa::Md20aMapStatus::kInvalidIntent);
  }
  auto bad = p; bad.motor.brake_requested = true;
  CHECK(f.mapped(bad).duty_ratio == 0.0);
  auto old_generation = p; ++old_generation.generation;
  CHECK(f.mapped(old_generation).status == aa::Md20aMapStatus::kInhibited);
  auto other_state = p; other_state.state = aa::ReceiverState::kArmed;
  CHECK(f.mapped(other_state).status == aa::Md20aMapStatus::kInhibited);
  for (const double cap : {0.0, -0.1, 1.1, std::numeric_limits<double>::infinity()}) {
    const auto out = aa::md20a_forward_command(p, f.receiver, {cap, true}, true);
    CHECK(out.status == aa::Md20aMapStatus::kInvalidProfile && out.duty_ratio == 0.0 && out.dir_high);
  }
  CHECK(f.receiver.state() == aa::ReceiverState::kActive && f.cycle.integral() == .003);
  f.receiver.tick(230000);
  CHECK(f.mapped(p).status == aa::Md20aMapStatus::kInhibited);  // 이전 주기 출력 보관 금지
  return true;
}

bool stop_and_expiry(sm::Scratch& s) {
  {
    ForwardFixture f(s, true); CHECK(f.start());
    const auto moving = f.cycle.tick(225000, true);
    CHECK(moving.motor.effort > 0.0);
    CHECK(sm::parse(s, "{\"v\":1,\"kind\":\"STOP\"}") && f.receiver.receive(s.command, 230000).accepted);
    CHECK(f.cycle.tick(230000, true).state == aa::ReceiverState::kEstop);
    CHECK(f.cycle.integral() == 0.0 && f.mapped(moving).duty_ratio == 0.0);
    CHECK(!f.drive(235000));
    CHECK(f.cycle.tick(235000, true).state == aa::ReceiverState::kEstop);
    CHECK(f.control("CLEAR", 240000));
    CHECK(f.cycle.tick(240000, true).state == aa::ReceiverState::kDisarmed);
    CHECK(!f.drive(245000));
    CHECK(f.control("ARM", 245000));
    CHECK(f.cycle.tick(245000, true).state == aa::ReceiverState::kArmed);
    CHECK(f.drive(250000));
    CHECK(f.cycle.tick(250000, true).motor.effort == .303);
  }
  {
    ForwardFixture f(s); CHECK(f.start()); f.cycle.tick(225000, true);
    f.receiver.set_estop(true, 230000); f.cycle.tick(230000, true);
    f.receiver.set_estop(false, 235000);
    CHECK(f.cycle.tick(235000, true).state == aa::ReceiverState::kEstop);
    CHECK(!f.drive(240000) && f.cycle.integral() == 0.0);
  }
  {
    ForwardFixture f(s); CHECK(f.start()); f.cycle.tick(225000, true);
    aa::MotorCyclePreview p;
    for (std::int64_t now = 230000; now <= 325000; now += 5000) {
      CHECK(f.feedback(now)); p = f.cycle.tick(now, true);
    }
    CHECK(sm::cycle_fault(p, "command_expired") && f.cycle.integral() == 0.0);
    CHECK(f.mapped(p).duty_ratio == 0.0 && !f.drive(330000));
  }
  {
    ForwardFixture f(s); CHECK(f.start()); f.cycle.tick(225000, true);
    CHECK(sm::cycle_fault(f.cycle.tick(230000, false), "output_unavailable"));
    CHECK(sm::cycle_fault(f.cycle.tick(235000, true), "output_unavailable"));
    CHECK(f.cycle.integral() == 0.0);
  }
  return true;
}

bool endpoint_limit_forwarding() {
  aa::TxConfig tx; tx.offer_period_us = 20000; tx.max_consecutive_events = 2;
  const auto cfg = sm::motor_config();
  aa::ActuationMotorEndpoint legacy(cfg, sm::kCycleBoot, 10, 10000,
      aa::PayloadLayout::kBareCommand, tx, .6, 1.2, .05);
  CHECK(legacy.control_tick(0).state == aa::ReceiverState::kDisarmed);
  aa::ActuationMotorEndpoint valid(cfg, sm::kCycleBoot, 10, 10000,
      aa::PayloadLayout::kBareCommand, tx, .6, 1.2, .05, 0.0, .4);
  CHECK(valid.control_tick(0).state == aa::ReceiverState::kDisarmed);
  aa::ActuationMotorEndpoint bad_lower(cfg, sm::kCycleBoot, 10, 10000,
      aa::PayloadLayout::kBareCommand, tx, .6, 1.2, .05, .1, .4);
  CHECK(sm::cycle_fault(bad_lower.control_tick(0), "control_invalid_config"));
  aa::ActuationMotorEndpoint bad_upper(cfg, sm::kCycleBoot, 10, 10000,
      aa::PayloadLayout::kBareCommand, tx, .6, 1.2, .05, 0.0, 0.0);
  CHECK(sm::cycle_fault(bad_upper.control_tick(0), "control_invalid_config"));
  return true;
}

int main() {
  static sm::Scratch scratch;
  for (const auto* name : sm::kMotorCycleCases) {
    if (!sm::motor_cycle_case(name, scratch)) { std::fprintf(stderr, "existing case: %s\n", name); return 1; }
  }
  if (!forward_and_overspeed(scratch, false) || !forward_and_overspeed(scratch, true) ||
      !mapping_contract(scratch) || !stop_and_expiry(scratch) || !endpoint_limit_forwarding()) return 1;
  std::puts("existing 25 cycle cases + forward duty/mapping/STOP/expiry/endpoint: PASS");
  return 0;
}
