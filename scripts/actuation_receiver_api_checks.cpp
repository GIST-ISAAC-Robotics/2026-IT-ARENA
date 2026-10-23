// PC 전용 코어 API 경계 회귀. 인자 하나당 독립 프로세스로 실행해 sanitizer 실패도 보존한다.
#define ARENA_ACTUATION_TESTING 1
#include "actuation_receiver.hpp"
#include <cstring>
#include <iostream>
#include <string>

namespace aa = arena_actuation;
constexpr std::uint32_t kBoot[] = {'b'};

aa::ReceiverConfig motor_config() {
  aa::ReceiverConfig c;
  c.feedback_mode = aa::FeedbackMode::kMotorShaft;
  c.wheel_radius_m = .0325; c.ticks_per_rev = 28; c.gear_ratio = 15.;
  return c;
}

void ready(aa::ActuationReceiverCore& r) {
  for (std::int64_t now = 0; now <= 220000; now += 10000) {
    r.update_motor_encoder(now, aa::IntInput::of(now / 10000), aa::IntInput::of(now), aa::IntInput::of(0));
    r.tick(now);
  }
}

aa::Command make_command(aa::KindName kind, const aa::ActuationReceiverCore& r, const aa::Lease& lease) {
  aa::Command c;
  c.status = aa::ParseStatus::kObject; c.kind = kind;
  auto set_int = [&](aa::CommandField f, std::int64_t value) {
    c.present |= aa::field_bit(f);
    c.fields[static_cast<unsigned>(f)].type = aa::ValueType::kInt;
    c.fields[static_cast<unsigned>(f)].integer = value;
  };
  set_int(aa::CommandField::kV, 1);
  set_int(aa::CommandField::kKind, 0);
  c.fields[static_cast<unsigned>(aa::CommandField::kKind)].type = aa::ValueType::kString;
  if (kind == aa::KindName::kStop) return c;
  set_int(aa::CommandField::kBoot, 0); set_int(aa::CommandField::kClient, 0);
  c.fields[static_cast<unsigned>(aa::CommandField::kBoot)].type = aa::ValueType::kString;
  c.fields[static_cast<unsigned>(aa::CommandField::kClient)].type = aa::ValueType::kString;
  c.boot = r.boot(); c.client = r.boot();
  set_int(aa::CommandField::kGeneration, r.generation());
  set_int(aa::CommandField::kLease, lease.id);
  set_int(aa::CommandField::kSeq, 1);
  if (kind == aa::KindName::kDrive) {
    set_int(aa::CommandField::kSeq, 2);
    set_int(aa::CommandField::kSpeed, 1); set_int(aa::CommandField::kSteering, 0);
    set_int(aa::CommandField::kSourceAge, 0); set_int(aa::CommandField::kTtl, 100000);
  }
  return c;
}

bool check(const std::string& name) {
  auto cfg = motor_config();
  if (name == "invalid_feedback_mode") {
    cfg.feedback_mode = static_cast<aa::FeedbackMode>(255);
    return !cfg.valid();
  }
  if (name == "invalid_config_constructor") {
    cfg.lease_us = 0;
    aa::ActuationReceiverCore r(cfg, kBoot, 1);
    r.set_estop(true, 0); r.set_estop(false, 0); ready(r);
    return r.state() == aa::ReceiverState::kFault && !r.offer(0).ok && r.target_speed() == 0.;
  }
  if (name == "invalid_boot_codepoint") {
    const std::uint32_t boot[] = {0x110000};
    aa::ActuationReceiverCore r(cfg, boot, 1);
    return r.state() == aa::ReceiverState::kFault && !r.offer(0).ok;
  }
  if (name == "null_boot_constructor") {
    aa::ActuationReceiverCore r(cfg, nullptr, 1);
    return r.state() == aa::ReceiverState::kFault && !r.offer(0).ok;
  }
  if (name == "empty_boot_constructor" || name == "long_boot_constructor") {
    std::uint32_t boot[65]{};
    aa::ActuationReceiverCore r(cfg, boot, name == "empty_boot_constructor" ? 0 : 65);
    return r.state() == aa::ReceiverState::kFault && !r.offer(0).ok;
  }
  aa::ActuationReceiverCore r(cfg, kBoot, 1);
  ready(r);
  if (name == "freshness_invalid_time") {
    return !r.fresh_feedback(INT64_MIN) && !r.fresh_feedback(-1) && r.fresh_feedback(220000);
  }
  if (name == "clear_at_counter_exhaustion") {
    r.set_estop(true, 220000); r.set_estop(false, 220000);
    r.test_set_counters(INT64_MAX, 0);
    const auto offer = r.offer(220000);
    const auto result = r.receive(make_command(aa::KindName::kClear, r, offer.lease), 220000);
    return !result.accepted && result.reason == aa::ReceiveReason::kGenerationExhausted &&
           r.state() == aa::ReceiverState::kFault && r.target_speed() == 0. && !r.offer(220000).ok;
  }
  if (name == "valid_motor_arm_drive_stop") {
    auto offer = r.offer(220000);
    if (!r.receive(make_command(aa::KindName::kArm, r, offer.lease), 220000).accepted) return false;
    offer = r.offer(220000);
    if (!r.receive(make_command(aa::KindName::kDrive, r, offer.lease), 220000).accepted || r.target_speed() != 1.) return false;
    return r.receive(make_command(aa::KindName::kStop, r, {}), 220000).accepted &&
           r.state() == aa::ReceiverState::kEstop && r.target_speed() == 0.;
  }
  return false;
}

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  const bool passed = check(argv[1]);
  std::cout << argv[1] << ':' << (passed ? "PASS" : "FAIL") << '\n';
  return passed ? 0 : 1;
}
