#pragma once
// 출력 없는 단일 모터 제어 주기 결합의 합성 사례. 시간·엔코더·준비값은 시험 입력이다.
#include "smoke_cases.hpp"
#include "motor_control_candidate.hpp"

namespace arena_smoke {
inline constexpr const char* kMotorCycleCases[] = {
  "normal", "zero_and_resume", "software_stop_rearm", "physical_stop",
  "output_loss_latches", "default_ready_false", "command_expiry", "feedback_timeout",
  "control_gap", "duplicate_cycle", "regressed_clock", "pi_arithmetic",
  "invalid_config", "arm_drive_batch", "attach_while_active", "generation_reset",
  "legacy_rejected", "poll_does_not_tick", "existing_fault_preserved", "invalid_clock",
  "motor_feedback_used", "normal_deceleration", "external_tick_cannot_mask_gap",
  "new_arm_batch_resets", "clear_without_fresh_feedback"
};
inline constexpr unsigned kMotorCycleCount = sizeof(kMotorCycleCases) / sizeof(kMotorCycleCases[0]);
inline constexpr std::uint32_t kCycleBoot[] = {'s','m','o','k','e','-','o','n','l','y'};

struct CycleFixture {
  aa::ActuationReceiverCore receiver;
  aa::MotorControlCandidate cycle;
  Scratch& scratch;
  explicit CycleFixture(Scratch& s, double kp = .6, double ki = 1.2,
                        aa::ReceiverConfig config = motor_config())
      : receiver(config, kCycleBoot, 10), cycle(receiver, kp, ki), scratch(s) {}
  bool feedback(std::int64_t now, std::int64_t count = 0) {
    return receiver.update_motor_encoder(now, aa::IntInput::of(now / 5000),
                                         aa::IntInput::of(now), aa::IntInput::of(count));
  }
  void prepare() {
    for (std::int64_t now = 0; now <= 215000; now += 5000) {
      feedback(now);
      cycle.tick(now);  // 비허가 상태이며 출력 준비는 기본 false
    }
  }
  bool control(const char* kind, std::int64_t now, unsigned seq = 1) {
    const auto offer = receiver.offer(now);
    return offer.ok && command(scratch, kind, receiver, offer.lease, seq) &&
           receiver.receive(scratch.command, now).accepted;
  }
  bool drive(std::int64_t now, double speed = .5, unsigned seq = 2) {
    const auto offer = receiver.offer(now);
    if (!offer.ok) return false;
    const int n = std::snprintf(scratch.text, sizeof(scratch.text),
        "{\"v\":1,\"kind\":\"DRIVE\",\"boot\":\"smoke-only\",\"generation\":%lld,\"lease\":%lld,"
        "\"client\":\"smoke-only\",\"seq\":%u,\"speed_mps\":%.17g,\"steering_rad\":0.2,"
        "\"source_age_us\":0,\"ttl_us\":100000}", static_cast<long long>(receiver.generation()),
        static_cast<long long>(offer.lease.id), seq, speed);
    return n > 0 && static_cast<std::size_t>(n) < sizeof(scratch.text) && parse(scratch, scratch.text) &&
           receiver.receive(scratch.command, now).accepted;
  }
  bool arm() {
    if (!control("ARM", 220000)) return false;
    const auto zero = cycle.tick(220000, true);
    return zero.state == aa::ReceiverState::kArmed && zero.motor.effort == 0.0;
  }
  bool start() { prepare(); return arm() && drive(225000); }
  bool stop(std::int64_t now) {
    return parse(scratch, "{\"v\":1,\"kind\":\"STOP\"}") && receiver.receive(scratch.command, now).accepted;
  }
};

inline bool cycle_fault(const aa::MotorCyclePreview& p, const char* reason) {
  return p.state == aa::ReceiverState::kFault && p.motor.effort == 0.0 && p.motor.brake_requested &&
         std::strcmp(p.reason, reason) == 0;
}
inline bool cycle_normal(const aa::MotorCyclePreview& p) {
  return p.state == aa::ReceiverState::kActive && std::fabs(p.motor.effort - .303) < 1e-15 &&
         !p.motor.brake_requested && p.steering_rad == .2;
}

inline bool motor_cycle_case(const char* name, Scratch& scratch) {
  if (std::strcmp(name, "legacy_rejected") == 0) {
    CycleFixture f(scratch, .6, 1.2, aa::ReceiverConfig{});
    return cycle_fault(f.cycle.tick(0), "control_invalid_config");
  }
  if (std::strcmp(name, "invalid_config") == 0) {
    CycleFixture f(scratch, 0.0);
    return cycle_fault(f.cycle.tick(0), "control_invalid_config");
  }
  if (std::strcmp(name, "pi_arithmetic") == 0) {
    CycleFixture f(scratch, 1e308);
    f.prepare();
    if (!f.arm() || !f.drive(225000, 2.5)) return false;
    const auto bad = f.cycle.tick(225000, true);
    return cycle_fault(bad, "control_pi_arithmetic") && f.cycle.integral() == 0.0 &&
           cycle_fault(f.cycle.tick(230000, true), "control_pi_arithmetic");
  }
  if (std::strcmp(name, "arm_drive_batch") == 0) {
    CycleFixture f(scratch);
    f.prepare();
    if (!f.control("ARM", 220000) || !f.drive(225000)) return false;
    // 직전 PI 주기는 215 ms다. 220 ARM·225 DRIVE 모두 receiver가 검증했고 dt는 10 ms.
    const auto out = f.cycle.tick(225000, true);
    return out.state == aa::ReceiverState::kActive && std::fabs(out.motor.effort - .306) < 1e-15 &&
           std::fabs(f.cycle.integral() - .006) < 1e-15;
  }
  CycleFixture f(scratch);
  if (!f.start()) return false;
  const auto first = f.cycle.tick(225000, true);
  if (!cycle_normal(first)) return false;
  if (std::strcmp(name, "normal") == 0) return f.cycle.integral() == .003 && f.receiver.motor_mode();
  if (std::strcmp(name, "motor_feedback_used") == 0 || std::strcmp(name, "normal_deceleration") == 0) {
    const std::int64_t count = std::strcmp(name, "motor_feedback_used") == 0 ? 1 : 20;
    if (!f.feedback(230000, count)) return false;
    const auto out = f.cycle.tick(230000, true);
    const double speed = static_cast<double>(count) * (2.0 * std::acos(-1.0) * .0325) / (28.0 * 15.0 * .015);
    const double error = .5 - speed;
    const double expected = .6 * error + (.003 + 1.2 * error * .005);
    return out.state == aa::ReceiverState::kActive && std::fabs(f.receiver.mean_mps() - speed) < 1e-14 &&
           std::fabs(out.motor.effort - expected) < 1e-14 && out.motor.brake_requested == (count == 20);
  }
  if (std::strcmp(name, "external_tick_cannot_mask_gap") == 0) {
    for (std::int64_t now = 230000; now <= 280000; now += 5000) { f.feedback(now); f.receiver.tick(now); }
    // 수신기 tick을 다른 곳에서 잘못 갱신해도 PI 주기 자체의 55 ms 공백을 숨기지 않는다.
    return cycle_fault(f.cycle.tick(280001, true), "control_invalid_period");
  }
  if (std::strcmp(name, "new_arm_batch_resets") == 0) {
    if (!f.stop(230000) || !f.control("CLEAR", 235000) || !f.control("ARM", 240000) || !f.drive(245000)) return false;
    const auto out = f.cycle.tick(245000, true);
    // 옛 적분 .003은 버리고 마지막 PI 주기 이후 20 ms로 새 입력을 계산한다.
    return out.state == aa::ReceiverState::kActive && std::fabs(out.motor.effort - .312) < 1e-15 &&
           std::fabs(f.cycle.integral() - .012) < 1e-15;
  }
  if (std::strcmp(name, "clear_without_fresh_feedback") == 0) {
    if (!cycle_fault(f.cycle.tick(230000, false), "output_unavailable")) return false;
    return !f.control("CLEAR", 280000) && cycle_fault(f.cycle.tick(280000, true), "output_unavailable");
  }
  if (std::strcmp(name, "zero_and_resume") == 0) {
    if (!f.drive(230000, 0.0, 3)) return false;
    const auto zero = f.cycle.tick(230000, true);
    if (zero.motor.effort != 0.0 || !zero.motor.brake_requested || f.cycle.integral() != 0.0) return false;
    return f.drive(235000, .5, 4) && cycle_normal(f.cycle.tick(235000, true));
  }
  if (std::strcmp(name, "software_stop_rearm") == 0 || std::strcmp(name, "generation_reset") == 0) {
    if (!f.stop(230000)) return false;
    if (std::strcmp(name, "software_stop_rearm") == 0) {
      const auto stopped = f.cycle.tick(230000, true);
      if (stopped.state != aa::ReceiverState::kEstop || stopped.motor.effort != 0.0 ||
          stopped.steering_rad != .2 || f.cycle.integral() != 0.0) return false;
    }
    // generation_reset은 STOP/CLEAR를 제어 주기 사이에 처리했어도 옛 적분이 사라지는지 본다.
    if (!f.control("CLEAR", 235000) || !f.control("ARM", 240000)) return false;
    if (f.cycle.tick(240000, true).state != aa::ReceiverState::kArmed || f.cycle.integral() != 0.0) return false;
    return f.drive(245000) && cycle_normal(f.cycle.tick(245000, true));
  }
  if (std::strcmp(name, "physical_stop") == 0) {
    f.receiver.set_estop(true, 230000);
    const auto stopped = f.cycle.tick(230000, true);
    f.receiver.set_estop(false, 235000);
    const auto released = f.cycle.tick(235000, true);
    return stopped.state == aa::ReceiverState::kEstop && stopped.motor.effort == 0.0 &&
           released.state == aa::ReceiverState::kEstop && released.motor.effort == 0.0 &&
           std::strcmp(released.reason, "physical_stop") == 0;
  }
  if (std::strcmp(name, "output_loss_latches") == 0 || std::strcmp(name, "default_ready_false") == 0) {
    const auto lost = std::strcmp(name, "default_ready_false") == 0 ? f.cycle.tick(230000)
                                                                   : f.cycle.tick(230000, false);
    if (!cycle_fault(lost, "output_unavailable") || f.cycle.integral() != 0.0 ||
        !cycle_fault(f.cycle.tick(235000, true), "output_unavailable")) return false;
    if (!f.control("CLEAR", 240000)) return false;
    const auto clear = f.cycle.tick(240000, true);
    if (clear.state != aa::ReceiverState::kDisarmed || clear.motor.effort != 0.0) return false;
    if (!f.control("ARM", 245000)) return false;
    f.cycle.tick(245000, true);
    return f.drive(250000) && cycle_normal(f.cycle.tick(250000, true));
  }
  if (std::strcmp(name, "command_expiry") == 0) {
    aa::MotorCyclePreview last;
    for (std::int64_t now = 230000; now <= 325000; now += 5000) {
      f.feedback(now);
      last = f.cycle.tick(now, true);
      if (now < 325000 && last.state != aa::ReceiverState::kActive) return false;
    }
    return cycle_fault(last, "command_expired") && f.cycle.integral() == 0.0;
  }
  if (std::strcmp(name, "feedback_timeout") == 0) {
    aa::MotorCyclePreview last;
    for (std::int64_t now = 230000; now <= 265000; now += 5000) last = f.cycle.tick(now, true);
    return cycle_fault(last, "encoder_timeout") && f.cycle.integral() == 0.0;
  }
  if (std::strcmp(name, "control_gap") == 0) return cycle_fault(f.cycle.tick(285000, true), "control_deadline_missed");
  if (std::strcmp(name, "duplicate_cycle") == 0) return cycle_fault(f.cycle.tick(225000, true), "control_invalid_period");
  if (std::strcmp(name, "regressed_clock") == 0) {
    const auto out = f.cycle.tick(224999, true);
    return out.state == aa::ReceiverState::kFault && out.motor.effort == 0.0 &&
           std::strcmp(f.receiver.reason(), "clock_regressed") == 0 && f.cycle.integral() == 0.0;
  }
  if (std::strcmp(name, "attach_while_active") == 0) {
    aa::MotorControlCandidate newly_attached(f.receiver);
    return cycle_fault(newly_attached.tick(230000, true), "control_invalid_period");
  }
  if (std::strcmp(name, "poll_does_not_tick") == 0) {
    for (std::int64_t now = 230000; now <= 280000; now += 5000) { f.feedback(now); f.receiver.offer(now); }
    return cycle_fault(f.cycle.tick(280001, true), "control_deadline_missed");
  }
  if (std::strcmp(name, "existing_fault_preserved") == 0) {
    f.receiver.set_estop(true, 230000);
    const auto generation = f.receiver.generation();
    f.receiver.report_control_fault(aa::ControlFault::kOutputUnavailable, 230000);
    const auto out = f.cycle.tick(230000, false);
    return out.state == aa::ReceiverState::kEstop && out.motor.effort == 0.0 &&
           std::strcmp(out.reason, "physical_stop") == 0 && generation == f.receiver.generation();
  }
  if (std::strcmp(name, "invalid_clock") == 0) {
    const auto out = f.cycle.tick(-1, true);
    return out.state == aa::ReceiverState::kFault && out.motor.effort == 0.0 &&
           std::strcmp(f.receiver.reason(), "invalid_clock") == 0 && f.cycle.integral() == 0.0;
  }
  return false;
}

inline Report run_motor_cycle(Scratch& scratch) {
  Report report;
  for (const char* name : kMotorCycleCases) report.record(motor_cycle_case(name, scratch));
  return report;
}
}  // namespace arena_smoke
