#pragma once
// 보드 독립 수신 상태기계(PC 검증 후보). actuation_contract.ActuationReceiver의 판정 순서를 옮긴다.
// 단일 제어 소유자 계약: 모든 메서드는 한 작업(또는 잠금 아래)에서만 호출한다. 통신 callback에서 직접 호출하지 않는다.
// 출력 목표(target_speed/steering)는 구동 허가 상태의 표현일 뿐 전기 출력·차체 정지를 보장하지 않는다.
// Python과 다른 점(고정 크기에서 wrap/UB를 막기 위한 명시적 선택):
//  - now_us는 0..kMaxNowUs만 유효하다. 그 밖은 invalid_clock 고장(Python은 2^63-1까지 허용).
//  - generation/lease 순번이 int64 끝에 닿으면 재사용하지 않고 counter_exhausted 고장으로 고정한다.
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "actuation_command.hpp"

namespace arena_actuation {

enum class ReceiverState : std::uint8_t { kDisarmed = 0, kArmed, kActive, kEstop, kFault };
enum class FeedbackMode : std::uint8_t { kMotorShaft = 0, kLegacyRearPair = 1 };

constexpr std::int64_t kMaxConfigUs = 10'000'000;
constexpr std::int64_t kMaxNowUs = INT64_MAX - 2 * kMaxConfigUs;
constexpr std::size_t kMaxLeases = 32;

struct ReceiverConfig {
  std::int64_t lease_us = 100'000;
  std::int64_t feedback_timeout_us = 50'000;
  std::int64_t settle_us = 200'000;
  std::int64_t max_control_gap_us = 50'000;
  double wheel_radius_m = .025;
  std::int64_t ticks_per_rev = 2048;
  double max_speed_mps = 2.5;
  double max_steering_rad = .37;
  double stopped_wheel_mps = .03;
  double max_wheel_mps = 5.;
  FeedbackMode feedback_mode = FeedbackMode::kLegacyRearPair;
  double gear_ratio = 1.;
  std::int64_t motor_no_progress_us = 500'000;
  double motor_progress_min_command_mps = .1;

  // ContractConfig.__post_init__과 같은 검사. false면 수신기를 만들지 않는다.
  bool valid() const {
    const auto in = [](std::int64_t v) { return v >= 1 && v <= kMaxConfigUs; };
    const auto pos = [](double v) { return std::isfinite(v) && v > 0; };
    if (!in(lease_us) || !in(feedback_timeout_us) || !in(settle_us) || !in(max_control_gap_us) ||
        !in(ticks_per_rev) || !in(motor_no_progress_us)) return false;
    if (!pos(gear_ratio) || (feedback_mode == FeedbackMode::kLegacyRearPair && gear_ratio != 1.)) return false;
    if (!pos(wheel_radius_m) || !pos(max_speed_mps) || !pos(max_steering_rad) || !pos(stopped_wheel_mps) ||
        !pos(max_wheel_mps) || !pos(motor_progress_min_command_mps)) return false;
    if (stopped_wheel_mps >= max_speed_mps || max_speed_mps > max_wheel_mps) return false;
    if (motor_progress_min_command_mps > max_speed_mps) return false;
    return limits_supported(DriveLimits{max_speed_mps, max_steering_rad, lease_us});
  }
};

// 외부 입력 정수. Python integer() 판정을 위해 int64 밖/정수 아님을 valid=false로 표현한다.
struct IntInput {
  bool valid = false;
  std::int64_t value = 0;
  static IntInput of(std::int64_t v) { return IntInput{true, v}; }
};

enum class ReceiveReason : std::uint8_t {
  kInvalidMessage, kSoftwareStop, kSchema, kSession, kExpiredLease, kNotReady, kControlNotReady, kNotLatched,
  kExplicitClear, kNotDisarmed, kExplicitArm, kNotOwnerOrArmed, kSequence, kExplicitDisarm, kInvalidDrive,
  kSourceExpired, kDrive, kZeroTarget, kGenerationExhausted,
};
struct ReceiveResult { bool accepted; ReceiveReason reason; };

struct Lease { std::int64_t id; std::int64_t issued_us; std::int64_t expires_us; };
struct OfferResult { bool ok; Lease lease; };

class ActuationReceiverCore {
 public:
  // boot_id는 64 코드포인트 이하. 수신기는 고정 크기이며 동적 할당이 없다.
  ActuationReceiverCore(const ReceiverConfig& config, const std::uint32_t* boot, std::size_t boot_length)
      : config_(config) {
    boot_.is_string = true;
    boot_.complete = boot_length <= kMaxStoredCodePoints;
    boot_.length = static_cast<std::uint16_t>(boot_.complete ? boot_length : 0);
    for (std::size_t i = 0; i < boot_.length; ++i) boot_.code_points[i] = boot[i];
    motor_ = config_.feedback_mode == FeedbackMode::kMotorShaft;
  }

  // ---- 공개 이벤트(Python 메서드와 같은 이름/순서) ----
  void tick(std::int64_t now) {
    if (check(now)) { last_control_ = now; has_last_control_ = true; }
  }
  bool update_motor_encoder(std::int64_t now, IntInput sequence, IntInput capture, IntInput counts) {
    if (!motor_) return mode_mismatch(now);
    const IntInput values[1] = {counts};
    return ingest(now, sequence, capture, values, 1);
  }
  bool update_legacy_encoder(std::int64_t now, IntInput sequence, IntInput capture, IntInput left, IntInput right) {
    if (motor_) return mode_mismatch(now);
    const IntInput values[2] = {left, right};
    return ingest(now, sequence, capture, values, 2);
  }
  void set_estop(bool pressed, std::int64_t now) {
    clock(now);
    estop_held_ = pressed;
    if (pressed) transition(ReceiverState::kEstop, "physical_stop");
    // 해제는 상태 전환이 아니다. CLEAR와 ARM이 따로 필요하다.
  }
  OfferResult offer(std::int64_t now) {
    if (!check(now)) return OfferResult{false, {}};
    if (lease_sequence_ == INT64_MAX) { exhaust(); return OfferResult{false, {}}; }
    ++lease_sequence_;
    const Lease lease{lease_sequence_, now, now + config_.lease_us};
    if (lease_count_ >= kMaxLeases) erase_lease(0);  // 가장 오래 삽입된 임대부터 퇴출
    leases_[lease_count_++] = lease;
    return OfferResult{true, lease};
  }

  ReceiveResult receive(const Command& c, std::int64_t now) {
    const bool clock_ok = check(now);
    const CommandClass cls = classify(c);
    if (!clock_ok || cls == CommandClass::kNotDelivered || cls == CommandClass::kEnvelopeIgnored ||
        cls == CommandClass::kInvalidMessage)
      return {false, ReceiveReason::kInvalidMessage};
    if (cls == CommandClass::kStop) {
      transition(ReceiverState::kEstop, "software_stop");
      return {true, generation_exhausted_ ? ReceiveReason::kGenerationExhausted : ReceiveReason::kSoftwareStop};
    }
    if (cls == CommandClass::kSchema) return {false, ReceiveReason::kSchema};
    const JsonValue& generation = c.field(CommandField::kGeneration);
    const JsonValue& lease_id = c.field(CommandField::kLease);
    const JsonValue& seq = c.field(CommandField::kSeq);
    if (!c.boot.equals(boot_.code_points.data(), boot_.length) || !boot_.complete || !session_shape_ok(c) ||
        generation.integer != generation_)
      return {false, ReceiveReason::kSession};
    const Lease* lease = find_lease(lease_id.integer);
    if (lease == nullptr) return {false, ReceiveReason::kExpiredLease};
    if (cls == CommandClass::kClear || cls == CommandClass::kArm) {
      if (!stationary(now) || estop_held_) return {false, ReceiveReason::kNotReady};
      if (!has_last_control_ || now - last_control_ > config_.max_control_gap_us)
        return {false, ReceiveReason::kControlNotReady};
      if (cls == CommandClass::kClear) {
        if (state_ != ReceiverState::kFault && state_ != ReceiverState::kEstop) return {false, ReceiveReason::kNotLatched};
        if (generation_exhausted_) return {false, ReceiveReason::kGenerationExhausted};
        transition(ReceiverState::kDisarmed, "explicit_clear");
        return {true, ReceiveReason::kExplicitClear};
      }
      if (state_ != ReceiverState::kDisarmed) return {false, ReceiveReason::kNotDisarmed};
      const std::int64_t lease_expires = lease->expires_us;  // transition이 임대를 지우기 전에 복사
      transition(ReceiverState::kArmed, "explicit_arm");
      if (generation_exhausted_) return {false, ReceiveReason::kGenerationExhausted};
      owner_ = c.client;
      has_owner_ = true;
      sequence_ = seq.integer;
      deadline_ = lease_expires;
      has_deadline_ = true;
      return {true, ReceiveReason::kExplicitArm};
    }
    if (!armed() || !has_owner_ || !c.client.equals(owner_.code_points.data(), owner_.length))
      return {false, ReceiveReason::kNotOwnerOrArmed};
    if (seq.integer <= sequence_) return {false, ReceiveReason::kSequence};
    if (cls == CommandClass::kDisarm) {
      transition(ReceiverState::kDisarmed, "explicit_disarm");
      return {!generation_exhausted_, generation_exhausted_ ? ReceiveReason::kGenerationExhausted
                                                            : ReceiveReason::kExplicitDisarm};
    }
    const DriveLimits limits{config_.max_speed_mps, config_.max_steering_rad, config_.lease_us};
    if (!drive_values_ok(c, limits)) {
      fault("invalid_drive");
      return {false, ReceiveReason::kInvalidDrive};
    }
    const std::int64_t age = c.field(CommandField::kSourceAge).integer;
    const std::int64_t ttl = c.field(CommandField::kTtl).integer;
    // 0 <= age < ttl <= lease_us 이고 issued <= kMaxNowUs이므로 overflow가 없다.
    const std::int64_t source_deadline = lease->issued_us + ttl - age;
    const std::int64_t deadline = lease->expires_us < source_deadline ? lease->expires_us : source_deadline;
    if (now >= deadline) return {false, ReceiveReason::kSourceExpired};
    sequence_ = seq.integer;
    const double speed = numeric_value(c.field(CommandField::kSpeed));
    if (motor_) {
      if (speed < config_.motor_progress_min_command_mps) has_progress_since_ = false;
      else if (!has_progress_since_) { progress_since_ = now; has_progress_since_ = true; }
    }
    target_speed_ = speed;
    steering_ = numeric_value(c.field(CommandField::kSteering));
    deadline_ = deadline;
    has_deadline_ = true;
    state_ = speed > 0 ? ReceiverState::kActive : ReceiverState::kArmed;
    reason_ = speed > 0 ? "drive" : "zero_target";
    return {true, speed > 0 ? ReceiveReason::kDrive : ReceiveReason::kZeroTarget};
  }

  // ---- 상태 조회(Python status()와 같은 필드) ----
  bool armed() const { return state_ == ReceiverState::kArmed || state_ == ReceiverState::kActive; }
  bool fresh_feedback(std::int64_t now) const {
    return feedback_valid_ && has_feedback_ && now - feedback_capture_ >= 0 &&
           now - feedback_capture_ < config_.feedback_timeout_us;
  }
  ReceiverState state() const { return state_; }
  const char* reason() const { return reason_; }
  std::int64_t generation() const { return generation_; }
  std::int64_t accepted_seq() const { return sequence_; }
  double target_speed() const { return target_speed_; }
  double steering() const { return steering_; }
  bool has_owner() const { return has_owner_; }
  const StoredString& owner() const { return owner_; }
  const StoredString& boot() const { return boot_; }
  bool has_deadline() const { return has_deadline_; }
  std::int64_t deadline() const { return deadline_; }
  bool estop_held() const { return estop_held_; }
  bool motor_mode() const { return motor_; }
  double left_mps() const { return left_; }
  double right_mps() const { return right_; }
  double mean_mps() const { return mean_; }
  bool has_feedback() const { return has_feedback_; }
  std::int64_t feedback_seq() const { return feedback_seq_; }
  std::int64_t feedback_capture() const { return feedback_capture_; }
  std::int64_t feedback_count(std::size_t i) const { return feedback_counts_[i]; }
  bool has_now() const { return has_now_; }
  std::int64_t now() const { return now_; }
  std::uint64_t deadline_miss_count() const { return deadline_miss_count_; }
  std::size_t lease_count() const { return lease_count_; }
  const Lease& lease_at(std::size_t i) const { return leases_[i]; }

  // PC 경계 시험 전용: 소진 경계를 짧게 재현한다(운영 코드에서 호출하지 않는다).
  void test_set_counters(std::int64_t generation, std::int64_t lease_sequence) {
    generation_ = generation;
    lease_sequence_ = lease_sequence;
  }

 private:
  void transition(ReceiverState state, const char* reason) {
    target_speed_ = 0.;
    // 정지 중 갑작스러운 중앙 조향은 하지 않는다(steering 유지).
    has_deadline_ = false;
    has_owner_ = false;
    owner_ = StoredString{};
    sequence_ = -1;
    lease_count_ = 0;
    has_progress_since_ = false;
    if (generation_exhausted_ || generation_ == INT64_MAX) { exhaust(); return; }
    state_ = state;
    reason_ = reason;
    ++generation_;
  }
  // 세대/임대 순번을 재사용하지 않는다. 재부팅 전까지 목표 0 고장으로 고정한다.
  void exhaust() {
    generation_exhausted_ = true;
    state_ = ReceiverState::kFault;
    reason_ = "counter_exhausted";
    target_speed_ = 0.;
    has_deadline_ = false;
    has_owner_ = false;
    owner_ = StoredString{};
    sequence_ = -1;
    lease_count_ = 0;
    has_progress_since_ = false;
  }
  void fault(const char* reason) {
    if (state_ != ReceiverState::kEstop && state_ != ReceiverState::kFault) transition(ReceiverState::kFault, reason);
  }
  bool clock(std::int64_t now) {
    if (now < 0 || now > kMaxNowUs) { fault("invalid_clock"); return false; }
    if (has_now_ && now < now_) {
      fault("clock_regressed");
      feedback_valid_ = false;
      has_zero_since_ = false;
      return false;
    }
    now_ = now;
    has_now_ = true;
    std::size_t kept = 0;
    for (std::size_t i = 0; i < lease_count_; ++i)
      if (now < leases_[i].expires_us) leases_[kept++] = leases_[i];
    lease_count_ = kept;
    return true;
  }
  bool stationary(std::int64_t now) const {
    return fresh_feedback(now) && has_zero_since_ && now - zero_since_ >= config_.settle_us;
  }
  bool check(std::int64_t now) {
    if (!clock(now)) return false;
    if (armed() && (!has_last_control_ || now - last_control_ > config_.max_control_gap_us)) {
      ++deadline_miss_count_;
      fault("control_deadline_missed");
    }
    if (armed()) {
      if (!fresh_feedback(now)) fault("encoder_timeout");
      else if (!has_deadline_ || now >= deadline_) fault("command_expired");
      else if (motor_ && has_progress_since_ && now - progress_since_ >= config_.motor_no_progress_us)
        fault("motor_feedback_no_progress");
    }
    return true;
  }
  bool mode_mismatch(std::int64_t now) {
    if (!clock(now)) return false;
    feedback_valid_ = false;
    has_zero_since_ = false;
    if (armed()) fault("feedback_mode_mismatch");
    return false;
  }
  // 차분은 부호 있는 65비트 크기를 uint64 크기+부호로 정확히 만든 뒤 double로 한 번 반올림한다(Python float(int)).
  static double delta(std::int64_t now_count, std::int64_t old) {
    const std::uint64_t a = static_cast<std::uint64_t>(now_count), b = static_cast<std::uint64_t>(old);
    if (now_count >= old) return static_cast<double>(a - b);
    return -static_cast<double>(b - a);
  }
  bool ingest(std::int64_t now, IntInput sequence, IntInput capture, const IntInput* counts, std::size_t n) {
    if (!clock(now)) return false;
    bool valid = sequence.valid && sequence.value >= 0 && capture.valid && capture.value >= 0 &&
                 capture.value <= now && now - capture.value < config_.feedback_timeout_us;
    for (std::size_t i = 0; i < n; ++i) valid = valid && counts[i].valid;
    if (!valid) {
      feedback_valid_ = false;
      has_zero_since_ = false;
      if (armed()) fault("invalid_encoder");
      return false;
    }
    if (!has_feedback_) { store_sample(sequence.value, capture.value, counts, n); return true; }
    if (sequence.value <= feedback_seq_ || capture.value <= feedback_capture_) return false;
    const std::int64_t dt = capture.value - feedback_capture_;
    if (dt >= config_.feedback_timeout_us) {
      store_sample(sequence.value, capture.value, counts, n);
      feedback_valid_ = false;
      has_zero_since_ = false;
      if (armed()) fault("encoder_gap");
      return false;
    }
    // Python과 같은 연산 순서: tau * r / (ticks * gear) * 1e6 / dt
    const double scale = 6.283185307179586 * config_.wheel_radius_m /
                         (static_cast<double>(config_.ticks_per_rev) * config_.gear_ratio) * 1e6 /
                         static_cast<double>(dt);
    double speeds[2] = {0., 0.};
    double peak = 0.;
    for (std::size_t i = 0; i < n; ++i) {
      speeds[i] = delta(counts[i].value, feedback_counts_[i]) * scale;
      if (std::fabs(speeds[i]) > peak || std::isnan(speeds[i])) peak = std::fabs(speeds[i]);
    }
    if (peak > config_.max_wheel_mps) {
      feedback_valid_ = false;
      has_zero_since_ = false;
      if (armed()) fault("encoder_implausible");
      return false;
    }
    const std::int64_t previous_count = feedback_counts_[0];
    store_sample(sequence.value, capture.value, counts, n);
    feedback_valid_ = true;
    if (motor_) {
      mean_ = speeds[0];
      if (counts[0].value != previous_count && has_progress_since_ && capture.value > progress_since_)
        progress_since_ = capture.value;
    } else {
      left_ = speeds[0];
      right_ = speeds[1];
      mean_ = (speeds[0] + speeds[1]) / 2;
    }
    // 모터축 모드의 정지 준비는 평균 0뿐이다. 좌우 반대 회전은 여기서 검출되지 않는다.
    if (peak <= config_.stopped_wheel_mps) {
      if (!has_zero_since_) { zero_since_ = capture.value; has_zero_since_ = true; }
    } else {
      has_zero_since_ = false;
    }
    return true;
  }
  void store_sample(std::int64_t seq, std::int64_t capture, const IntInput* counts, std::size_t n) {
    has_feedback_ = true;
    feedback_seq_ = seq;
    feedback_capture_ = capture;
    for (std::size_t i = 0; i < n; ++i) feedback_counts_[i] = counts[i].value;
  }
  const Lease* find_lease(std::int64_t id) const {
    for (std::size_t i = 0; i < lease_count_; ++i) if (leases_[i].id == id) return &leases_[i];
    return nullptr;
  }
  void erase_lease(std::size_t index) {
    for (std::size_t i = index + 1; i < lease_count_; ++i) leases_[i-1] = leases_[i];
    --lease_count_;
  }

  ReceiverConfig config_;
  StoredString boot_;
  bool motor_ = false;
  ReceiverState state_ = ReceiverState::kDisarmed;
  const char* reason_ = "boot";
  std::int64_t generation_ = 0;
  bool generation_exhausted_ = false;
  StoredString owner_;
  bool has_owner_ = false;
  std::int64_t sequence_ = -1;
  double target_speed_ = 0., steering_ = 0.;
  std::int64_t deadline_ = 0;
  bool has_deadline_ = false;
  std::int64_t now_ = 0;
  bool has_now_ = false;
  std::int64_t last_control_ = 0;
  bool has_last_control_ = false;
  bool has_feedback_ = false;
  bool feedback_valid_ = false;
  std::int64_t feedback_seq_ = 0, feedback_capture_ = 0;
  std::array<std::int64_t, 2> feedback_counts_{};
  double left_ = 0., right_ = 0., mean_ = 0.;
  std::int64_t zero_since_ = 0;
  bool has_zero_since_ = false;
  bool estop_held_ = false;
  std::array<Lease, kMaxLeases> leases_{};
  std::size_t lease_count_ = 0;
  std::int64_t lease_sequence_ = 0;
  std::int64_t progress_since_ = 0;
  bool has_progress_since_ = false;
  std::uint64_t deadline_miss_count_ = 0;
};

inline const char* receive_reason_name(ReceiveReason r) {
  constexpr const char* names[] = {
    "invalid_message", "software_stop", "schema", "session", "expired_lease", "not_ready", "control_not_ready",
    "not_latched", "explicit_clear", "not_disarmed", "explicit_arm", "not_owner_or_armed", "sequence",
    "explicit_disarm", "invalid_drive", "source_expired", "drive", "zero_target", "counter_exhausted",
  };
  return names[static_cast<std::size_t>(r)];
}

}  // namespace arena_actuation
