#pragma once
// 단일 모터축 평균 바퀴 속도 PI. PWM/DIR/전기 제동/실차 게인 매핑이 아니다.
#include <cmath>
#include <limits>

#if defined(__FAST_MATH__) || (defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__ > 0)
#error "SpeedPI requires nonfinite checks; fast-math/finite-math-only is unsupported"
#endif

namespace arena_actuation {

static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559 &&
              std::numeric_limits<double>::digits == 53, "SpeedPI requires IEEE binary64");

enum class PiStatus { kNormal, kDisabled, kZeroTarget, kInvalidConfig, kInvalidInput, kArithmetic };

struct MotorIntent {
  double effort = 0.0;
  bool brake_requested = true;
  PiStatus status = PiStatus::kDisabled;
};

// 설정은 객체 수명 동안 불변. 기본값은 Python 가상 모터 참조값이며 실차 튜닝값이 아니다.
class SpeedPI {
 public:
  explicit SpeedPI(double kp = 0.6, double ki = 1.2, double max_dt_s = 0.05) noexcept
      : kp_(kp), ki_(ki), max_dt_s_(max_dt_s),
        valid_(positive_finite(kp) && positive_finite(ki) && positive_finite(max_dt_s)) {}

  bool valid() const noexcept { return valid_; }
  double integral() const noexcept { return integral_; }
  void reset() noexcept { integral_ = 0.0; }

  // enabled는 호출자가 검증한 허가이며 이 계산기 자체가 ARM/STOP/watchdog을 대신하지 않는다.
  MotorIntent update_mean(double target_mps, double mean_wheel_mps, double dt_s, bool enabled) noexcept {
    if (!valid_) return inhibit(PiStatus::kInvalidConfig);
    if (!std::isfinite(target_mps) || !std::isfinite(mean_wheel_mps) || !std::isfinite(dt_s) ||
        target_mps < 0.0 || !(dt_s > 0.0 && dt_s <= max_dt_s_)) return inhibit(PiStatus::kInvalidInput);
    if (!enabled) return inhibit(PiStatus::kDisabled);
    if (target_mps == 0.0) return inhibit(PiStatus::kZeroTarget);
    const double error = target_mps - mean_wheel_mps;
    const double candidate = integral_ + ki_ * error * dt_s;
    const double raw = kp_ * error + candidate;
    if (!std::isfinite(error) || !std::isfinite(candidate) || !std::isfinite(raw))
      return inhibit(PiStatus::kArithmetic);
    if ((raw >= -1.0 && raw <= 1.0) || (raw > 1.0 && error < 0.0) || (raw < -1.0 && error > 0.0))
      integral_ = clamp(candidate);
    const double effort_raw = kp_ * error + integral_;
    if (!std::isfinite(effort_raw)) return inhibit(PiStatus::kArithmetic);
    const double effort = clamp(effort_raw);
    return MotorIntent{effort, effort < 0.0, PiStatus::kNormal};
  }

 private:
  static bool positive_finite(double v) noexcept { return std::isfinite(v) && v > 0.0; }
  static double clamp(double v) noexcept { return v < -1.0 ? -1.0 : v > 1.0 ? 1.0 : v; }
  MotorIntent inhibit(PiStatus status) noexcept { reset(); return MotorIntent{0.0, true, status}; }
  const double kp_, ki_, max_dt_s_;
  const bool valid_;
  double integral_ = 0.0;
};

}  // namespace arena_actuation
