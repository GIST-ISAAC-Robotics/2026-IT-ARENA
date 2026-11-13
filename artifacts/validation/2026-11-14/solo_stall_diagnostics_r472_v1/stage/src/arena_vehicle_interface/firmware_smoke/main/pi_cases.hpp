#pragma once
// 호스트/MCU 검사 앱 공유. 전기 출력 없는 합성 PI 계산만 수행한다.
#include "smoke_cases.hpp"
#include "speed_pi.hpp"
#include <limits>

namespace arena_smoke {
inline Report run_pi() {
  Report report;
  aa::SpeedPI pi;
  auto value = pi.update_mean(1.0, 0.5, 0.005, true);
  report.record(pi.valid() && std::fabs(value.effort - 0.303) < 1e-15 && !value.brake_requested &&
                std::fabs(pi.integral() - 0.003) < 1e-15);
  const double integral = pi.integral();
  for (unsigned i = 0; i < 100; ++i) value = pi.update_mean(5.0, 0.0, 0.005, true);
  report.record(value.effort == 1.0 && pi.integral() == integral);
  value = pi.update_mean(0.1, 4.0, 0.005, true);
  report.record(value.effort == -1.0 && value.brake_requested && pi.integral() == integral);
  value = pi.update_mean(0.0, 0.5, 0.005, true);
  report.record(value.effort == 0.0 && value.brake_requested && pi.integral() == 0.0 &&
                value.status == aa::PiStatus::kZeroTarget);
  pi.update_mean(1.0, 0.5, 0.005, true);
  value = pi.update_mean(1.0, 0.5, 0.005, false);
  report.record(value.effort == 0.0 && pi.integral() == 0.0 && value.status == aa::PiStatus::kDisabled);
  value = pi.update_mean(1.0, 0.5, 0.051, true);
  report.record(value.effort == 0.0 && value.status == aa::PiStatus::kInvalidInput);
  value = pi.update_mean(1.0, std::numeric_limits<double>::quiet_NaN(), 0.005, true);
  report.record(value.effort == 0.0 && value.status == aa::PiStatus::kInvalidInput);
  value = pi.update_mean(1e308, -1e308, 0.005, true);
  report.record(value.effort == 0.0 && value.brake_requested && value.status == aa::PiStatus::kArithmetic);
  aa::SpeedPI p_overflow(1e308, 1.2, 0.05);
  value = p_overflow.update_mean(2.0, 0.0, 0.005, true);
  report.record(value.effort == 0.0 && value.status == aa::PiStatus::kArithmetic);
  aa::SpeedPI i_overflow(0.6, 1e308, 0.05);
  value = i_overflow.update_mean(2.0, 0.0, 0.005, true);
  report.record(value.effort == 0.0 && value.status == aa::PiStatus::kArithmetic);
  aa::SpeedPI invalid(0.0, 1.2, 0.05);
  value = invalid.update_mean(1.0, 0.0, 0.005, true);
  report.record(!invalid.valid() && value.effort == 0.0 && value.status == aa::PiStatus::kInvalidConfig);
  value = pi.update_mean(1.0, 0.5, 0.005, true);
  report.record(value.status == aa::PiStatus::kNormal && std::fabs(value.effort - 0.303) < 1e-15);
  return report;
}
}  // namespace arena_smoke
