// 합성 수치 입력 전용. 첫 행 kp ki max_dt output_min output_max.
#include "speed_pi.hpp"
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

bool read_double(double& value) {
  std::string token;
  if (!(std::cin >> token)) return false;
  char* end = nullptr;
  value = std::strtod(token.c_str(), &end);
  return end == token.c_str() + token.size();
}
int main() {
  double kp, ki, max_dt, lower, upper;
  if (!read_double(kp) || !read_double(ki) || !read_double(max_dt) ||
      !read_double(lower) || !read_double(upper)) return 2;
  arena_actuation::SpeedPI pi(kp, ki, max_dt, lower, upper);
  std::cout << std::setprecision(17);
  while (true) {
    std::cin >> std::ws;
    if (std::cin.eof()) break;
    double target, measured, dt;
    int enabled;
    if (!read_double(target) || !read_double(measured) || !read_double(dt) ||
        !(std::cin >> enabled) || (enabled != 0 && enabled != 1)) return 2;
    const auto intent = pi.update_mean(target, measured, dt, enabled == 1);
    std::cout << "{\"effort\":" << intent.effort << ",\"brake_requested\":"
              << (intent.brake_requested ? "true" : "false") << ",\"integral\":" << pi.integral()
              << ",\"valid\":" << (pi.valid() ? "true" : "false")
              << ",\"status\":" << static_cast<int>(intent.status) << "}\n";
  }
}
