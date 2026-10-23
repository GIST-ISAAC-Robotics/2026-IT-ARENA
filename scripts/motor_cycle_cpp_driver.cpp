// PC 전용 합성 검사. 어떤 장치 출력도 호출하지 않는다.
#include "motor_cycle_cases.hpp"

int main(int argc, char** argv) {
  static arena_smoke::Scratch scratch;
  if (argc != 2) return 2;
  const bool ok = arena_smoke::motor_cycle_case(argv[1], scratch);
  std::printf("%s:%s\n", argv[1], ok ? "PASS" : "FAIL");
  return ok ? 0 : 1;
}
