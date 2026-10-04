#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace arena
{
// 질량 없는 강체 기어의 축약 모델. 바퀴 관성/접촉은 Gazebo가 적분합니다.
// 모터 회전자 반사 관성, 기어 유격, 전기/열/PWM/단락 제동 회로를 재현하는 모델은 아닙니다.
// 실행값은 vehicle.yaml → SDF로 주입합니다. 아래 기본값은 2026-10-02 설정과 같게 둔
// 대체값이며, 과거 독립 시험(반지름 0.025 m·8:1·2200 rad/s)의 값이 아닙니다.
struct DriveParameters
{
  double radius{0.0325};             // 65 mm 바깥지름, 유효 구름 반지름 미측정
  double ratio{15.0};                // 팀 전달 초기 6:1 × 2.5:1, 조립 미확인
  double efficiency{1.0};
  double torqueLimit{0.060};          // 모터 축 N m 임시 상한. MB4266 정지 토크 외삽값 아님
  double brakeTorqueLimit{0.045};     // 임시 제동 상한. MD20A 단락 제동 모델 아님
  double freeSpeed{1884.9555921539}; // MB4266-24180 24 V 무부하 18,000 rpm 사양, 실측 아님
  double responseTime{0.040};
  double speedKp{0.10};              // N m / (m/s), 단일 평균속도 PI
  double speedKi{0.20};
  double maxSpeed{5.5555555556};
  double acceleration{3.0};
  double carrierDrag{0.0};           // N m / (rad/s), 캐리어 기준
  double differentialViscosity{0.0}; // N m / (rad/s), 좌우 출력축 사이
  double differentialTorqueLimit{0.0};

  void Validate() const
  {
    const double positive[]{radius, ratio, efficiency, torqueLimit, brakeTorqueLimit,
      freeSpeed, responseTime, speedKp, maxSpeed, acceleration};
    for (const auto value : positive)
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("구동 매개변수는 유한한 양수여야 합니다.");
    const double nonnegative[]{speedKi, carrierDrag, differentialViscosity,
      differentialTorqueLimit};
    for (const auto value : nonnegative)
      if (!std::isfinite(value) || value < 0)
        throw std::invalid_argument("손실/이득 매개변수는 유한한 비음수여야 합니다.");
    if (efficiency > 1) throw std::invalid_argument("기어 효율은 1 이하여야 합니다.");
  }
};

struct DriveState
{
  double motorSpeed{}, motorTorque{}, carrierSpeed{}, leftTorque{}, rightTorque{};
  double differentialTorque{}, referenceSpeed{}, speedError{}, lossPower{};
};

class SingleMotor
{
public:
  explicit SingleMotor(DriveParameters parameters = {}) : p(parameters) { p.Validate(); }
  void Reset() { integral = torque = reference = 0; }

  DriveState Step(double target, double leftSpeed, double rightSpeed, double dt)
  {
    if (!std::isfinite(target) || !std::isfinite(leftSpeed) ||
        !std::isfinite(rightSpeed) || !std::isfinite(dt) || dt <= 0)
      throw std::invalid_argument("비유한 구동 입력 또는 잘못된 시간 간격입니다.");
    DriveState s;
    s.carrierSpeed = (leftSpeed + rightSpeed) / 2;
    s.motorSpeed = p.ratio * s.carrierSpeed;
    target = std::clamp(target, -p.maxSpeed, p.maxSpeed);
    // 증속만 램프 제한합니다. 정지/감속 요청은 실제 토크 한계로 제동합니다.
    if (target * reference < 0 || std::abs(target) <= std::abs(reference)) reference = target;
    else reference += std::clamp(target - reference, -p.acceleration * dt, p.acceleration * dt);
    s.referenceSpeed = reference;
    s.speedError = reference - p.radius * s.carrierSpeed;
    const double raw = p.speedKp * s.speedError + integral;
    const bool braking = raw * s.motorSpeed < 0;
    const double limit = braking ? p.brakeTorqueLimit :
      p.torqueLimit * std::max(0.0, 1.0 - std::abs(s.motorSpeed) / p.freeSpeed);
    const double demand = std::clamp(raw, -limit, limit);
    // 포화 시 같은 방향의 적분 누적을 막습니다.
    if (std::abs(raw) <= limit || raw * s.speedError < 0)
      integral = std::clamp(integral + p.speedKi * s.speedError * dt,
                            -p.torqueLimit, p.torqueLimit);
    if (target == 0 && std::abs(s.carrierSpeed * p.radius) < 0.008) integral = 0;
    torque += (demand - torque) * (1 - std::exp(-dt / p.responseTime));
    torque = std::clamp(torque, -limit, limit);
    s.motorTorque = torque;
    // 역구동에서도 손실이 에너지를 생성하지 않도록 동력 방향을 구분합니다.
    const bool backDriven = torque * s.motorSpeed < 0;
    const double geared = torque * p.ratio * (backDriven ? 1 / p.efficiency : p.efficiency);
    const double carrierTorque = geared - p.carrierDrag * s.carrierSpeed;
    const double difference = leftSpeed - rightSpeed;
    s.differentialTorque = std::clamp(p.differentialViscosity * difference,
      -p.differentialTorqueLimit, p.differentialTorqueLimit);
    s.leftTorque = carrierTorque / 2 - s.differentialTorque;
    s.rightTorque = carrierTorque / 2 + s.differentialTorque;
    s.lossPower = torque * s.motorSpeed -
      (s.leftTorque * leftSpeed + s.rightTorque * rightSpeed);
    return s;
  }
private:
  DriveParameters p;
  double integral{}, torque{}, reference{};
};

inline std::pair<double, double> SteeringAngles(double central, double wheelbase,
                                               double track, double wheelLimit)
{
  const double centralLimit = std::atan(wheelbase /
    (wheelbase / std::tan(wheelLimit) + track / 2));
  central = std::clamp(central, -centralLimit, centralLimit);
  const double curvature = std::tan(central) / wheelbase;
  return {std::atan(wheelbase * curvature / (1 - track * curvature / 2)),
          std::atan(wheelbase * curvature / (1 + track * curvature / 2))};
}
}  // namespace arena
