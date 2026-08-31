#include "arena_gazebo/drivetrain.hpp"
#include <iostream>
#include <random>

void Require(bool ok, const char *message)
{
  if (!ok) throw std::runtime_error(message);
}

int main()
{
  arena::DriveParameters p;
  arena::SingleMotor open(p);
  auto s = open.Step(1, 10, 30, .001);
  Require(std::abs(s.motorSpeed - 160) < 1e-12, "motor average constraint");
  Require(std::abs(s.leftTorque - s.rightTorque) < 1e-12, "open equal torque");
  Require(std::abs(s.lossPower) < 1e-10, "ideal power balance");
  Require(s.referenceSpeed <= .00301, "acceleration ramp");
  for (int i = 0; i != 10000; ++i) s = open.Step(5.55556, 0, 0, .001);
  Require(std::abs(s.motorTorque) <= p.torqueLimit, "motor torque limit");
  s = open.Step(0, 100, 100, .001);
  Require(s.referenceSpeed == 0, "stop bypasses acceleration ramp");
  for (int i = 0; i != 1000; ++i) s = open.Step(0, 100, 100, .001);
  Require(s.motorTorque < 0 && -s.motorTorque <= p.brakeTorqueLimit, "motor braking");
  open.Reset();
  s = open.Step(0, 0, 0, .001);
  Require(s.motorTorque == 0 && s.leftTorque == 0, "reset safe");
  p.efficiency = .85;
  p.carrierDrag = .0001;
  p.differentialViscosity = .002;
  p.differentialTorqueLimit = .03;
  arena::SingleMotor lossy(p);
  s = lossy.Step(1, 10, 100, .001);
  Require(s.leftTorque > s.rightTorque, "passive coupling favors slower side");
  Require(std::abs(s.differentialTorque) <= .03, "coupling cap");
  std::mt19937 rng(31);
  std::uniform_real_distribution<double> speeds(-400, 400), commands(-8, 8);
  for (int i = 0; i != 20000; ++i)
  {
    s = lossy.Step(commands(rng), speeds(rng), speeds(rng), .001);
    Require(s.lossPower >= -1e-9, "loss never generates power including reverse/braking");
    Require(std::abs(s.referenceSpeed) <= p.maxSpeed, "speed cap");
  }
  const auto left = arena::SteeringAngles(.9, .145, .135, .45);
  const auto right = arena::SteeringAngles(-.9, .145, .135, .45);
  Require(std::abs(left.first - .45) < 1e-12, "inner steering bound");
  Require(left.second < left.first, "outer angle smaller");
  Require(std::abs(left.first + right.second) < 1e-12, "steering mirror");
  bool rejected = false;
  try { p.efficiency = 1.1; arena::SingleMotor invalid(p); }
  catch (const std::invalid_argument &) { rejected = true; }
  Require(rejected, "reject energy generating efficiency");
  std::cout << "single motor equations: PASS (20000 randomized power checks)\n";
}
