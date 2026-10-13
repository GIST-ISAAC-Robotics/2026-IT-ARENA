// Installed DART algorithm diagnostic only. No Gazebo world or vehicle model.
#include <dart/constraint/BoxedLcpConstraintSolver.hpp>
#include <dart/constraint/DantzigBoxedLcpSolver.hpp>
#include <dart/constraint/PgsBoxedLcpSolver.hpp>
#include <dart/simulation/World.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>

struct Result { bool ok; double n, t; };
template<typename Solver> Result Probe()
{
  // SPD Delassus matrix with normal/tangent coupling, stored with dPAD(2)=4.
  double A[8] = {1., -.5, 0., 0., -.5, 1., 0., 0.};
  double x[2] = {0., 0.}, b[2] = {1., -1.};
  double lo[2] = {0., -.9}, hi[2] = {std::numeric_limits<double>::infinity(), .9};
  int findex[2] = {-1, 0};
  Solver solver;
  bool ok = solver.solve(2, A, x, b, 0, lo, hi, findex, false);
  return {ok, x[0], x[1]};
}
int main()
{
  dart::simulation::World world;
  auto *boxed = dynamic_cast<dart::constraint::BoxedLcpConstraintSolver *>(world.getConstraintSolver());
  const auto d = Probe<dart::constraint::DantzigBoxedLcpSolver>();
  const auto p = Probe<dart::constraint::PgsBoxedLcpSolver>();
  const bool reproduced = d.ok && p.ok && d.n > 0 && p.n > 0 &&
      std::abs(d.t) > .9*d.n*1.1 && std::abs(p.t) <= .9*p.n*1.001;
  std::cout << std::setprecision(15) << "{\"default_primary\":\""
      << (boxed ? boxed->getBoxedLcpSolver()->getType() : "unknown")
      << "\",\"dantzig\":{\"ok\":" << (d.ok ? "true" : "false")
      << ",\"normal\":" << d.n << ",\"tangent\":" << d.t
      << ",\"bound_ratio\":" << std::abs(d.t)/(.9*d.n)
      << "},\"pgs\":{\"ok\":" << (p.ok ? "true" : "false")
      << ",\"normal\":" << p.n << ",\"tangent\":" << p.t
      << ",\"bound_ratio\":" << std::abs(p.t)/(.9*p.n)
      << "},\"mechanism_reproduced\":" << (reproduced ? "true" : "false") << "}\n";
  return reproduced ? 0 : 1;
}
