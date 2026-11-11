// Isolated performance experiment. No fast-math; near numerical boundaries
// return uncertain so the original NumPy implementation remains authoritative.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" int depth_evaluate(
    std::int64_t rows, std::int64_t rays, std::int64_t width, std::int64_t cycle,
    const double* q, const double* origin, const double* direction,
    const std::int64_t* indices, const std::uint8_t* valid,
    const double* depth, const double* pad, const std::int64_t* chosen,
    std::uint8_t* supported, std::uint8_t* uncertain) {
  if (rows < 0 || rays <= 0 || width <= 0 || !q || !origin || !direction ||
      !indices || !valid || !depth || !pad || !chosen || !supported || !uncertain)
    return -1;
  const double inf = std::numeric_limits<double>::infinity();
  for (std::int64_t row=0; row<rows; ++row) {
    supported[row]=0; uncertain[row]=0;
    double low=inf, high=inf, second_low=inf, second_high=inf;
    double low_angle=0., high_angle=0., low_r2=0., high_r2=0.;
    std::int64_t lo=-1, hi=-1;
    bool overlap=false;
    for (std::int64_t col=0; col<width; ++col) {
      const auto ray=chosen[row*width+col];
      if (ray == rays) continue; // Existing padded sentinel.
      if (ray < 0 || ray >= rays) return -2;
      const double dx=q[2*row]-origin[2*ray];
      const double dy=q[2*row+1]-origin[2*ray+1];
      const double radius2=dx*dx+dy*dy;
      overlap |= radius2 <= 1e-18;
      if (std::abs(radius2-1e-18) < 1e-24) uncertain[row]=1;
      const double cross=direction[2*ray]*dy-direction[2*ray+1]*dx;
      const double dot=direction[2*ray]*dx+direction[2*ray+1]*dy;
      const double angle=std::atan2(cross,dot);
      if (!std::isfinite(angle)) continue;
      if (std::abs(angle-1e-10)<1e-12 || std::abs(angle+1e-10)<1e-12)
        uncertain[row]=1;
      if (angle>=-1e-10) {
        const double score=std::max(angle,0.);
        if (score<low) {
          second_low=low; low=score; lo=ray; low_angle=angle; low_r2=radius2;
        } else second_low=std::min(second_low,score);
      }
      if (angle<=1e-10) {
        const double score=std::max(-angle,0.);
        if (score<high) {
          second_high=high; high=score; hi=ray; high_angle=angle; high_r2=radius2;
        } else second_high=std::min(second_high,score);
      }
    }
    if (overlap || lo<0 || hi<0) continue;
    // Ties clamped exactly to zero retain the first original index in both paths.
    if ((low>0. && second_low-low < 1e-12) || (high>0. && second_high-high < 1e-12))
      uncertain[row]=1;
    const double angle_sum=std::abs(low_angle)+std::abs(high_angle);
    const double lr=std::sqrt(low_r2), hr=std::sqrt(high_r2);
    const double lm=depth[lo]-lr-pad[lo], hm=depth[hi]-hr-pad[hi];
    if (std::abs(angle_sum-.025)<1e-12 || std::abs(lr-1e-9)<1e-15 ||
        std::abs(hr-1e-9)<1e-15 || std::abs(lm)<1e-11 || std::abs(hm)<1e-11)
      uncertain[row]=1;
    const auto delta=std::abs(indices[lo]-indices[hi]);
    const bool adjacent=delta<=1 || (cycle>0 && delta==cycle-1);
    supported[row]=angle_sum<=.025 && adjacent && valid[lo] && valid[hi] &&
                   lr>1e-9 && hr>1e-9 && lm>0. && hm>0.;
  }
  return 0;
}
