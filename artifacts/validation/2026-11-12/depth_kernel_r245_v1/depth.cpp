// 비교 전용: 기존 NumPy evaluate(part, chosen)의 순서와 경계를 보존한다.
#include <cmath>
#include <cstdint>
#include <limits>

extern "C" int arena_depth_evaluate(
    int64_t rows, int64_t width, int64_t rays, int64_t cycle,
    const double* points, const int64_t* chosen, const double* origins,
    const double* directions, const int64_t* indices, const uint8_t* valid,
    const double* depth, const double* padding, uint8_t* result) {
    if (rows<0 || width<1 || rays<1) return 1;
    for (int64_t row=0;row<rows;++row) {
        double best_low=std::numeric_limits<double>::infinity();
        double best_high=best_low, low_angle=0., high_angle=0., low_r2=0., high_r2=0.;
        int64_t lo=-1,hi=-1;
        bool overlap=false;
        const double x=points[2*row], y=points[2*row+1];
        for (int64_t col=0;col<width;++col) {
            int64_t i=chosen[row*width+col];
            if (i<0 || i>rays) return 2;
            if (i==rays) continue;  // NumPy present=False sentinel
            const double dx=x-origins[2*i],dy=y-origins[2*i+1];
            const double r2=dx*dx+dy*dy;
            if (r2<=1e-18) overlap=true;
            const double angle=std::atan2(directions[2*i]*dy-directions[2*i+1]*dx,
                                          directions[2*i]*dx+directions[2*i+1]*dy);
            // np.argmin: 동률이면 chosen의 첫 원소를 유지한다.
            const double low=angle>0. ? angle : 0.;
            const double high=-angle>0. ? -angle : 0.;
            if (angle>=-1e-10 && low<best_low) {
                best_low=low;lo=i;low_angle=angle;low_r2=r2;
            }
            if (angle<=1e-10 && high<best_high) {
                best_high=high;hi=i;high_angle=angle;high_r2=r2;
            }
        }
        bool okay=!overlap && lo>=0 && hi>=0;
        if (okay) {
            const int64_t delta=indices[lo]>indices[hi] ? indices[lo]-indices[hi] : indices[hi]-indices[lo];
            const double low_radius=std::sqrt(low_r2),high_radius=std::sqrt(high_r2);
            okay=std::abs(low_angle)+std::abs(high_angle)<=.025 &&
                 (delta<=1 || (cycle>=0 && delta==cycle-1)) && valid[lo] && valid[hi] &&
                 low_radius>1e-9 && high_radius>1e-9 &&
                 depth[lo]-low_radius-padding[lo]>0. && depth[hi]-high_radius-padding[hi]>0.;
        }
        result[row]=okay ? 1 : 0;
    }
    return 0;
}
