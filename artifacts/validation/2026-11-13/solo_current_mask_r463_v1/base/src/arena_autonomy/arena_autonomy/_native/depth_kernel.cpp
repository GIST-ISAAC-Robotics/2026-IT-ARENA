// 명시 선택 수치 커널: 기존 NumPy evaluate(part, chosen)의 순서와 경계를 보존한다.
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

// masked_only의 분류만 계산한다. depth support의 자유 공간 허가와 구별한다.
// status: 0 unknown, 1 return, 2 masked, 3 other. 기존 원점/동률/비순환 인접 규칙 유지.
extern "C" int arena_masked_evaluate(
    int64_t rows, int64_t width, int64_t rays,
    const double* points, const int64_t* chosen, const double* origins,
    const double* directions, const int64_t* indices, const uint8_t* status,
    const double* depth, const double* padding, uint8_t* result) {
    if (rows<0 || width<1 || rays<1) return 1;
    for (int64_t row=0;row<rows;++row) {
        double best_low=std::numeric_limits<double>::infinity(), best_high=best_low;
        double low_dx=0.,low_dy=0.,high_dx=0.,high_dy=0.;
        int64_t lo=-1,hi=-1;
        const double x=points[2*row],y=points[2*row+1];
        for (int64_t col=0;col<width;++col) {
            const int64_t i=chosen[row*width+col];
            if(i<0 || i>rays)return 2;
            if(i==rays)continue;
            const double dx=x-origins[2*i],dy=y-origins[2*i+1];
            const double angle=std::atan2(directions[2*i]*dy-directions[2*i+1]*dx,
                                         directions[2*i]*dx+directions[2*i+1]*dy);
            const double low=angle>0. ? angle : 0., high=-angle>0. ? -angle : 0.;
            if(angle>=-1e-10 && low<best_low){best_low=low;lo=i;low_dx=dx;low_dy=dy;}
            if(angle<=1e-10 && high<best_high){best_high=high;hi=i;high_dx=dx;high_dy=dy;}
        }
        bool okay=lo>=0 && hi>=0 && best_low+best_high<=.025;
        if(okay){
            const uint64_t delta=indices[lo]>=indices[hi] ?
                uint64_t(indices[lo])-uint64_t(indices[hi]) : uint64_t(indices[hi])-uint64_t(indices[lo]);
            okay=delta<=1 && status[lo]!=0 && status[hi]!=0 && (status[lo]==2 || status[hi]==2) &&
                (status[lo]!=1 || depth[lo]-std::hypot(low_dx,low_dy)-padding[lo]>0.) &&
                (status[hi]!=1 || depth[hi]-std::hypot(high_dx,high_dy)-padding[hi]>0.);
        }
        result[row]=okay ? 1 : 0;
    }
    return 0;
}

// 같은 각도 탐색으로 독립적인 free(1)/masked(2) 원 규칙을 모두 계산한다.
extern "C" int arena_support_classes(
    int64_t rows,int64_t width,int64_t rays,int64_t cycle,
    const double* points,const int64_t* chosen,const double* origins,
    const double* directions,const int64_t* indices,const uint8_t* status,
    const double* depth,const double* padding,uint8_t* result) {
    if(rows<0 || width<1 || rays<1)return 1;
    for(int64_t row=0;row<rows;++row){
        double low=std::numeric_limits<double>::infinity(),high=low;
        double la=0.,ha=0.,lx=0.,ly=0.,hx=0.,hy=0.;
        int64_t lo=-1,hi=-1;bool overlap=false;
        const double x=points[2*row],y=points[2*row+1];
        for(int64_t col=0;col<width;++col){
            const int64_t i=chosen[row*width+col];
            if(i<0 || i>rays)return 2;
            if(i==rays)continue;
            const double dx=x-origins[2*i],dy=y-origins[2*i+1];
            if(dx*dx+dy*dy<=1e-18)overlap=true;
            const double angle=std::atan2(directions[2*i]*dy-directions[2*i+1]*dx,
                                         directions[2*i]*dx+directions[2*i+1]*dy);
            const double l=angle>0.?angle:0.,h=-angle>0.?-angle:0.;
            if(angle>=-1e-10 && l<low){low=l;lo=i;la=angle;lx=dx;ly=dy;}
            if(angle<=1e-10 && h<high){high=h;hi=i;ha=angle;hx=dx;hy=dy;}
        }
        bool free=false,masked=false;
        if(lo>=0 && hi>=0){
            const uint64_t delta=indices[lo]>=indices[hi] ?
                uint64_t(indices[lo])-uint64_t(indices[hi]) : uint64_t(indices[hi])-uint64_t(indices[lo]);
            const double lr=std::sqrt(lx*lx+ly*ly),hr=std::sqrt(hx*hx+hy*hy);
            free=!overlap && std::abs(la)+std::abs(ha)<=.025 &&
                (delta<=1 || (cycle>0 && delta==uint64_t(cycle-1))) && status[lo]==1 && status[hi]==1 &&
                lr>1e-9 && hr>1e-9 && depth[lo]-lr-padding[lo]>0. && depth[hi]-hr-padding[hi]>0.;
            masked=low+high<=.025 && delta<=1 && status[lo]!=0 && status[hi]!=0 &&
                (status[lo]==2 || status[hi]==2) &&
                (status[lo]!=1 || depth[lo]-std::hypot(lx,ly)-padding[lo]>0.) &&
                (status[hi]!=1 || depth[hi]-std::hypot(hx,hy)-padding[hi]>0.);
        }
        result[row]=(free?1:0)|(masked?2:0);
    }
    return 0;
}
