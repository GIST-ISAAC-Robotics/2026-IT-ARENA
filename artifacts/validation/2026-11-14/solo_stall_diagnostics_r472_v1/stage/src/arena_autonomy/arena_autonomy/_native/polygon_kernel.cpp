#include <cmath>
#include <cstdint>
#include <limits>

// area 판정은 기존 NumPy 합산을 그대로 사용해 wrapper에서 전달한다.
extern "C" int arena_polygon_distance(int64_t count,int64_t vertices,int area_positive,
    const double* points,const double* polygon,double* output) {
    if(count<0 || vertices<1)return 1;
    for(int64_t i=0;i<count;++i) {
        const double x=points[2*i],y=points[2*i+1];
        double best=std::numeric_limits<double>::infinity();
        bool ccw=true,cw=true;
        for(int64_t j=0;j<vertices;++j) {
            const int64_t next=(j+1)%vertices;
            const double ex=polygon[2*next]-polygon[2*j],ey=polygon[2*next+1]-polygon[2*j+1];
            const double dx=x-polygon[2*j],dy=y-polygon[2*j+1];
            const double norm=ex*ex+ey*ey;
            double u=(dx*ex+dy*ey)/(norm>1e-30 ? norm : 1e-30);
            if(u<0.)u=0.;else if(u>1.)u=1.;
            const double rx=dx-u*ex,ry=dy-u*ey;
            const double distance=std::sqrt(rx*rx+ry*ry);
            if(std::isnan(distance) || distance<best)best=distance;
            const double cross=ex*dy-ey*dx;
            ccw=ccw && cross>=-1e-12;cw=cw && cross<=1e-12;
        }
        output[i]=area_positive && (cw||ccw) ? 0. : best;
    }
    return 0;
}
