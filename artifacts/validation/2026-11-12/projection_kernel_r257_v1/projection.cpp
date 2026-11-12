// 동일 scalar 연산 순서의 원천 측정 투영. unknown/마스크/시각 판단을 완화하지 않는다.
#include <cmath>
#include <cstdint>

extern "C" int arena_project(int64_t count,int64_t sectors,const double* ranges,
    const double* angles,const double* poses,const double* config,const double* masks,double* out) {
    if(count<2 || count>8192 || sectors<0 || sectors>16384)return 1;
    const double sx=config[0],sy=config[1],yaw=config[2],low=config[3],high=config[4],rear=config[5];
    const double tau=0x1.921fb54442d18p+2;
    for(int64_t i=0;i<count;++i) {
        const double distance=ranges[i],dx=poses[3*i],dy=poses[3*i+1],turn=poses[3*i+2];
        const double c=std::cos(turn),s=std::sin(turn);
        const double ox=dx+c*sx-s*sy,oy=dy+s*sx+c*sy;
        const double angle=std::remainder(angles[i],tau);
        const double mask_angle=std::remainder(angle,tau);
        bool masked=std::abs(mask_angle)>rear;
        for(int64_t j=0;j<sectors && !masked;++j)
            masked=std::abs(std::remainder(mask_angle-masks[2*j],tau))<=masks[2*j+1];
        const bool valid=std::isfinite(distance) && distance>=low && distance<=high;
        const int status=masked ? 2 : (valid ? 0 : 1);
        const double bearing=angle+yaw+turn;
        const double ux=std::cos(bearing),uy=std::sin(bearing);
        const double hx=status==0 ? ox+distance*ux : 0.;
        const double hy=status==0 ? oy+distance*uy : 0.;
        if(!std::isfinite(ox)||!std::isfinite(oy)||!std::isfinite(hx)||!std::isfinite(hy)
            ||!std::isfinite(ux)||!std::isfinite(uy)||std::abs(std::hypot(ux,uy)-1.)>1e-10)return 2;
        double* row=out+7*i;
        row[0]=ox;row[1]=oy;row[2]=ux;row[3]=uy;row[4]=hx;row[5]=hy;row[6]=status;
    }
    return 0;
}
