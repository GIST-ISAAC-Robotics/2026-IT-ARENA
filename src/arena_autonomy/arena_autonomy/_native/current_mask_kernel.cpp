// Actual returned-hit padding only; no free-space or bracket/structure policy.
#include <cmath>
#include <cstdint>

extern "C" int arena_current_mask_occupied(
    int64_t queries, int64_t hits, const double* points,
    const double* returned, const double* padding, uint8_t* result) {
    if(queries<0 || hits<0)return 1;
    result[0]=0;
    for(int64_t q=0;q<queries;++q){
        for(int64_t h=0;h<hits;++h){
            const double dx=points[2*q]-returned[2*h];
            const double dy=points[2*q+1]-returned[2*h+1];
            // Preserve NumPy norm's multiply/add/sqrt order and inclusive <=.
            // Existing build requires -ffp-contract=off and -fno-fast-math.
            const double distance=std::sqrt(dx*dx+dy*dy);
            if(distance<=padding[h]){result[0]=1;return 0;}
        }
    }
    return 0;
}
