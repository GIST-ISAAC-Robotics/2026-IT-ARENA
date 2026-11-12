#include "steering_mapping.hpp"
#include "st3215_transaction.hpp"
#include <cstdio>
#include <limits>
#include <vector>
using arena_steering::Mapping;
using arena_steering::Sample;
static bool check(bool ok,const char* name){std::printf("%s %s\n",ok?"PASS":"FAIL",name);return ok;}
int main(){
  // 전부 합성 예제다. 실제 중립·좌우 각도·기계 한계가 아니다.
  const Sample example[]={{-.4,1600},{-.2,1850},{0.,2050},{.2,2230},{.4,2380}};
  Mapping map(example,5);
  if(!check(map.valid() && map.covers(-.4,.4) && !map.covers(-.41,.4),"declared_controller_range"))return 1;
  if(!check(!Mapping().goal(0.).valid,"absent_calibration_no_goal"))return 1;
  if(!check(map.goal(-.4).position==1600 && map.goal(0).position==2050 && map.goal(.4).position==2380,
            "measured_points_exact"))return 1;
  if(!check(map.goal(-.1).position==1950 && map.goal(.1).position==2140,"asymmetric_linkage_interpolation"))return 1;
  const Sample reverse[]={{-.4,2380},{-.2,2230},{0.,2050},{.2,1850},{.4,1600}};
  Mapping inverted(reverse,5);
  if(!check(inverted.valid() && inverted.goal(.1).position==1950,"opposite_servo_orientation"))return 1;
  if(!check(!map.goal(.401).valid && !map.goal(-.401).valid &&
            !map.goal(std::numeric_limits<double>::quiet_NaN()).valid,"no_extrapolation_or_nonfinite_goal"))return 1;
  const Sample fold[]={{-.4,1600},{0.,2050},{.4,1900}};
  const Sample no_zero[]={{-.4,1600},{.1,2050},{.4,2380}};
  const Sample outside[]={{-.4,1600},{0.,2050},{.4,4096}};
  if(!check(!Mapping(fold,3).valid() && !Mapping(no_zero,3).valid() && !Mapping(outside,3).valid(),
            "reject_wrong_measurement_table"))return 1;
  arena_servo::Transaction wire;
  const auto goal=map.goal(.1);
  if(!goal.valid || !wire.goal(1000,1,goal.position,200,10))return 1;
  std::vector<unsigned char> packet;
  wire.pump(1001,[&](const unsigned char* p,std::size_t n){packet.insert(packet.end(),p,p+n);return n;});
  unsigned checksum=0;for(std::size_t i=2;i<packet.size();++i)checksum+=packet[i];
  if(!check(packet.size()==14 && packet[5]==41 && (packet[7]|(unsigned(packet[8])<<8))==2140 &&
            (checksum&255)==255,"vehicle_angle_to_existing_RAM_goal_frame"))return 1;
}
