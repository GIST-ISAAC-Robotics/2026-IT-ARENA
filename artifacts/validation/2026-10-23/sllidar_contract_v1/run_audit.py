"""고정된 제조사 소스의 발행/정렬/각도 복제만 PC stub으로 실행한다. 장치 I/O 없음."""
import hashlib
import json
from pathlib import Path
import subprocess
import urllib.request

OUT = Path(__file__).resolve().parent
SHA = '34300099fadfc772965962dec837bf436706188f'
REF = f'https://raw.githubusercontent.com/Slamtec/sllidar_ros2/{SHA}/'
FILES = ['src/sllidar_node.cpp', 'sdk/src/sl_lidar_driver.cpp',
         'sdk/include/sl_lidar_driver.h', 'launch/sllidar_c1_launch.py', 'LICENSE']
upstream = OUT/'upstream'
upstream.mkdir(exist_ok=False)
manifest, contents = {}, {}
for name in FILES:
    data = urllib.request.urlopen(REF+name, timeout=30).read(2_000_001)
    if len(data) > 2_000_000:
        raise ValueError('source size limit exceeded')
    path = upstream/name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    manifest[name] = dict(url=REF+name, sha256=hashlib.sha256(data).hexdigest(), bytes=len(data))
    contents[name] = data.decode('utf-8').replace('\r\n', '\n')
(OUT/'source_manifest.json').write_text(json.dumps(dict(commit=SHA, files=manifest), indent=2))


def block(source, marker):
    # 고정한 소스에서 검토한 함수/분기만 추출한다. 전체 driver/main은 실행하지 않는다.
    assert source.count(marker) == 1, marker
    begin = source.index(marker)
    brace = source.index('{', begin)
    depth = 1
    end = brace+1
    while depth:
        if source[end] == '{':
            depth += 1
        elif source[end] == '}':
            depth -= 1
        end += 1
    return source[begin:end], source[brace+1:end-1]


driver, sdk = contents[FILES[0]], contents[FILES[1]]
publish, _ = block(driver, 'void publish_scan(')
_, compensation = block(driver, 'if (angle_compensate)')
selected = [block(sdk, marker)[0] for marker in [
    'static inline float getAngle(const sl_lidar_response_measurement_node_hq_t& node)',
    'static inline void setAngle(sl_lidar_response_measurement_node_hq_t& node, float v)',
    'static inline sl_u32 getDistanceQ2(const sl_lidar_response_measurement_node_hq_t& node)',
    'template <class TNode>\n    static bool angleLessThan',
    'template < class TNode >\n    static sl_result ascendScanData_']]
assert 'start_scan_time = this->now();' in driver
assert 'scan_duration = (end_scan_time - start_scan_time).seconds();' in driver
assert 'grabScanDataHqWithTimeStamp' not in driver
assert 'grabScanDataHqWithTimeStamp' in contents[FILES[2]]

stub = r'''
#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>
using sl_u32 = uint32_t;
using sl_result = int;
constexpr int SL_RESULT_OK = 0, SL_RESULT_OPERATION_FAIL = -1;
struct sl_lidar_response_measurement_node_hq_t {
  uint16_t angle_z_q14;
  uint32_t dist_mm_q2;
  uint8_t quality;
};
namespace rclcpp {
struct Time { double value; };
template<class T> struct Publisher {
  using SharedPtr = std::shared_ptr<Publisher<T>>;
  T last;
  void publish(const T& v) { last=v; }
};
}
namespace sensor_msgs { namespace msg {
struct LaserScan {
  struct Header { rclcpp::Time stamp; std::string frame_id; } header;
  float angle_min=0, angle_max=0, angle_increment=0;
  float scan_time=0, time_increment=0, range_min=0, range_max=0;
  std::vector<float> ranges, intensities;
};
}}
'''
wrapper = r'''
void compensate(rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& scan_pub,
  sl_lidar_response_measurement_node_hq_t* nodes, size_t count,
  size_t angle_compensate_multiple, double scan_duration) {
  const rclcpp::Time start_scan_time{10.};
  const bool inverted=false;
  const float angle_min=0, angle_max=2*M_PI, max_distance=12;
  const std::string frame_id="laser";
'''+compensation+'\n}\n'
main = r'''
using Scan = sensor_msgs::msg::LaserScan;
void print_small(const Scan& msg) {
  std::cout << "{\"stamp_s\":" << msg.header.stamp.value
    << ",\"time_increment_s\":" << msg.time_increment << ",\"ranges\":[";
  for (size_t i=0;i<msg.ranges.size();++i) { if(i) std::cout<<","; std::cout<<msg.ranges[i]; }
  std::cout << "]}";
}
int main() {
  std::cout << std::setprecision(12);
  auto pub=std::make_shared<rclcpp::Publisher<Scan>>();
  sl_lidar_response_measurement_node_hq_t nodes[4]{};
  const float angles[4]={270,0,90,180};
  for (size_t i=0;i<4;++i) {setAngle(nodes[i],angles[i]);nodes[i].dist_mm_q2=(i+1)*4000;}
  assert(ascendScanData_(nodes,4)==0);
  assert(nodes[0].dist_mm_q2==8000 && nodes[3].dist_mm_q2==4000);
  publish_scan(pub,nodes,4,{10.},.1,false,0,2*M_PI,12,"laser");
  auto normal=pub->last;
  assert(normal.ranges==std::vector<float>({1,4,3,2}));
  publish_scan(pub,nodes,4,{10.},.001,false,0,2*M_PI,12,"laser");
  auto ready=pub->last;
  assert(normal.ranges==ready.ranges && normal.time_increment>ready.time_increment*99);
  publish_scan(pub,nodes,4,{10.},0.,false,0,2*M_PI,12,"laser");
  auto zero=pub->last;
  assert(zero.time_increment==0.);
  std::vector<sl_lidar_response_measurement_node_hq_t> dense(500);
  for(size_t i=0;i<dense.size();++i) {
    setAngle(dense[i],360.f*i/dense.size()); dense[i].dist_mm_q2=(1000+i)*4;
  }
  compensate(pub,dense.data(),dense.size(),2,.1);
  const auto compensated=pub->last;
  assert(compensated.ranges.size()==720);
  std::map<float,std::vector<size_t>> by_source;
  size_t finite=0;
  for(size_t i=0;i<compensated.ranges.size();++i)
    if(std::isfinite(compensated.ranges[i])) {++finite;by_source[compensated.ranges[i]].push_back(i);}
  assert(finite>by_source.size());
  const auto duplicate=std::find_if(by_source.begin(),by_source.end(),[](const auto& p){return p.second.size()>1;});
  assert(duplicate!=by_source.end());
  auto sparse=dense;
  for(size_t i=2;i<sparse.size();++i) sparse[i].dist_mm_q2=0;
  compensate(pub,sparse.data(),sparse.size(),2,.1);
  const auto invalid_count=std::count_if(pub->last.ranges.begin(),pub->last.ranges.end(),[](float v){return std::isinf(v);});
  assert(invalid_count>700);
  std::cout << "{\"native_assertions_passed\":true,\"acquisition_order_range_ids\":[1,2,3,4],\"sorted_range_ids\":[2,3,4,1],\"wait_100ms\":";
  print_small(normal);
  std::cout << ",\"wait_1ms\":"; print_small(ready);
  std::cout << ",\"wait_0ms\":"; print_small(zero);
  std::cout << ",\"compensation\":{\"input_count\":500,\"output_count\":720,\"finite_output_count\":" << finite
    << ",\"unique_surviving_source_values\":" << by_source.size()
    << ",\"extra_occurrences_of_same_source\":" << finite-by_source.size()
    << ",\"example_duplicate_indices\":[" << duplicate->second[0] << "," << duplicate->second[1]
    << "],\"artificial_time_separation_s\":" << (duplicate->second[1]-duplicate->second[0])*compensated.time_increment
    << "},\"sparse_input_zero_to_inf_count\":" << invalid_count << "}\n";
}
'''
source = stub+'\n'+ '\n'.join(selected)+'\n'+publish+'\n'+wrapper+'\n'+main
(OUT/'probe.cpp').write_text(source)
commands = [['g++','-std=c++17','-O2','-Wall','-Wextra',str(OUT/'probe.cpp'),'-o',str(OUT/'probe')], [str(OUT/'probe')]]
for label, command in zip(('compile','native'), commands):
    run = subprocess.run(command, capture_output=True, text=True, timeout=60)
    (OUT/(label+'.stdout')).write_text(run.stdout)
    (OUT/(label+'.stderr')).write_text(run.stderr)
    (OUT/(label+'_exit.txt')).write_text(str(run.returncode)+'\n')
    if run.returncode:
        raise RuntimeError(f'{label} failed; preserve evidence, no retry')
native = json.loads(run.stdout)
result = dict(upstream_commit=SHA, source_manifest='source_manifest.json',
    scope='extracted_upstream_functions_with_type_stubs_synthetic_data_no_device_or_ros',
    native=native, sdk_timestamp_api_present=True, ros_node_uses_timestamp_api=False,
    source_files_unchanged=all(hashlib.sha256((upstream/name).read_bytes()).hexdigest()==m['sha256'] for name,m in manifest.items()),
    timing_verified=False, hardware_verified=False, ready_to_drive=False,
    limits=['Type stubs reproduce the used fields, not a full ROS/SDK integration build.',
            'Synthetic wait durations and acquisition order are counterexamples, not measured C1 timing.',
            'Sorting/reversal/bin duplication erase timing provenance; disabling bins alone does not restore it.',
            'SDK timestamp may be estimated and still needs its clock/model/sequence contract checked.'])
with (OUT/'report.json').open('x') as handle:
    json.dump(result,handle,indent=2,allow_nan=False)
print(json.dumps(result))
