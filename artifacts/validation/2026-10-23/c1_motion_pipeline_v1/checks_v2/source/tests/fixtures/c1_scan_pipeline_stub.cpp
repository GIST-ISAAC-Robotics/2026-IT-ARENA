// Test-only ROS type/transport stub. Vendor function bodies are extracted unchanged
// into vendor_functions.inc after verifying the pinned upstream file hashes.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
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
  void publish(const T& value) { last = value; }
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
#include "vendor_functions.inc"
using Scan = sensor_msgs::msg::LaserScan;
void print_scan(const Scan& scan) {
  std::cout << "{\"stamp_s\":" << scan.header.stamp.value
    << ",\"angle_min\":" << scan.angle_min
    << ",\"angle_increment\":" << scan.angle_increment
    << ",\"time_increment\":" << scan.time_increment
    << ",\"ranges\":[";
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    if (i) std::cout << ',';
    if (std::isfinite(scan.ranges[i])) std::cout << scan.ranges[i];
    else std::cout << "null";
  }
  std::cout << "]}";
}
int main() {
  size_t count, multiple;
  double stamp, duration;
  if (!(std::cin >> count >> multiple >> stamp >> duration)
      || count < 2 || count > 8192 || multiple != 2
      || !std::isfinite(stamp) || !std::isfinite(duration)
      || stamp < 0 || duration < 0) return 2;
  std::vector<sl_lidar_response_measurement_node_hq_t> nodes(count), labels(count);
  for (size_t i=0; i<count; ++i) {
    unsigned angle;
    uint64_t distance;
    if (!(std::cin >> angle >> distance) || angle > 65535
        || distance > UINT32_MAX) return 3;
    nodes[i] = {static_cast<uint16_t>(angle), static_cast<uint32_t>(distance), 0};
    // Separate diagnostic pass through the SAME native functions. Range magnitude
    // never controls sorting/binning (only zero/nonzero does). Not sensor evidence.
    labels[i] = {nodes[i].angle_z_q14, distance ? uint32_t((i+1)*4000) : 0, 0};
  }
  if (ascendScanData_(nodes.data(), count) != 0
      || ascendScanData_(labels.data(), count) != 0) return 4;
  auto pub = std::make_shared<rclcpp::Publisher<Scan>>();
  compensate(pub, nodes.data(), count, multiple, stamp, duration);
  std::cout << std::setprecision(17) << "{\"measured\":";
  print_scan(pub->last);
  compensate(pub, labels.data(), count, multiple, stamp, duration);
  std::cout << ",\"provenance_labels_only\":";
  print_scan(pub->last);
  std::cout << "}\n";
}
