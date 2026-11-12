// 기존 probe와 같은 SDK/Standard 모드. JSONL raw 전송만 하며 ROS/구동 출력은 없다.
#include "sl_lidar.h"
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
using namespace sl;
static volatile std::sig_atomic_t interrupted = 0;
static void interrupt_handler(int) { interrupted = 1; }
static long long monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static void require_ok(sl_result r, const char* operation) {
    if (SL_IS_FAIL(r)) throw std::runtime_error(std::string(operation)+" result="+std::to_string(r));
}
struct Cleanup {
    ILidarDriver* driver;
    ~Cleanup() {
        if (driver && driver->isConnected()) {
            std::cerr << "stop_result=" << driver->stop()
                      << " motor_zero_result=" << driver->setMotorSpeed(0) << '\n';
            driver->disconnect();
        }
    }
};
int main(int argc, char** argv) {
    if (argc != 3) { std::cerr << "Usage: stream_c1_native_preview PORT SECONDS\n"; return 2; }
    std::signal(SIGINT, interrupt_handler); std::signal(SIGTERM, interrupt_handler);
    // 출력 파이프가 닫혀도 정리 소멸자를 거쳐 센서를 정지한다.
    std::signal(SIGPIPE, SIG_IGN);
    try {
        const double seconds = std::stod(argv[2]);
        if (!std::isfinite(seconds) || seconds < 1 || seconds > 120)
            throw std::runtime_error("seconds must be within 1..120");
        std::cout.exceptions(std::ios::badbit | std::ios::failbit);
        std::unique_ptr<IChannel> channel(*createSerialPortChannel(argv[1], 460800));
        std::unique_ptr<ILidarDriver> driver(*createLidarDriver());
        if (!channel || !driver) throw std::runtime_error("SDK allocation failed");
        Cleanup cleanup{driver.get()};
        require_ok(driver->connect(channel.get()), "connect");
        sl_lidar_response_device_info_t info{};
        sl_lidar_response_device_health_t health{};
        require_ok(driver->getDeviceInfo(info), "device info");
        require_ok(driver->getHealth(health), "health");
        if (info.model != 65 || health.status != 0) throw std::runtime_error("C1 identity/health differs");
        std::vector<LidarScanMode> modes;
        require_ok(driver->getAllSupportedScanModes(modes), "modes");
        const LidarScanMode* selected = nullptr;
        for (const auto& mode : modes) if (std::string(mode.scan_mode)=="Standard") selected=&mode;
        if (!selected || selected->us_per_sample != 200. || selected->ans_type != 0x81)
            throw std::runtime_error("Standard mode differs");
        require_ok(driver->setMotorSpeed(), "default motor start");
        LidarScanMode actual{};
        require_ok(driver->startScanExpress(false, selected->id, 0, &actual), "scan start");
        std::cerr << "C1 Standard nominal_us=" << actual.us_per_sample << " timing_verified=false\n";
        const auto began=monotonic_ns(); long long first_success=0; size_t scan=0;
        while (!interrupted && (!first_success || (monotonic_ns()-first_success)/1e9 < seconds)) {
            sl_lidar_response_measurement_node_hq_t nodes[8192]; size_t count=8192; sl_u64 first=0;
            const auto begin=monotonic_ns();
            auto result=driver->grabScanDataHqWithTimeStamp(nodes,count,first,2000);
            const auto end=monotonic_ns();
            if (!first_success && result == SL_RESULT_OPERATION_TIMEOUT && (end-began)/1e9 < 15.) {
                std::cerr << "startup_scan_timeout\n";
                continue;
            }
            require_ok(result, "grab scan");
            if (!first_success) first_success=end;
            std::cout << "{\"schema\":\"c1_sdk_native_preview_v1\",\"scan\":" << scan++
                << ",\"begin_monotonic_ns\":" << begin << ",\"end_monotonic_ns\":" << end
                << ",\"sdk_estimated_first_us\":" << first << ",\"us_per_sample\":"
                << std::setprecision(12) << actual.us_per_sample << ",\"nodes\":[";
            for (size_t i=0;i<count;++i) {
                if (i) std::cout << ',';
                std::cout << '[' << nodes[i].angle_z_q14 << ',' << nodes[i].dist_mm_q2
                          << ',' << unsigned(nodes[i].quality) << ',' << unsigned(nodes[i].flag) << ']';
            }
            std::cout << "]}\n" << std::flush;
        }
        return interrupted ? 1 : 0;
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
