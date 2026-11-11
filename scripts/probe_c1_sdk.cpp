// 정렬·각도 칸 보완 전 제조사 SDK 반환을 보존하는 구동 없는 C1 진단.
// 차량 모터가 아닌 LiDAR 내부 회전 모터만 SDK를 통해 시작/정지한다.
#include "sl_lidar.h"
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
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
static void require_ok(sl_result result, const char* what) {
    if (SL_IS_FAIL(result)) throw std::runtime_error(
        std::string(what) + " result=" + std::to_string(result));
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
    if (argc < 5 || argc > 8) {
        std::cerr << "Usage: probe_c1_sdk PORT OUTPUT_PREFIX DURATION_S WARMUP_S [MODE [RPM [--rpm-after-warmup]]]\n";
        return 2;
    }
    std::signal(SIGINT, interrupt_handler);
    std::signal(SIGTERM, interrupt_handler);
    try {
        const double duration = std::stod(argv[3]), warmup = std::stod(argv[4]);
        if (!(duration > 0 && warmup >= 0)) throw std::runtime_error("invalid duration");
        const std::string requested_mode = argc >= 6 ? argv[5] : "Standard";
        const int requested_rpm = argc >= 7 ? std::stoi(argv[6]) : -1;
        const bool delayed_rpm = argc == 8 && std::string(argv[7]) == "--rpm-after-warmup";
        if (argc == 8 && (!delayed_rpm || requested_rpm == -1))
            throw std::runtime_error("invalid delayed RPM option");
        if (requested_mode != "Standard" && requested_mode != "DenseBoost")
            throw std::runtime_error("unsupported probe mode");
        if (requested_rpm != -1 && (requested_rpm < 480 || requested_rpm > 720))
            throw std::runtime_error("probe RPM must stay within 480..720");
        const std::string prefix = argv[2];
        for (const auto suffix : {"-scans.csv", "-points.csv", "-metadata.json"}) {
            if (std::ifstream(prefix + suffix).good()) throw std::runtime_error("output exists");
        }
        std::ofstream scans(prefix + "-scans.csv"), points(prefix + "-points.csv");
        scans.exceptions(std::ios::failbit | std::ios::badbit);
        points.exceptions(std::ios::failbit | std::ios::badbit);
        scans << "scan,begin_monotonic_ns,end_monotonic_ns,sdk_estimated_first_us,count,result,measured\n";
        points << "scan,index,angle_q14,dist_mm_q2,quality,flag\n";
        std::unique_ptr<IChannel> channel(*createSerialPortChannel(argv[1], 460800));
        std::unique_ptr<ILidarDriver> driver(*createLidarDriver());
        if (!channel || !driver) throw std::runtime_error("SDK allocation failed");
        Cleanup cleanup{driver.get()};
        require_ok(driver->connect(channel.get()), "connect");
        sl_lidar_response_device_info_t info{};
        sl_lidar_response_device_health_t health{};
        require_ok(driver->getDeviceInfo(info), "device info");
        require_ok(driver->getHealth(health), "health");
        if (health.status != 0) throw std::runtime_error("device health not OK");
        MotorCtrlSupport motor_support=MotorCtrlSupportNone;
        const auto motor_query_result=driver->checkMotorCtrlSupport(motor_support);
        LidarMotorInfo motor_info{};
        motor_info.motorCtrlSupport=motor_support;
        const auto motor_info_result=SL_IS_OK(motor_query_result)
            ? driver->getMotorInfo(motor_info) : motor_query_result;
        std::vector<LidarScanMode> modes;
        require_ok(driver->getAllSupportedScanModes(modes), "scan modes");
        const LidarScanMode* selected = nullptr;
        for (const auto& mode : modes) if (std::string(mode.scan_mode) == requested_mode) selected = &mode;
        if (!selected) throw std::runtime_error("requested scan mode absent");
        if (requested_rpm != -1 && (SL_IS_FAIL(motor_query_result) || motor_support != MotorCtrlSupportRpm))
            throw std::runtime_error("RPM control not verified by SDK motor support query");
        require_ok(driver->setMotorSpeed(), "motor start");
        LidarScanMode actual{};
        require_ok(driver->startScanExpress(false, selected->id, 0, &actual), "scan start");
        if (requested_rpm != -1 && !delayed_rpm)
            require_ok(driver->setMotorSpeed(static_cast<sl_u16>(requested_rpm)), "RPM set");
        std::ofstream metadata(prefix + "-metadata.json");
        metadata << std::setprecision(12)
                 << "{\"baud\":460800,\"model_byte\":" << unsigned(info.model)
                 << ",\"firmware_major\":" << unsigned(info.firmware_version >> 8)
                 << ",\"firmware_minor\":" << unsigned(info.firmware_version & 255)
                 << ",\"hardware\":" << unsigned(info.hardware_version)
                 << ",\"health\":" << unsigned(health.status)
                 << ",\"motor_control_support\":" << int(motor_support)
                 << ",\"motor_query_result\":" << motor_query_result
                 << ",\"motor_info_result\":" << motor_info_result
                 << ",\"reported_min_speed\":" << motor_info.min_speed
                 << ",\"reported_max_speed\":" << motor_info.max_speed
                 << ",\"reported_desired_speed\":" << motor_info.desired_speed
                 << ",\"requested_rpm\":" << requested_rpm
                 << ",\"rpm_after_stream_warmup\":" << (delayed_rpm ? "true" : "false")
                 << ",\"duration_s\":" << duration << ",\"warmup_s\":" << warmup
                 << ",\"mode\":\"" << actual.scan_mode << "\",\"us_per_sample\":"
                 << actual.us_per_sample << ",\"max_distance_m\":" << actual.max_distance
                 << ",\"answer_type\":" << unsigned(actual.ans_type) << ",\"supported_modes\":[";
        for (size_t i=0; i<modes.size(); ++i) {
            const auto& mode=modes[i];
            if (i) metadata << ',';
            metadata << "{\"name\":\"" << mode.scan_mode << "\",\"id\":" << mode.id
                     << ",\"us_per_sample\":" << mode.us_per_sample
                     << ",\"answer_type\":" << unsigned(mode.ans_type) << '}';
        }
        metadata << "]}\n";
        metadata.close();
        std::cout << "model=" << unsigned(info.model) << " firmware="
                  << (info.firmware_version >> 8) << '.' << (info.firmware_version & 255)
                  << " health=" << unsigned(health.status) << " mode=" << actual.scan_mode
                  << " us_per_sample=" << actual.us_per_sample
                  << " ans_type=" << unsigned(actual.ans_type) << std::endl;
        const long long start=monotonic_ns();
        long long first_success=0;
        long long measured_start=0;
        bool rpm_applied=!delayed_rpm;
        size_t scan=0;
        while (!interrupted) {
            sl_lidar_response_measurement_node_hq_t nodes[8192];
            size_t count=8192;
            sl_u64 sdk_first=0;
            const auto begin=monotonic_ns();
            const auto result=driver->grabScanDataHqWithTimeStamp(nodes,count,sdk_first,2000);
            const auto end=monotonic_ns();
            if (SL_IS_OK(result) && !rpm_applied && first_success && (end-first_success)/1e9 >= warmup) {
                require_ok(driver->setMotorSpeed(static_cast<sl_u16>(requested_rpm)), "delayed RPM set");
                std::cerr << "delayed_rpm_sent=" << requested_rpm << " monotonic_ns=" << monotonic_ns() << '\n';
                first_success=end;
                rpm_applied=true;
            }
            const bool measured=first_success && (end-first_success)/1e9 >= warmup;
            scans << scan << ',' << begin << ',' << end << ',' << sdk_first << ','
                  << (SL_IS_OK(result) ? count : 0) << ',' << result << ',' << measured << '\n';
            scans.flush();
            if (SL_IS_FAIL(result)) {
                ++scan;
                if (!first_success && (end-start)/1e9 < 15) continue;
                require_ok(result,"grab scan");
            }
            if (!first_success) first_success=end;
            for (size_t i=0;i<count;++i) points << scan << ',' << i << ',' << nodes[i].angle_z_q14
                << ',' << nodes[i].dist_mm_q2 << ',' << unsigned(nodes[i].quality)
                << ',' << unsigned(nodes[i].flag) << '\n';
            if (measured && !measured_start) measured_start=end;
            ++scan;
            if (measured_start && (end-measured_start)/1e9 >= duration) break;
        }
        std::cout << "scans=" << scan << " interrupted=" << interrupted << std::endl;
        if (requested_rpm != -1)
            require_ok(driver->setMotorSpeed(), "restore default motor speed");
        return interrupted ? 1 : 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
