// PC-only harness. The normal-node handler and wire types are unchanged vendor code.
// No serial/network/ROS, clock reads, motor commands, or driver initialization.
#include "sdk/src/dataunpacker/dataunnpacker_commondef.h"
#include "sdk/src/dataunpacker/dataunpacker.h"
#include "sdk/src/dataunpacker/dataunnpacker_internal.h"
#include "sdk/src/dataunpacker/unpacker/handler_normalnode.h"
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace sl { namespace internal {
// Match the trivial upstream base constructor/destructor; do not build its registry.
LIDARSampleDataUnpacker::LIDARSampleDataUnpacker(LIDARSampleDataListener& l): _listener(l) {}
LIDARSampleDataUnpacker::~LIDARSampleDataUnpacker() {}
}}
using namespace sl;
using namespace sl::internal;

class Listener : public LIDARSampleDataListener {
public:
    void onHQNodeScanResetReq() override {}
    void onHQNodeDecoded(_u64, const rplidar_response_measurement_node_hq_t*) override {}
};

class Engine : public LIDARSampleDataUnpackerInner {
public:
    explicit Engine(Listener& l): LIDARSampleDataUnpackerInner(l) {}
    _u64 clock = 0, step = 0, last_clock = 0;
    size_t action = 0, sequence = 0, offset = 0, length = 0;
    void updateUnpackerContext(UnpackerContextType, const void*, size_t) override {}
    void enable() override {}
    void disable() override {}
    bool onSampleData(_u8, const void*, size_t) override { throw std::logic_error("not a transport"); }
    void reset() override {}
    void clearCache() override {}
    void publishDecodingErrorMsg(int, _u8, const void*, size_t) override { throw std::logic_error("unexpected callback"); }
    void publishCustomData(_u8, _u32, const void*, size_t) override { throw std::logic_error("unexpected callback"); }
    void publishNewScanReset() override { throw std::logic_error("unexpected callback"); }
    _u64 getCurrentTimestamp_uS() override {
        last_clock = clock;
        if (step > std::numeric_limits<_u64>::max() - clock) throw std::overflow_error("clock");
        clock += step;
        return last_clock;
    }
    void publishHQNode(_u64 ts, const rplidar_response_measurement_node_hq_t* n) override {
        std::cout << "{\"kind\":\"point\",\"sequence\":" << sequence++
                  << ",\"action\":" << action << ",\"completion_chunk_offset\":" << offset
                  << ",\"completion_chunk_bytes\":" << length
                  << ",\"decode_clock_us\":" << last_clock << ",\"sdk_estimate_us\":" << ts
                  << ",\"angle_q14\":" << n->angle_z_q14 << ",\"distance_q2\":" << n->dist_mm_q2
                  << ",\"quality\":" << unsigned(n->quality) << ",\"flag\":" << unsigned(n->flag) << "}\n";
    }
};

class ProbeHandler : public unpacker::UnpackerHandler_NormalNode {
public:
    int pending() const { return _cached_scan_node_buf_pos; }
};

static unsigned nibble(char c) {
    if (c >= '0' && c <= '9') return unsigned(c - '0');
    if (c >= 'a' && c <= 'f') return unsigned(c - 'a' + 10);
    throw std::invalid_argument("lowercase hex required");
}

int main() try {
    static_assert(sizeof(rplidar_response_measurement_node_t) == 5, "wire ABI");
    static_assert(sizeof(rplidar_response_measurement_node_hq_t) == 8, "HQ ABI");
    SlamtecLidarTimingDesc timing{};
    unsigned interface_type = 0;
    size_t actions = 0, offset = 0;
    if (!(std::cin >> timing.sample_duration_uS >> timing.native_baudrate
          >> timing.linkage_delay_uS >> interface_type >> actions) || actions > 10000)
        throw std::invalid_argument("fixture header");
    timing.native_interface_type = static_cast<LIDARInterfaceType>(interface_type);
    timing.native_timestamp_support = false;
    ProbeHandler handler;
    handler.onUnpackerContextSet(LIDARSampleDataUnpacker::UNPACKER_CONTEXT_TYPE_LIDAR_TIMING,
                                &timing, sizeof(timing));
    Listener listener;
    Engine engine(listener);
    for (size_t i = 0; i < actions; ++i) {
        char command;
        if (!(std::cin >> command)) throw std::invalid_argument("missing action");
        engine.action = i;
        if (command == 'R') { handler.reset(); continue; }
        std::string hex;
        if (command != 'D' || !(std::cin >> engine.clock >> engine.step >> hex)
            || hex.empty() || hex.size() % 2 || hex.size() > 131072)
            throw std::invalid_argument("bounded chunk required");
        std::vector<_u8> data;
        for (size_t j = 0; j < hex.size(); j += 2) data.push_back(nibble(hex[j])*16+nibble(hex[j+1]));
        if (offset + data.size() > 131072) throw std::invalid_argument("fixture byte limit");
        engine.offset = offset;
        engine.length = data.size();
        handler.onData(&engine, data.data(), data.size());
        offset += data.size();
    }
    std::string trailing;
    if (std::cin >> trailing) throw std::invalid_argument("trailing input");
    std::cout << "{\"kind\":\"end\",\"points\":" << engine.sequence
              << ",\"pending_bytes\":" << handler.pending() << ",\"input_bytes\":" << offset
              << ",\"answer_type\":" << unsigned(handler.getSampleAnswerType()) << "}\n";
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
}
