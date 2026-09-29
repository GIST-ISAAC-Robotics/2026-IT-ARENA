// Read-only in-process audit, compiled against the installed Fast DDS library.
// No participant is created, removed or reconfigured here.
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <sstream>
#include <string>
#include <iomanip>

extern "C" const char* arena_dds_transport_json(unsigned int domain) noexcept
{
    static thread_local std::string result;
    try {
        namespace fastdds = eprosima::fastdds;
        auto participants = fastdds::dds::DomainParticipantFactory::get_instance()->lookup_participants(domain);
        std::ostringstream out;
        out << "{\"participants\":[";
        bool first = true;
        for (auto* participant : participants) {
            if (!first) out << ',';
            first = false;
            const auto& transport = participant->get_qos().transport();
            out << "{\"builtin\":" << (transport.use_builtin_transports ? "true" : "false")
                << ",\"transports\":[";
            bool first_transport = true;
            for (const auto& descriptor : transport.user_transports) {
                if (!first_transport) out << ',';
                first_transport = false;
                auto udp = std::dynamic_pointer_cast<fastdds::rtps::UDPv4TransportDescriptor>(descriptor);
                auto shm = std::dynamic_pointer_cast<fastdds::rtps::SharedMemTransportDescriptor>(descriptor);
                if (udp) {
                    out << "{\"type\":\"UDPv4\",\"whitelist\":[";
                    bool first_address = true;
                    for (const auto& address : udp->interfaceWhiteList) {
                        if (!first_address) out << ',';
                        first_address = false;
                        out << std::quoted(address);
                    }
                    out << "]}";
                } else if (shm) {
                    out << "{\"type\":\"SHM\",\"segment_size\":" << shm->segment_size()
                        << ",\"queue_capacity\":" << shm->port_queue_capacity() << '}';
                } else {
                    out << "{\"type\":\"unknown\"}";
                }
            }
            out << "]}";
        }
        out << "]}";
        result = out.str();
    } catch (...) {
        result = "{\"error\":\"DDS inspection exception\"}";
    }
    return result.c_str();
}
