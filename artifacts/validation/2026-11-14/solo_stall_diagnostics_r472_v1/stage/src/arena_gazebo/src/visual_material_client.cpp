// 시뮬레이터 신호 광원용 지속 Gazebo 서비스 클라이언트. 입력 한 줄에 요청 한 번, 재시도 없음.
// 광원별 `gz service` 프로세스의 응답 누락(10/27 3대 시험)을 피하려고 Node 하나를 유지합니다.
#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <google/protobuf/text_format.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/transport/Node.hh>

namespace {
constexpr unsigned int kTimeoutMs = 3000;

bool request(gz::transport::Node &node, const std::string &service,
             const std::string &text, unsigned int timeout, std::ostream &out) {
  gz::msgs::Visual input;
  const bool parsed = google::protobuf::TextFormat::ParseFromString(text, &input);
  gz::msgs::Boolean reply;
  bool result = false;
  const auto began = std::chrono::steady_clock::now();
  const bool executed = parsed && node.Request(service, input, timeout, reply, result);
  const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-began).count();
  out << "{\"parsed\":" << (parsed ? "true" : "false")
      << ",\"executed\":" << (executed ? "true" : "false")
      << ",\"service_result\":" << (result ? "true" : "false")
      << ",\"applied\":" << (reply.data() ? "true" : "false")
      << ",\"elapsed_wall_s\":" << elapsed << "}" << std::endl;
  return parsed && executed && result && reply.data();
}

int self_test() {
  gz::transport::Node server, client;
  std::function<bool(const gz::msgs::Visual &, gz::msgs::Boolean &)> cb =
    [](const gz::msgs::Visual &in, gz::msgs::Boolean &out) {
      out.set_data(in.name() == "accept");
      return in.name() != "service_fail";
    };
  if (!server.Advertise("/visual_material/check", cb)) return 2;
  if (!request(client, "/visual_material/check", "name: \"accept\"", kTimeoutMs, std::cout)) return 3;
  if (request(client, "/visual_material/check", "name: \"reject\"", kTimeoutMs, std::cout)) return 4;
  if (request(client, "/visual_material/check", "name: \"service_fail\"", kTimeoutMs, std::cout)) return 5;
  if (request(client, "/visual_material/missing", "name: \"accept\"", 100, std::cout)) return 6;
  if (request(client, "/visual_material/check", "not_a_proto", 100, std::cout)) return 7;
  return 0;
}
}  // namespace

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") return self_test();
  if (argc != 2 || std::string(argv[1]).find("/world/") != 0) return 2;
  gz::transport::Node client;
  std::string line;
  while (std::getline(std::cin, line)) {
    // 불명확한 결과는 이미 적용됐을 수 있으므로 재시도하지 않고 종료합니다.
    if (!request(client, argv[1], line, kTimeoutMs, std::cout)) return 1;
  }
  return 0;
}
