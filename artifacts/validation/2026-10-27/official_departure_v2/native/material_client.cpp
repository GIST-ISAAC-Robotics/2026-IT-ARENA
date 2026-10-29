// Evaluation-only persistent Gazebo service client. One request per input, no retry.
#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <google/protobuf/text_format.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/visual.pb.h>
#include <gz/transport/Node.hh>

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
  if (!server.Advertise("/material_trial/check", cb)) return 2;
  if (!request(client,"/material_trial/check","name: \"accept\"",3000,std::cout)) return 3;
  if (request(client,"/material_trial/check","name: \"reject\"",3000,std::cout)) return 4;
  if (request(client,"/material_trial/check","name: \"service_fail\"",3000,std::cout)) return 5;
  if (request(client,"/material_trial/missing","name: \"accept\"",100,std::cout)) return 6;
  if (request(client,"/material_trial/check","not_a_proto",100,std::cout)) return 7;
  return 0;
}

int main(int argc, char **argv) {
  if (argc == 2 && std::string(argv[1]) == "--self-test") return self_test();
  if (argc != 2 || std::string(argv[1]).find("/world/") != 0) return 2;
  gz::transport::Node client;
  std::string line;
  while (std::getline(std::cin, line)) {
    // An uncertain result ends the client rather than retrying a possibly applied request.
    if (!request(client,argv[1],line,3000,std::cout)) return 1;
  }
  return 0;
}
