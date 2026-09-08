#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <chrono>
#include <iostream>
int main(int argc, char **argv) {
  if (argc != 2) return 2;
  gz::sim::ServerConfig config;
  config.SetSdfFile(argv[1]);
  gz::sim::Server server(config);
  if (!server.EntityCount().has_value()) return 5;
  server.SetUpdatePeriod(std::chrono::nanoseconds(0));
  if (!server.Run(true, 100, false)) return 3;
  auto start = std::chrono::steady_clock::now();
  if (!server.Run(true, 1000, false)) return 4;
  if (server.IterationCount().value_or(0) != 1100) return 6;
  double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  std::cout << "PROFILE {\"iterations\":1000,\"step_s\":0.001,\"wall_s\":" << elapsed
            << ",\"rtf\":" << 1.0 / elapsed << "}" << std::endl;
}
