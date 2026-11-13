#pragma once
#include "steering_zero.hpp"
// Generated mounted zero; no UART, torque, ARM or firmware output enable.
namespace arena_steering_mount {
inline constexpr char kCanonicalProfileSha256[] = "88cc31a7cf21c9d54e470863b741533d0814e25eadc4efa6dc008914c3f094ae";
inline constexpr char kCadTableSha256[] = "882d0a66d987a15f66ef58303bcbd74c32d0c1afef255dc42f0a30ff1973b189";
inline arena_steering::ZeroProfile Profile() {
  arena_steering::ZeroProfile p;
  p.zero=2000; p.minimum=1880; p.maximum=2120;
  p.direction=1; p.counts_per_turn=4096;
  p.straight_verified=p.direction_verified=p.endpoints_verified=true;
  p.position_mode_verified=p.scale_verified=true;
  return p;
}
}  // namespace arena_steering_mount
