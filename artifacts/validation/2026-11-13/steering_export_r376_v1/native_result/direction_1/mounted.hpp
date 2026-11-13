#pragma once
#include "steering_zero.hpp"
// Generated mounted zero; no UART, torque, ARM or firmware output enable.
namespace arena_steering_mount {
inline constexpr char kCanonicalProfileSha256[] = "61cba234a7434b92e1c2dbb559e1383f4a803a24e1eabd89d22482973186e066";
inline constexpr char kCadTableSha256[] = "882d0a66d987a15f66ef58303bcbd74c32d0c1afef255dc42f0a30ff1973b189";
inline arena_steering::ZeroProfile Profile() {
  arena_steering::ZeroProfile p;
  p.zero=2137; p.minimum=1600; p.maximum=2700;
  p.direction=1; p.counts_per_turn=4096;
  p.straight_verified=p.direction_verified=p.endpoints_verified=true;
  p.position_mode_verified=p.scale_verified=true;
  return p;
}
}  // namespace arena_steering_mount
