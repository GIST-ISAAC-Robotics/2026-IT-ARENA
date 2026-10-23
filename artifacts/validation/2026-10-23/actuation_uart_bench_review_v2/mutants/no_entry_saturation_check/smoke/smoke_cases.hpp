#pragma once
// 동일 사례를 호스트와 ESP-IDF에서 빌드한다. 시각/엔코더는 모두 합성이며 실제 장치를 읽거나 쓰지 않는다.
#include "actuation_link.hpp"
#include "actuation_receiver.hpp"
#include <cstdio>
#include <cstring>
#include <limits>
#include <type_traits>
#include <utility>

namespace arena_smoke {
namespace aa = arena_actuation;
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559 &&
              std::numeric_limits<double>::digits == 53, "binary64 required");
template<class T, class = void> struct HasCounterSetter : std::false_type {};
template<class T> struct HasCounterSetter<T, std::void_t<decltype(std::declval<T&>().test_set_counters(0, 0))>>
    : std::true_type {};
static_assert(!HasCounterSetter<aa::ActuationReceiverCore>::value, "test injection API must be absent");

struct Report {
  unsigned checks = 0;
  unsigned failures = 0;
  std::uint64_t failure_mask = 0;
  void record(bool result) {
    if (!result) { ++failures; failure_mask |= UINT64_C(1) << checks; }
    ++checks;
  }
};

inline aa::ReceiverConfig motor_config() {
  aa::ReceiverConfig c;
  c.feedback_mode = aa::FeedbackMode::kMotorShaft;
  c.ticks_per_rev = 28;
  c.gear_ratio = 15.;
  c.wheel_radius_m = .0325;
  return c;
}

inline std::uint64_t bits(double number) {
  std::uint64_t result;
  std::memcpy(&result, &number, sizeof(result));
  return result;
}

// 대형 parser/명령 작업 공간은 명시적으로 호출자가 소유한다(실제 task stack 보장 아님).
struct Scratch {
  aa::CommandParser parser;
  aa::Command command;
  std::array<std::uint8_t, aa::kMaxFrame> frame{};
  char text[256]{};
};

inline bool parse(Scratch& scratch, const char* json) {
  return scratch.parser.parse(reinterpret_cast<const std::uint8_t*>(json), std::strlen(json),
                              aa::PayloadLayout::kBareCommand, scratch.command) == aa::ParseStatus::kObject;
}

inline bool command(Scratch& scratch, const char* kind, const aa::ActuationReceiverCore& receiver,
                    const aa::Lease& lease, unsigned seq, bool drive = false) {
  const int size = std::snprintf(scratch.text, sizeof(scratch.text),
      "{\"v\":1,\"kind\":\"%s\",\"boot\":\"smoke-only\",\"generation\":%lld,\"lease\":%lld,"
      "\"client\":\"smoke-only\",\"seq\":%u%s}", kind,
      static_cast<long long>(receiver.generation()), static_cast<long long>(lease.id), seq,
      drive ? ",\"speed_mps\":0.5,\"steering_rad\":0.2,\"source_age_us\":0,\"ttl_us\":100000" : "");
  return size > 0 && static_cast<std::size_t>(size) < sizeof(scratch.text) && parse(scratch, scratch.text);
}

inline void stationary_samples(aa::ActuationReceiverCore& receiver, std::int64_t begin, std::int64_t end) {
  for (std::int64_t now = begin; now <= end; now += 10000) {
    receiver.update_motor_encoder(now, aa::IntInput::of(now / 10000), aa::IntInput::of(now), aa::IntInput::of(0));
    receiver.tick(now);
  }
}

inline Report run(Scratch& scratch) {
  Report report;
  // 고정 boot ID는 이 합성 검사에만 쓴다. 실물 제어 세션에 복사하면 안 된다.
  const std::uint32_t boot[] = {'s','m','o','k','e','-','o','n','l','y'};
  const auto cfg = motor_config();
  aa::ActuationReceiverCore receiver(cfg, boot, 10);
  report.record(cfg.valid() && receiver.motor_mode() && receiver.state() == aa::ReceiverState::kDisarmed &&
                receiver.target_speed() == 0.);
  stationary_samples(receiver, 0, 220000);
  auto offer = receiver.offer(220000);
  report.record(offer.ok && receiver.fresh_feedback(220000));
  report.record(command(scratch, "ARM", receiver, offer.lease, 1) &&
                receiver.receive(scratch.command, 220000).accepted && receiver.state() == aa::ReceiverState::kArmed);
  offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "DRIVE", receiver, offer.lease, 2, true) &&
                receiver.receive(scratch.command, 220000).accepted && receiver.target_speed() == .5 &&
                receiver.steering() == .2 && receiver.state() == aa::ReceiverState::kActive);

  const char stop[] = "{\"v\":1,\"kind\":\"STOP\"}";
  const auto n = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(stop), sizeof(stop) - 1,
                                  scratch.frame.data(), scratch.frame.size());
  aa::FrameLink link(30000);  // 합성 조립 기한, 실물 기본값 아님.
  unsigned delivered = 0;
  auto receive_frame = [&](const std::uint8_t* data, std::size_t size, aa::FrameOrigin origin) {
    if (origin == aa::FrameOrigin::kLive && scratch.parser.parse(data, size, aa::PayloadLayout::kBareCommand,
        scratch.command) == aa::ParseStatus::kObject && receiver.receive(scratch.command, 220000).accepted) ++delivered;
  };
  report.record(n > 3 && link.connect(220000) == aa::LinkResult::kOk &&
                link.feed(220000, scratch.frame.data(), 3, receive_frame) == aa::LinkResult::kOk && delivered == 0);
  report.record(link.feed(220000, scratch.frame.data() + 3, n - 3, receive_frame) == aa::LinkResult::kOk &&
                delivered == 1 && receiver.state() == aa::ReceiverState::kEstop &&
                receiver.target_speed() == 0. && receiver.steering() == .2);
  receiver.set_estop(false, 220000);
  report.record(receiver.state() == aa::ReceiverState::kEstop && receiver.target_speed() == 0.);
  offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "CLEAR", receiver, offer.lease, 1) &&
                receiver.receive(scratch.command, 220000).accepted && receiver.state() == aa::ReceiverState::kDisarmed);
  offer = receiver.offer(220000);
  report.record(offer.ok && command(scratch, "ARM", receiver, offer.lease, 1) &&
                receiver.receive(scratch.command, 220000).accepted);
  receiver.tick(280000);
  report.record(receiver.state() == aa::ReceiverState::kFault && receiver.target_speed() == 0. &&
                std::strcmp(receiver.reason(), "control_deadline_missed") == 0);

  report.record(!parse(scratch, "{\"v\":1,\"v\":1}") && scratch.command.status == aa::ParseStatus::kDuplicateKey);
  report.record(!parse(scratch, "{\"v\":NaN}") && scratch.command.status == aa::ParseStatus::kNonFiniteConstant);
  const struct { const char* value; std::uint64_t expected; } numbers[] = {
    {"0.1", UINT64_C(0x3fb999999999999a)}, {"-0.0", UINT64_C(0x8000000000000000)},
    {"5e-324", UINT64_C(1)}, {"2.5000000000000002", UINT64_C(0x4004000000000000)},
    {"1e400", UINT64_C(0x7ff0000000000000)}, {"1e-400", UINT64_C(0)},
  };
  for (const auto& number : numbers) {
    const int size = std::snprintf(scratch.text, sizeof(scratch.text), "{\"speed_mps\":%s}", number.value);
    report.record(size > 0 && static_cast<std::size_t>(size) < sizeof(scratch.text) && parse(scratch, scratch.text) &&
                  scratch.command.field(aa::CommandField::kSpeed).type == aa::ValueType::kFloat &&
                  bits(scratch.command.field(aa::CommandField::kSpeed).real) == number.expected);
  }
  report.record(parse(scratch, "{\"seq\":9223372036854775807}") &&
                scratch.command.field(aa::CommandField::kSeq).type == aa::ValueType::kInt &&
                scratch.command.field(aa::CommandField::kSeq).integer == INT64_MAX);
  report.record(!receiver.fresh_feedback(INT64_MIN));
  return report;
}
}  // namespace arena_smoke
