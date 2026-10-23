// MCU 대상 컴파일/심볼 검사 전용. 장치 I/O와 펌웨어 entry point가 없으며 flash 대상이 아니다.
// 외부 인수를 받는 함수로 parser·link·receiver 코드를 실제로 인스턴스화한다.
#include "actuation_link.hpp"
#include "actuation_receiver.hpp"
#include <limits>
#include <type_traits>
#include <utility>

namespace aa = arena_actuation;
static_assert(sizeof(std::int64_t) == 8, "64-bit timestamps and encoder counts required");
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559 &&
              std::numeric_limits<double>::digits == 53, "binary64 required");
template<class T, class = void> struct HasCounterSetter : std::false_type {};
template<class T> struct HasCounterSetter<T, std::void_t<decltype(std::declval<T&>().test_set_counters(0, 0))>>
    : std::true_type {};
static_assert(!HasCounterSetter<aa::ActuationReceiverCore>::value, "test API exposed in target build");

extern "C" {
// nm -S로 대상 ABI의 크기를 읽는다. 전부 미초기화 BSS이며 작동 객체/메모리 예산이 아니다.
unsigned char arena_size_frame_decoder[sizeof(aa::FrameDecoder)];
unsigned char arena_size_frame_link[sizeof(aa::FrameLink)];
unsigned char arena_size_command_parser[sizeof(aa::CommandParser)];
unsigned char arena_size_command[sizeof(aa::Command)];
unsigned char arena_size_receiver[sizeof(aa::ActuationReceiverCore)];
unsigned char arena_size_receiver_config[sizeof(aa::ReceiverConfig)];

aa::ParseStatus arena_probe_parse(aa::CommandParser* parser, const std::uint8_t* data, std::size_t size,
                                aa::PayloadLayout layout, aa::Command* command) {
  return parser->parse(data, size, layout, *command);
}

aa::ReceiveResult arena_probe_receive(aa::ActuationReceiverCore* receiver, const aa::Command* command,
                                     std::int64_t now) {
  return receiver->receive(*command, now);
}

aa::OfferResult arena_probe_tick_offer(aa::ActuationReceiverCore* receiver, std::int64_t now) {
  receiver->tick(now);
  return receiver->offer(now);
}

bool arena_probe_encoder(aa::ActuationReceiverCore* receiver, std::int64_t now, std::int64_t seq,
                         std::int64_t capture, std::int64_t count) {
  return receiver->update_motor_encoder(now, aa::IntInput::of(seq), aa::IntInput::of(capture), aa::IntInput::of(count));
}

void arena_probe_estop(aa::ActuationReceiverCore* receiver, bool held, std::int64_t now) {
  receiver->set_estop(held, now);
}

aa::LinkResult arena_probe_link(aa::FrameLink* link, std::uint64_t now, const std::uint8_t* data,
                               std::size_t size, std::uint32_t* crc_sink) {
  return link->feed(now, data, size, [crc_sink](const std::uint8_t* payload, std::size_t n, aa::FrameOrigin origin) {
    *crc_sink = aa::crc32(payload, n) ^ static_cast<unsigned>(origin);
  });
}

aa::ReceiverState arena_probe_construct(const std::uint32_t* boot, std::size_t length) {
  aa::ReceiverConfig cfg;
  cfg.feedback_mode = aa::FeedbackMode::kMotorShaft;
  cfg.ticks_per_rev = 28;
  cfg.gear_ratio = 15.;
  cfg.wheel_radius_m = .0325;
  aa::ActuationReceiverCore receiver(cfg, boot, length);
  return receiver.state();
}
}
