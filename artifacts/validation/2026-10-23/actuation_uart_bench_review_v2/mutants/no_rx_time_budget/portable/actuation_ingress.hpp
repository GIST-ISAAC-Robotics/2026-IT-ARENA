#pragma once
// 단일 소유자용 바이트 -> 요청 상태 연결부. 장치 드라이버/출력 허가/제어 task가 아니다.
// receiver와 이 객체의 모든 호출은 같은 작업(또는 같은 잠금)에서 직렬화해야 한다.
#include "actuation_link.hpp"
#include "actuation_receiver.hpp"

namespace arena_actuation {

enum class IngressFault : std::uint8_t { kNone, kInvalidConfig, kInvalidClock };
struct IngressDelivery {
  FrameOrigin origin;
  ParseStatus parse_status;
  CommandClass classification;
  bool receiver_called;
  ReceiveResult result;
  bool output_gate_rejected = false;  // 명령 객체는 처리했으나 ARM 준비 관문에서 receive 이전 거절
};
struct IngressResult {
  LinkResult link;
  IngressFault fault;
  bool stop_injected = false;
  ReceiveResult stop_result{false, ReceiveReason::kInvalidMessage};
};

class ActuationIngress {
 public:
  // 형식/조립 기한을 호출자가 명시한다. 운영 timeout 기본값은 두지 않는다.
  // receiver를 참조하므로 receiver의 수명은 이 객체보다 길어야 한다.
  ActuationIngress(ActuationReceiverCore& receiver, std::uint64_t assembly_timeout_us, PayloadLayout layout)
      : receiver_(receiver), link_(assembly_timeout_us), layout_(layout) {
    constexpr char stop[] = "{\"v\":1,\"kind\":\"STOP\"}";
    parser_.parse(reinterpret_cast<const std::uint8_t*>(stop), sizeof(stop) - 1,
                  PayloadLayout::kBareCommand, stop_);
    if (assembly_timeout_us == 0 || (layout != PayloadLayout::kBareCommand && layout != PayloadLayout::kEnvelope))
      fault_ = IngressFault::kInvalidConfig;
  }
  ActuationIngress(const ActuationIngress&) = delete;
  ActuationIngress& operator=(const ActuationIngress&) = delete;

  IngressResult connect(std::int64_t now) {
    if (!valid_time(now)) return rejected(now);
    return finish(link_.connect(static_cast<std::uint64_t>(now)), now);
  }
  IngressResult disconnect(std::int64_t now) {
    if (!valid_time(now)) return rejected(now);
    return finish(link_.disconnect(static_cast<std::uint64_t>(now)), now);
  }
  // 링크 고장만 해제한다. receiver의 ESTOP/FAULT·소유권·임대는 복구하지 않는다.
  IngressResult clear_link_fault(std::int64_t now) {
    if (!valid_time(now)) return rejected(now);
    return finish(link_.clear_fault(static_cast<std::uint64_t>(now)), now);
  }

  // observer(delivery, command)는 동기 진단 callback이다. 참조 보관·객체 재진입·receiver 변경 금지.
  // 객체 결과가 아닌 JSON은 receiver에 전달하지 않는다. 형식이 잘못된 명령 객체는 receiver가 판정한다.
  // expired-drain도 원래 임대/원천 만료를 적용하며 현재 수신 시각으로 수명을 새로 만들지 않는다.
  // output_ready는 독립 출력 감시가 이번 호출 시점에 판정한 값이다. 기본 false이며 객체에 저장하지 않는다.
  // true도 출력 허가가 아니고 ARM을 receive()에 넘길 조건일 뿐이다. 실행 중 출력 상실 감시를 대신하지 않는다.
  template<class Observer>
  IngressResult feed(std::int64_t now, const std::uint8_t* data, std::size_t size, Observer&& observer,
                     bool output_ready = false) {
    if (!valid_time(now)) return rejected(now);
    const auto result = link_.feed(static_cast<std::uint64_t>(now), data, size,
        [&](const std::uint8_t* payload, std::size_t n, FrameOrigin origin) {
          dispatch(payload, n, origin, now, observer, output_ready);
        });
    return finish(result, now);
  }
  template<class Observer>
  IngressResult poll(std::int64_t now, Observer&& observer, bool output_ready = false) {
    if (!valid_time(now)) return rejected(now);
    const auto result = link_.poll(static_cast<std::uint64_t>(now),
        [&](const std::uint8_t* payload, std::size_t n, FrameOrigin origin) {
          dispatch(payload, n, origin, now, observer, output_ready);
        });
    return finish(result, now);
  }

  const FrameLink& link() const { return link_; }
  IngressFault fault() const { return fault_; }

 private:
  bool valid_time(std::int64_t now) {
    // 음수를 unsigned로 바꾸거나 수신기의 유효 시각 범위를 넘기지 않는다.
    // 범위 오류/구성 오류는 객체 재생성 전까지 해제하지 않는다.
    if (fault_ == IngressFault::kNone && (now < 0 || now > kMaxNowUs)) fault_ = IngressFault::kInvalidClock;
    return fault_ == IngressFault::kNone;
  }
  void stop_if_needed(IngressResult& result, std::int64_t now) {
    if (receiver_.state() != ReceiverState::kEstop && receiver_.state() != ReceiverState::kFault) {
      result.stop_injected = true;
      result.stop_result = receiver_.receive(stop_, now);
    }
  }
  IngressResult rejected(std::int64_t now) {
    IngressResult result{LinkResult::kFaultLatched, fault_};
    stop_if_needed(result, now);
    return result;
  }
  IngressResult finish(LinkResult operation, std::int64_t now) {
    IngressResult result{operation, fault_};
    if (link_.session_breaks() != breaks_seen_) {
      breaks_seen_ = link_.session_breaks();
      stop_if_needed(result, now);
    }
    return result;
  }
  template<class Observer>
  void dispatch(const std::uint8_t* payload, std::size_t n, FrameOrigin origin,
                std::int64_t now, Observer& observer, bool output_ready) {
    const auto status = parser_.parse(payload, n, layout_, command_);
    const auto cls = classify(command_);
    const bool delivered = status == ParseStatus::kObject && cls != CommandClass::kEnvelopeIgnored;
    // VirtualMCU.on_tx의 raw kind == ARM 관문은 버전/세션/준비 검사보다 먼저다.
    // 단, 다른 receiver 이벤트보다 now가 뒤로 갔다면 기존 clock 검사부터 실행해 고장을 숨기지 않는다.
    const bool clock_regressed = receiver_.has_now() && now < receiver_.now();
    const bool blocked = delivered && command_.kind == KindName::kArm && !output_ready && !clock_regressed;
    const bool called = delivered && !blocked;
    const auto result = blocked ? ReceiveResult{false, ReceiveReason::kOutputNotReady}
                        : called ? receiver_.receive(command_, now)
                                  : ReceiveResult{false, ReceiveReason::kInvalidMessage};
    observer(IngressDelivery{origin, status, cls, called, result, blocked}, command_);
  }

  ActuationReceiverCore& receiver_;
  FrameLink link_;
  CommandParser parser_;
  Command command_;
  Command stop_;
  PayloadLayout layout_;
  std::uint64_t breaks_seen_ = 0;
  IngressFault fault_ = IngressFault::kNone;
};

}  // namespace arena_actuation
