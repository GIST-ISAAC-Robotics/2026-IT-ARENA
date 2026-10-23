#pragma once
// 출력 비활성 UART 주기 어댑터 검사용 가짜 I/O·합성 시각. 호스트와 ESP-IDF 공용이며 실제 장치를 열지 않는다.
// 결함 재현기(scripts/uart_bench_defect_repro.cpp)가 이전 어댑터 헤더로도 컴파일할 수 있게 공통 API만 쓴다.
#include "bench_cases.hpp"
#include "actuation_uart_bench.hpp"
#include <new>

namespace arena_smoke {

struct FakeClock {
  std::int64_t now = 0;
  std::int64_t step = 0;            // 각 읽기 뒤 증가(호출 사이 경과 시간 모사)
  long regress_on_call = -1;        // 이 번째 호출에서 직전보다 1 µs 작은 값을 돌려준다
  bool regress_next = false;        // 다음 한 번의 호출에서 직전보다 1 µs 작은 값을 돌려준다
  long calls = 0;
  std::int64_t now_us() {
    ++calls;
    if (calls == regress_on_call) return now - 1;
    if (regress_next) { regress_next = false; return now - 1; }
    const auto value = now;
    now += step;
    return value;
  }
};
struct FakeIo {
  std::array<std::uint8_t, 8192> rx{};
  std::size_t rx_begin = 0, rx_end = 0;
  bool endless_rx = false, read_fail = false, read_over = false;
  std::array<aa::UartIoEvent, 48> queue{};
  std::size_t queued = 0;
  bool endless_events = false;
  long inject_after_read = -1;      // 이 번째 read가 데이터를 돌려준 직후 사건이 보이게 한다
  aa::UartIoEvent inject_event = aa::UartIoEvent::kRxFifoOverflow;
  std::size_t data_events_per_read = 0;  // 데이터를 돌려준 read마다 DATA 사건을 이만큼 추가(드라이버 모사)
  long refill_on_event_call = -1;   // 이 번째 next_event 뒤 큐를 refill_to까지 DATA로 다시 채운다(꺼내는 중 포화)
  std::size_t refill_to = 0;
  // 데이터를 돌려준 다음 read 뒤 처음 '큐 비어 있음'으로 답한 직후 이 사건을 넣는다(빈 응답과 마지막 조회 사이 도착)
  bool enqueue_after_empty_on_next_read = false;
  bool armed_after_empty = false;
  aa::UartIoEvent after_empty_event = aa::UartIoEvent::kFrameError;
  FakeClock* clock = nullptr;       // 쓰기/폐기에 걸린 시간 모사
  std::int64_t write_cost_us = 0, discard_cost_us = 0;
  bool regress_after_write = false; // 쓰기 직후 다음 시각 읽기를 역행시킨다
  std::array<std::uint8_t, 16384> wire{};
  std::size_t wire_used = 0;
  std::size_t write_limit = static_cast<std::size_t>(-1);
  bool write_fail = false, write_over = false, discard_fail = false;
  unsigned long reads = 0, writes = 0, event_calls = 0, pending_calls = 0, discards = 0;

  unsigned long calls() const { return reads + writes + event_calls + pending_calls + discards; }
  bool push_rx(const std::uint8_t* data, std::size_t size) {
    if (size > rx.size() - rx_end) return false;
    std::memcpy(rx.data() + rx_end, data, size);
    rx_end += size;
    return true;
  }
  void push_event(aa::UartIoEvent event) {
    if (queued < queue.size()) queue[queued++] = event;
  }
  int read(std::uint8_t* dst, std::size_t max) {
    ++reads;
    if (read_fail) return -1;
    if (read_over) return static_cast<int>(max) + 1;
    std::size_t n = max;
    if (endless_rx) {
      std::memset(dst, 0x55, n);
    } else {
      if (n > rx_end - rx_begin) n = rx_end - rx_begin;
      std::memcpy(dst, rx.data() + rx_begin, n);
      rx_begin += n;
    }
    if (n != 0) for (std::size_t i = 0; i < data_events_per_read; ++i) push_event(aa::UartIoEvent::kData);
    if (n != 0 && static_cast<long>(reads) == inject_after_read) push_event(inject_event);
    if (n != 0 && enqueue_after_empty_on_next_read) { enqueue_after_empty_on_next_read = false; armed_after_empty = true; }
    return static_cast<int>(n);
  }
  int write(const std::uint8_t* src, std::size_t size) {
    ++writes;
    if (clock) clock->now += write_cost_us;
    if (clock && regress_after_write) clock->regress_next = true;
    if (write_fail) return -1;
    if (write_over) return static_cast<int>(size) + 1;
    std::size_t n = size < write_limit ? size : write_limit;
    if (n > wire.size() - wire_used) n = wire.size() - wire_used;
    std::memcpy(wire.data() + wire_used, src, n);
    wire_used += n;
    return static_cast<int>(n);
  }
  bool next_event(aa::UartIoEvent& out) {
    ++event_calls;
    if (endless_events) { out = aa::UartIoEvent::kData; return true; }
    if (queued == 0) {
      if (armed_after_empty) { armed_after_empty = false; push_event(after_empty_event); }
      return false;
    }
    out = queue[0];
    for (std::size_t i = 1; i < queued; ++i) queue[i-1] = queue[i];
    --queued;
    if (static_cast<long>(event_calls) == refill_on_event_call)
      while (queued < refill_to) push_event(aa::UartIoEvent::kData);
    return true;
  }
  std::size_t pending_events() {
    ++pending_calls;
    return endless_events ? 1000 : queued;
  }
  bool discard_input() {
    ++discards;
    if (clock) clock->now += discard_cost_us;
    queued = 0;
    rx_begin = rx_end;
    return !discard_fail;
  }
};

using UartAdapter = aa::UartBenchAdapter<FakeIo, FakeClock>;

// 이후 판본에서 추가된 구성 필드는 있을 때만 채운다(이전 판본 재현용).
template<class C> auto set_reserve(C& c, int value, int) -> decltype(c.post_read_event_reserve = 0, void()) {
  c.post_read_event_reserve = static_cast<std::uint8_t>(value);
}
template<class C> void set_reserve(C&, int, long) {}

inline aa::UartBenchConfig uart_config() {
  aa::UartBenchConfig c;                 // 합성 시험값. 실물 운영값 아님
  c.control_period_us = 5000;
  c.cycle_budget_us = 2000;
  c.max_rx_bytes_per_cycle = 1024;
  c.max_events_per_cycle = 8;
  set_reserve(c, 3, 0);
  c.event_queue_capacity = 16;
  return c;
}
// 큰 객체는 정적 저장 공간에서 사례마다 새로 만든다(호스트/대상 main이 소유).
struct UartScratch {
  alignas(aa::ActuationBenchEndpoint) unsigned char endpoint_storage[sizeof(aa::ActuationBenchEndpoint)];
  alignas(UartAdapter) unsigned char adapter_storage[sizeof(UartAdapter)];
  aa::ActuationBenchEndpoint* endpoint = nullptr;
  UartAdapter* adapter = nullptr;
  FakeIo io;
  FakeClock clock;
  aa::FrameDecoder decoder;
  unsigned wire_frames = 0;

  UartAdapter& make(bool io_enabled, const aa::UartBenchConfig& config = uart_config()) {
    destroy();
    // 24 KiB 가짜 I/O를 임시 객체로 만들어 대입하면 대상 stack에 그만큼 잡힐 수 있다. 제자리에서 다시 만든다.
    io.~FakeIo();
    new (&io) FakeIo();
    clock.~FakeClock();
    new (&clock) FakeClock();
    endpoint = new (endpoint_storage) aa::ActuationBenchEndpoint(motor_config(), kSmokeBoot, 10, 30000,
                                                                 aa::PayloadLayout::kEnvelope, tx_config());
    io.clock = &clock;
    adapter = new (adapter_storage) UartAdapter(*endpoint, config, io, clock, io_enabled);
    return *adapter;
  }
  void destroy() {
    if (adapter) adapter->~UartAdapter();
    if (endpoint) endpoint->~ActuationBenchEndpoint();
    adapter = nullptr;
    endpoint = nullptr;
  }
  ~UartScratch() { destroy(); }
};

inline bool push_frame(Scratch& scratch, FakeIo& io, const char* payload, std::size_t keep = 0) {
  const auto size = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(payload), std::strlen(payload),
                                     scratch.frame.data(), scratch.frame.size());
  if (size == 0) return false;
  return io.push_rx(scratch.frame.data(), keep == 0 || keep > size ? size : keep);
}
inline bool push_rest(Scratch& scratch, FakeIo& io, const char* payload, std::size_t from) {
  const auto size = aa::encode_frame(reinterpret_cast<const std::uint8_t*>(payload), std::strlen(payload),
                                     scratch.frame.data(), scratch.frame.size());
  return size > from && io.push_rx(scratch.frame.data() + from, size - from);
}
// 호스트 쪽 복호: CRC 통과 프레임 중 token을 담은 것이 있는가. 전체 wire를 처음부터 다시 읽는다.
inline bool wire_contains(UartScratch& f, const char* token) {
  f.decoder.reset();
  f.wire_frames = 0;
  bool seen = false;
  for (std::size_t i = 0; i < f.io.wire_used; i += aa::kMaxInputPerCall) {
    const auto n = f.io.wire_used - i < aa::kMaxInputPerCall ? f.io.wire_used - i : aa::kMaxInputPerCall;
    f.decoder.feed(f.io.wire.data() + i, n, [&](const std::uint8_t* p, std::size_t size) {
      ++f.wire_frames;
      seen = contains(p, size, token) || seen;
    });
  }
  return seen;
}
inline void control_text(Scratch& scratch, const char* request, const char* kind, std::int64_t generation,
                         std::int64_t lease, int seq) {
  std::snprintf(scratch.text, sizeof(scratch.text),
      "{\"type\":\"command\",\"request\":\"%s\",\"command\":{\"v\":1,\"kind\":\"%s\",\"boot\":\"smoke-only\","
      "\"generation\":%lld,\"lease\":%lld,\"client\":\"smoke-only\",\"seq\":%d}}",
      request, kind, static_cast<long long>(generation), static_cast<long long>(lease), seq);
}
inline aa::UartCycleReport cycle_at(UartScratch& f, std::int64_t now) {
  f.clock.now = now;
  return f.adapter->run_cycle();
}

}  // namespace arena_smoke
