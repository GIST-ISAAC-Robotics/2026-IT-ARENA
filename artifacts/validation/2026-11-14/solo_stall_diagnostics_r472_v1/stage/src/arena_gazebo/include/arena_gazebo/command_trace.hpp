#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace arena
{
// 평가 전용 명령 수신/적용 전환 기록. 제어 계산·보호 판정에는 쓰지 않습니다.
// ChannelReceiveTrace::Received/Take는 플러그인의 기존 mutex 안에서만, CommandApplyTrace::Step은
// PreUpdate 스레드에서만 호출합니다. 꺼져 있으면 플러그인은 이 객체를 호출하지 않습니다.
struct ReceiveMark
{
  uint64_t sequence{};        // 채널 누적 수신 번호(1부터). 값이 같은 메시지도 셉니다.
  int64_t wallNs{-1};         // 수신 콜백의 steady_clock(ns, CLOCK_MONOTONIC)
  uint64_t afterIteration{};  // 수신 직전에 잠금 구역을 지난 마지막 PreUpdate iteration(0: 아직 없음)
  int64_t afterSimNs{-1};     // 그 PreUpdate의 UpdateInfo::simTime(ns)
};

struct ReceiveSnapshot
{
  ReceiveMark first;               // 현재 수신값을 처음 실어 온 메시지
  uint64_t latestSequence{};       // 이 PreUpdate까지 받은 마지막 메시지 번호
  uint64_t superseded{};           // 어떤 PreUpdate에도 보이기 전에 다른 값으로 덮인 수신 전환 누적
  uint64_t receivedTransitions{};  // 수신 스트림의 값 전환 누적
};

class ChannelReceiveTrace
{
public:
  void Received(double value, int64_t wallNs, uint64_t lastIteration, int64_t lastSimNs)
  {
    ++sequence;
    if (!hasValue || value != lastValue)
    {
      if (transitionPending) ++superseded;
      first = {sequence, wallNs, lastIteration, lastSimNs};
      transitionPending = true;
      ++receivedTransitions;
    }
    lastValue = value;
    hasValue = true;
  }

  // PreUpdate 잠금 안에서 적용값을 읽은 직후 호출합니다. 미소비 수신 전환 표시만 지웁니다.
  ReceiveSnapshot Take()
  {
    transitionPending = false;
    return {first, sequence, superseded, receivedTransitions};
  }

private:
  uint64_t sequence{}, superseded{}, receivedTransitions{};
  ReceiveMark first;
  double lastValue{};
  bool hasValue{}, transitionPending{};
};

struct TraceStep
{
  uint64_t iteration{};  // UpdateInfo::iterations
  int64_t simNs{};       // UpdateInfo::simTime (PreUpdate에 전달된 값)
  int64_t dtNs{};
  int64_t wallNs{};      // PreUpdate 진입 후 steady_clock
};

class CommandApplyTrace
{
public:
  // 적용값(감시 만료 반영 후 motor/조향 계산에 실제로 넘긴 값)이 직전 step과 다를 때만 이벤트를 만듭니다.
  void Step(const TraceStep &step, double speed, double steering, bool stale, const ReceiveSnapshot &speedRx,
            const ReceiveSnapshot &steeringRx, std::vector<std::string> &out)
  {
    Count(speedChannel, speedRx);
    Count(steeringChannel, steeringRx);
    if (!started)
    {
      started = true;
      std::ostringstream json;
      Begin(json, "first_update", step);
      json << ",\"speed\":" << speed << ",\"steering\":" << steering << ",\"stale\":" << Bool(stale) << "}";
      out.push_back(json.str());
      nextSummaryNs = (step.simNs / kSummaryPeriodNs + 1) * kSummaryPeriodNs;
    }
    else
    {
      Apply(out, step, "speed", lastSpeed, speed, stale, speedRx, speedChannel);
      Apply(out, step, "steering", lastSteering, steering, stale, steeringRx, steeringChannel);
    }
    lastSpeed = speed;
    lastSteering = steering;
    lastStale = stale;
    if (step.simNs >= nextSummaryNs)
    {
      std::ostringstream json;
      Begin(json, "summary", step);
      Counters(json, "speed", speedChannel, speedRx);
      Counters(json, "steering", steeringChannel, steeringRx);
      json << "}";
      out.push_back(json.str());
      nextSummaryNs = (step.simNs / kSummaryPeriodNs + 1) * kSummaryPeriodNs;
    }
  }

  void Reset(const TraceStep &step, std::vector<std::string> &out)
  {
    std::ostringstream json;
    Begin(json, "reset", step);
    json << "}";
    out.push_back(json.str());
    started = false;
  }

  static constexpr int64_t kSummaryPeriodNs = 1000000000;

private:
  struct Channel
  {
    uint64_t previousLatest{}, stepsWithNewMessage{}, multiMessageSteps{}, applyEvents{};
  };

  static const char *Bool(bool value) { return value ? "true" : "false"; }

  static void Count(Channel &channel, const ReceiveSnapshot &rx)
  {
    const uint64_t added = rx.latestSequence - channel.previousLatest;
    channel.stepsWithNewMessage += added > 0 ? 1 : 0;
    channel.multiMessageSteps += added > 1 ? 1 : 0;
    channel.previousLatest = rx.latestSequence;
  }

  void Begin(std::ostringstream &json, const char *type, const TraceStep &step)
  {
    json << std::setprecision(17) << "{\"type\":\"" << type << "\",\"event_seq\":" << ++eventSequence
         << ",\"iteration\":" << step.iteration << ",\"sim_ns\":" << step.simNs << ",\"dt_ns\":" << step.dtNs
         << ",\"wall_ns\":" << step.wallNs;
  }

  static void Counters(std::ostringstream &json, const char *name, const Channel &channel, const ReceiveSnapshot &rx)
  {
    json << ",\"" << name << "\":{\"rx_messages\":" << rx.latestSequence
         << ",\"rx_transitions\":" << rx.receivedTransitions << ",\"rx_superseded\":" << rx.superseded
         << ",\"steps_with_new_message\":" << channel.stepsWithNewMessage
         << ",\"multi_message_steps\":" << channel.multiMessageSteps
         << ",\"apply_events\":" << channel.applyEvents << "}";
  }

  void Apply(std::vector<std::string> &out, const TraceStep &step, const char *name, double before, double after,
             bool stale, const ReceiveSnapshot &rx, Channel &channel)
  {
    if (after == before) return;
    ++channel.applyEvents;
    const bool watchdog = stale != lastStale;
    std::ostringstream json;
    Begin(json, "apply", step);
    json << ",\"channel\":\"" << name << "\",\"from\":" << before << ",\"to\":" << after
         << ",\"cause\":\"" << (watchdog ? "watchdog" : "command") << "\",\"stale\":" << Bool(stale)
         << ",\"rx_seq_first\":" << rx.first.sequence << ",\"rx_seq_latest\":" << rx.latestSequence
         << ",\"rx_wall_ns\":" << rx.first.wallNs << ",\"rx_after_iteration\":" << rx.first.afterIteration
         << ",\"rx_after_sim_ns\":" << rx.first.afterSimNs
         << ",\"steps_after_rx\":" << static_cast<int64_t>(step.iteration - rx.first.afterIteration)
         << ",\"rx_to_apply_wall_ns\":" << (rx.first.wallNs < 0 ? -1 : step.wallNs - rx.first.wallNs)
         << ",\"rx_superseded_total\":" << rx.superseded << "}";
    out.push_back(json.str());
  }

  bool started{}, lastStale{true};
  double lastSpeed{}, lastSteering{};
  uint64_t eventSequence{};
  int64_t nextSummaryNs{};
  Channel speedChannel, steeringChannel;
};
}  // namespace arena
