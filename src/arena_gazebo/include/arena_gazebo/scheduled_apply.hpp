#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <iomanip>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace arena {
// 평가 전용: 보호된 출력만 미래 step에 예약. 실물 PWM/제동 모델이 아니다.
struct ScheduledPacket {
  int64_t seq{}, stamp{}, at{}, deadline{};
  std::string kind, epoch;
  double speed{}, steering{};
  int64_t rxWall{}, rxSim{};
  uint64_t rxIteration{};
};

inline bool ParseScheduled(const std::string &text, ScheduledPacket &p) {
  if (text.size()>512) return false;
  std::istringstream in(text); std::string version, extra;
  if (!(in>>version>>p.seq>>p.kind>>p.epoch>>p.stamp>>p.at>>p.deadline>>p.speed>>p.steering)
      || (in>>extra) || version!="A1") return false;
  return p.seq>0 && p.seq<1000000000 && p.stamp>=0 && p.at>=0 && p.deadline>=0
      && p.stamp<1000000000000LL && p.at<1000000000000LL && p.deadline<1000000000000LL
      && (p.kind=="arm" || p.kind=="set" || p.kind=="stop")
      && !p.epoch.empty() && p.epoch.size()<=90
      && p.epoch.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-:.")==std::string::npos
      && std::isfinite(p.speed) && p.speed>=0 && p.speed<=2.5
      && std::isfinite(p.steering) && std::abs(p.steering)<=.45
      && (p.kind=="set" || (p.speed==0 && p.steering==0));
}

class ScheduledApply {
public:
  // 호출자는 기존 플러그인 mutex를 보유한다. 이벤트는 PostUpdate에서 발행한다.
  void Receive(const std::string &text, int64_t now, int64_t wall, uint64_t iteration) {
    ScheduledPacket p;
    if (!ParseScheduled(text,p)) {Fault("malformed",now);return;}
    p.rxWall=wall; p.rxSim=now*1000; p.rxIteration=iteration;
    if (p.kind=="stop") {
      Event("received",p,now); Cancel(now); active=false; speed=steering=0;
      lastSeq=std::max(lastSeq,p.seq); Event("stopped",p,now); return;
    }
    if (failed) {Event("rejected",p,now,"latched_fault");return;}
    if (p.seq!=lastSeq+1) {Fault("sequence",now);return;}
    lastSeq=p.seq;
    if (p.stamp>now || now-p.stamp>=100000 || p.deadline<=now || p.deadline-p.stamp>100000) {
      Fault("stamp_or_expiry",now);return;
    }
    if (p.kind=="arm") {
      if (active || seen.count(p.epoch) || seen.size()>=64) {Fault("arm_epoch",now);return;}
      Cancel(now); seen.insert(p.epoch); epoch=p.epoch; active=true; speed=steering=0;
      lastStamp=p.stamp; appliedDeadline=p.deadline; lastWall=wall; lastAt=-1;
      Event("received",p,now); Event("armed",p,now); return;
    }
    if (!active || p.epoch!=epoch || p.stamp<=lastStamp || p.at!=p.stamp+20000
        || p.at%1000 || p.at<=now || p.at>=p.deadline || p.at<=lastAt || queue.size()>=64) {
      Fault("invalid_or_late_set",now);return;
    }
    lastStamp=p.stamp; lastAt=p.at; lastWall=wall;
    queue.push_back(p); Event("received",p,now);
  }

  void Step(int64_t now, int64_t dt, int64_t wall) {
    if (dt!=1000 || (lastNow>=0 && now<=lastNow)) {Fault("clock_or_step",now);return;}
    lastNow=now;
    if (!active) return;
    // 원래 guard의 100 ms sim 유효 기한과 실행별 wall watchdog을 보존한다.
    if (now>=appliedDeadline || wall-lastWall>=wallTimeoutNs) {Fault("expired",now);return;}
    if (queue.empty()) return;
    if (queue.front().at<now) {Fault("missed_apply_step",now);return;}
    if (queue.front().at!=now) return;
    const auto p=queue.front(); queue.pop_front();
    if (p.deadline<=now) {Fault("expired_at_apply",now);return;}
    speed=p.speed; steering=p.steering; appliedDeadline=p.deadline; applied=p;
    Event("applied",p,now);
  }

  void Fault(const std::string &reason, int64_t now) {
    if (!failed) { ScheduledPacket p; p.seq=lastSeq; p.epoch=epoch.empty()?"none":epoch;
      Event("fault",p,now,reason); }
    failed=true; active=false; speed=steering=0; Cancel(now);
  }
  std::vector<std::string> TakeEvents() {std::vector<std::string> out;out.swap(events);return out;}
  double speed{},steering{};
  bool active{},failed{};
  int64_t wallTimeoutNs=30000000000LL;
  ScheduledPacket applied;

private:
  void Cancel(int64_t now) {for(const auto &p:queue) Event("cancelled",p,now);queue.clear();}
  void Event(const std::string &type,const ScheduledPacket &p,int64_t now,const std::string &reason="") {
    std::ostringstream s;
    s<<std::setprecision(17)<<"{\"event_seq\":"<<++eventSeq<<",\"type\":\""<<type
     <<"\",\"seq\":"<<p.seq<<",\"kind\":\""<<p.kind<<"\",\"epoch\":\""<<p.epoch
     <<"\",\"stamp_us\":"<<p.stamp<<",\"apply_us\":"<<p.at<<",\"deadline_us\":"<<p.deadline
     <<",\"sim_us\":"<<now<<",\"speed\":"<<p.speed<<",\"steering\":"<<p.steering
     <<",\"reason\":\""<<reason<<"\"}";
    events.push_back(s.str());
  }
  std::deque<ScheduledPacket> queue;
  std::vector<std::string> events;
  std::set<std::string> seen;
  std::string epoch;
  int64_t lastSeq{},lastStamp{-1},lastAt{-1},lastNow{-1},lastWall{},appliedDeadline{};
  uint64_t eventSeq{};
};
} // namespace arena
