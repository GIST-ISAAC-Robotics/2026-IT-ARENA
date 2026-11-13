#pragma once
// ST3215 RAM 순서기. 단일 소유자이며 GPIO/UART/HAL/receiver 상태를 소유하지 않는다.
#include "cad_servo_goal.hpp"
#include "st3215_transaction.hpp"

namespace arena_servo {

struct RuntimeServoSnapshot {
  bool armed=false;
  std::int64_t generation=0;
  bool drive_seen=false;
  double steering_rad=0.0;
};

// 모두 명시 입력이다. 0 기본값은 운영 설정이 아니며 유효 프로필 없이는 송신하지 않는다.
struct RuntimeServoConfig {
  unsigned id=254, goal_speed=0, goal_acceleration=0;
  std::int64_t position_period_us=0, observation_max_age_us=0;
  double command_limit=0.0, wheelbase=0.0, track=0.0, wheel_limit=0.0;
  arena_steering::ZeroProfile zero{};
};

enum class RuntimeServoState { Probe, Standby, Engage, Engaged, Fault };
enum class RuntimeServoFault { None, InvalidConfig, InvalidMode, InvalidPosition,
                               InvalidGoal, Transaction, InvalidSnapshot };

class RuntimeServo {
 public:
  // map 및 그 samples는 이 객체보다 오래 살아야 한다. 설정/프로필은 생성 후 불변이다.
  RuntimeServo(const arena_steering::CadMapping& map, const RuntimeServoConfig& config)
      : map_(map), config_(config), valid_(valid_config()) {
    if (!valid_) fail(RuntimeServoFault::InvalidConfig);
  }
  RuntimeServo(const RuntimeServo&)=delete;
  RuntimeServo& operator=(const RuntimeServo&)=delete;

  // snapshot은 같은 소유 작업의 실제 receiver 검증 결과다. 비신뢰 host 입력용 API가 아니다.
  // 최신 snapshot을 먼저 반영한 뒤 ACK를 소비한다. STOP 뒤 ACK가 Engaged를 복원하지 않는다.
  // sink는 즉시 수락한 바이트 수를 반환한다. 기다림/재시도/목표 큐는 없다.
  template<class Sink>
  void step(std::int64_t now, const RuntimeServoSnapshot& snapshot,
            const std::uint8_t* rx, std::size_t size, Sink&& sink) {
    if (now<0 || (clock_seen_ && now<last_step_us_)) {
      transaction_.tick(now); fail(RuntimeServoFault::Transaction); return;
    }
    clock_seen_=true; last_step_us_=now;
    observe(snapshot);
    if ((operation_==Operation::Goal || replacement_pending_) && now>=goal_deadline_us_) {
      sequence_error_=Error::Deadline; fail(RuntimeServoFault::Transaction); return;
    }
    transaction_.tick(now);
    if (transaction_.phase()==Phase::Fault) fail(RuntimeServoFault::Transaction);
    if (state_==RuntimeServoState::Fault) return;
    transaction_.feed(now,rx,size);
    if (transaction_.phase()==Phase::Fault) { fail(RuntimeServoFault::Transaction); return; }
    Response response;
    if (transaction_.take(response)) complete(response);
    if (state_==RuntimeServoState::Fault) return;
    if (operation_==Operation::None) schedule(now);
    if (state_==RuntimeServoState::Fault) return;
    transaction_.pump(now,[&](const std::uint8_t* bytes,std::size_t length)->std::size_t {
      const auto accepted=sink(bytes,length);
      if (accepted>0) sent_=true;
      return accepted;
    });
    if (transaction_.phase()==Phase::Fault) fail(RuntimeServoFault::Transaction);
  }

  bool ready(std::int64_t now) const {
    return valid_ && state_!=RuntimeServoState::Fault && mode_ok_ && has_position_ &&
        now>=position_us_ && now-position_us_<config_.observation_max_age_us;
  }
  bool engaged() const { return state_==RuntimeServoState::Engaged && authorized_; }
  RuntimeServoState state() const { return state_; }
  RuntimeServoFault fault() const { return fault_; }
  Phase transaction_phase() const { return sequence_error_==Error::None?transaction_.phase():Phase::Fault; }
  Error transaction_error() const { return sequence_error_==Error::None?transaction_.error():sequence_error_; }
  std::int64_t transaction_deadline() const {
    return operation_==Operation::Goal || replacement_pending_?goal_deadline_us_:transaction_.deadline();
  }
  bool has_position() const { return has_position_; }
  unsigned position() const { return position_; }
  std::int64_t position_observed_us() const { return position_us_; }

  // 외부가 TX/RX drain 및 충분한 무송신 구간을 확보한 뒤에만 호출한다.
  // Fault 원목표를 재시도하지 않는다. 같은 ARM 세대는 복구 뒤에도 쓰기를 재개하지 못한다.
  // ST 응답에는 요청 ID가 없으므로 실제 late ACK 완전 제거는 이 함수만으로 보장하지 않는다.
  bool reset_after_drain(std::int64_t now) {
    if (!valid_ || (clock_seen_ && now<last_step_us_) || !transaction_.reset_after_drain(now)) return false;
    clock_seen_=true; last_step_us_=now;
    state_=RuntimeServoState::Probe; fault_=RuntimeServoFault::None;
    operation_=Operation::None; sent_=authorized_=mode_ok_=has_position_=has_goal_=false;
    replacement_pending_=false; sequence_error_=Error::None;
    engage_stage_=EngageStage::Read;
    return true;
  }

 private:
  enum class Operation { None, Mode, Position, EngagePosition, Hold, Torque, Goal };
  enum class EngageStage { Read, Hold, Torque };

  bool valid_config() const {
    // 기존 Transaction API의 허용범위다. probe 수치가 주행용으로 검증되었다는 뜻이 아니다.
    return config_.id<=253 && config_.goal_speed>0 && config_.goal_speed<=400 &&
        config_.goal_acceleration>=5 && config_.goal_acceleration<=20 &&
        config_.position_period_us>0 && config_.observation_max_age_us>0 &&
        arena_steering::ResolveCadServoGoal(map_,config_.zero,0.0,config_.command_limit,
            config_.wheelbase,config_.track,config_.wheel_limit).valid;
  }
  void fail(RuntimeServoFault why) {
    if (state_!=RuntimeServoState::Fault) fault_=why;
    state_=RuntimeServoState::Fault; authorized_=false; has_goal_=false;
  }
  void discard_unsent() {
    if (operation_!=Operation::None && !sent_) {
      // 아직 UART에 1 byte도 맡기지 않은 요청이다. 응답이 생길 수 없어 drain이 필요 없다.
      transaction_=Transaction{}; operation_=Operation::None;
    }
  }
  void observe(const RuntimeServoSnapshot& current) {
    const bool changed=!snapshot_seen_ || current.armed!=snapshot_.armed ||
        current.generation!=snapshot_.generation;
    if (!changed && state_!=RuntimeServoState::Fault && operation_==Operation::Goal && !sent_ &&
        (!current.drive_seen || current.steering_rad!=snapshot_.steering_rad)) {
      // 미송신 목표는 교체하되 최초 요청의 기존 30 ms 마감은 연장하지 않는다.
      replacement_pending_=current.drive_seen;
      discard_unsent();
    }
    snapshot_=current; snapshot_seen_=true;
    if (current.generation<0) { fail(RuntimeServoFault::InvalidSnapshot); return; }
    if (changed) {
      if (state_==RuntimeServoState::Fault) return;
      replacement_pending_=false;
      discard_unsent(); authorized_=false; has_goal_=false;
      authorized_=current.armed;
      if (state_!=RuntimeServoState::Probe) state_=RuntimeServoState::Standby;
    }
  }
  bool same_authorization() const {
    return authorized_ && snapshot_.armed && operation_generation_==snapshot_.generation;
  }
  void complete(const Response& response) {
    const auto done=operation_;
    operation_=Operation::None; sent_=false;
    if (done==Operation::Mode) {
      if (response.size!=1 || response.data[0]!=0) { fail(RuntimeServoFault::InvalidMode); return; }
      mode_ok_=true;
    } else if (done==Operation::Position || done==Operation::EngagePosition) {
      const unsigned value=response.data[0]|(static_cast<unsigned>(response.data[1])<<8);
      if (response.size!=2 || value<static_cast<unsigned>(config_.zero.minimum) ||
          value>static_cast<unsigned>(config_.zero.maximum)) { fail(RuntimeServoFault::InvalidPosition); return; }
      position_=value; position_us_=response.completed_us; has_position_=true;
      if (state_==RuntimeServoState::Probe) state_=RuntimeServoState::Standby;
      if (done==Operation::EngagePosition && state_==RuntimeServoState::Engage && same_authorization()) {
        hold_position_=value; engage_stage_=EngageStage::Hold;
      }
    } else if (done==Operation::Hold && state_==RuntimeServoState::Engage && same_authorization()) {
      last_goal_=hold_position_; has_goal_=true; engage_stage_=EngageStage::Torque;
    } else if (done==Operation::Torque && state_==RuntimeServoState::Engage && same_authorization()) {
      state_=RuntimeServoState::Engaged;
    } else if (done==Operation::Goal && state_==RuntimeServoState::Engaged && same_authorization()) {
      last_goal_=inflight_goal_; has_goal_=true;
    }
  }
  void begin(Operation op, std::int64_t now, unsigned goal=0) {
    const bool replacement=op==Operation::Goal && replacement_pending_;
    replacement_pending_=false;
    bool ok=false;
    switch(op) {
      case Operation::Mode: ok=transaction_.read_mode(now,config_.id); break;
      case Operation::Position:
      case Operation::EngagePosition: ok=transaction_.read_position(now,config_.id); break;
      case Operation::Hold:
      case Operation::Goal:
        ok=transaction_.goal(now,config_.id,goal,config_.goal_speed,config_.goal_acceleration); break;
      case Operation::Torque: ok=transaction_.torque(now,config_.id,true); break;
      case Operation::None: return;
    }
    if (!ok) { fail(RuntimeServoFault::Transaction); return; }
    operation_=op; operation_generation_=snapshot_.generation; sent_=false;
    if (op==Operation::Goal) {
      inflight_goal_=goal;
      if (!replacement) goal_deadline_us_=transaction_.deadline();
    }
  }
  void schedule(std::int64_t now) {
    if (state_==RuntimeServoState::Probe) {
      begin(mode_ok_?Operation::Position:Operation::Mode,now); return;
    }
    if (state_==RuntimeServoState::Standby && authorized_ && snapshot_.armed) {
      state_=RuntimeServoState::Engage; engage_stage_=EngageStage::Read;
    }
    if (state_==RuntimeServoState::Engage) {
      if (engage_stage_==EngageStage::Read) begin(Operation::EngagePosition,now);
      else if (engage_stage_==EngageStage::Hold) begin(Operation::Hold,now,hold_position_);
      else begin(Operation::Torque,now);
      return;
    }
    if (!has_position_ || now-position_us_>=config_.position_period_us || !ready(now)) {
      begin(Operation::Position,now); return;
    }
    if (engaged() && snapshot_.drive_seen) {
      const auto goal=arena_steering::ResolveCadServoGoal(map_,config_.zero,snapshot_.steering_rad,
          config_.command_limit,config_.wheelbase,config_.track,config_.wheel_limit);
      if (!goal.valid) { fail(RuntimeServoFault::InvalidGoal); return; }
      if (!has_goal_ || goal.servo.position!=last_goal_) begin(Operation::Goal,now,goal.servo.position);
      else replacement_pending_=false; // 최신 목표가 이미 ACK된 위치면 대기 슬롯 자체가 없다.
    }
  }

  const arena_steering::CadMapping& map_;
  const RuntimeServoConfig config_;
  const bool valid_;
  Transaction transaction_;
  RuntimeServoState state_=RuntimeServoState::Probe;
  RuntimeServoFault fault_=RuntimeServoFault::None;
  Error sequence_error_=Error::None;
  Operation operation_=Operation::None;
  EngageStage engage_stage_=EngageStage::Read;
  RuntimeServoSnapshot snapshot_{};
  bool snapshot_seen_=false, authorized_=false, sent_=false, mode_ok_=false, has_position_=false, has_goal_=false;
  bool clock_seen_=false, replacement_pending_=false;
  std::int64_t operation_generation_=0, position_us_=0, last_step_us_=0;
  std::int64_t goal_deadline_us_=0;
  unsigned position_=0, hold_position_=0, inflight_goal_=0, last_goal_=0;
};

} // namespace arena_servo
