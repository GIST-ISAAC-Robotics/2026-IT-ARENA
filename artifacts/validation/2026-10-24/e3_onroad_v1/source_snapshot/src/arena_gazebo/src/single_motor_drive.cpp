#include "arena_gazebo/command_trace.hpp"
#include "arena_gazebo/drivetrain.hpp"
#include "arena_gazebo/scheduled_apply.hpp"

#include <chrono>
#include <iomanip>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>

namespace arena
{
class SingleMotorDrive : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate,
                         public gz::sim::ISystemPostUpdate
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm, gz::sim::EventManager &) override
  {
    gz::sim::Model model(entity);
    auto get = [&](const char *name, double value) { return sdf->Get<double>(name, value).first; };
    parameters.radius = get("wheel_radius", parameters.radius);
    parameters.ratio = get("gear_ratio", parameters.ratio);
    parameters.efficiency = get("gear_efficiency", parameters.efficiency);
    parameters.torqueLimit = get("motor_torque_limit", parameters.torqueLimit);
    parameters.brakeTorqueLimit = get("motor_brake_torque_limit", parameters.brakeTorqueLimit);
    parameters.freeSpeed = get("motor_free_speed", parameters.freeSpeed);
    parameters.responseTime = get("motor_response_time", parameters.responseTime);
    parameters.speedKp = get("speed_kp", parameters.speedKp);
    parameters.speedKi = get("speed_ki", parameters.speedKi);
    parameters.maxSpeed = get("max_speed", parameters.maxSpeed);
    parameters.acceleration = get("acceleration_limit", parameters.acceleration);
    parameters.carrierDrag = get("carrier_drag", 0);
    parameters.differentialViscosity = get("differential_viscosity", 0);
    parameters.differentialTorqueLimit = get("differential_torque_limit", 0);
    wheelbase = get("wheelbase", 0.135);  // SDF 누락 시 대체값. 실행값은 vehicle.yaml 주입
    track = get("track_width", 0.135);
    wheelLimit = get("steering_limit", 0.45);
    servoGain = get("steering_gain", 0.40);
    servoIntegralGain = get("steering_integral_gain", 0.25);
    servoDamping = get("steering_velocity_gain", 0.003);
    servoTorque = get("steering_torque_limit", 0.12);
    servoRate = get("steering_rate_limit", 4.0);
    timeout = get("command_timeout", 0.5);
    for (const double value : {wheelbase, track, wheelLimit, servoGain, servoIntegralGain, servoDamping,
                               servoTorque, servoRate, timeout})
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument("조향/감시 매개변수는 유한한 양수여야 합니다.");
    if (wheelLimit >= 1.4) throw std::invalid_argument("조향 한계는 1.4 rad 미만이어야 합니다.");
    zeroSpeedMode = sdf->Get<std::string>("evaluation_zero_speed_mode", "legacy").first;
    // 평가 전용 명령 전환 이벤트. 기본 SDF에는 없으며 꺼져 있으면 아래 추적 분기는 실행되지 않습니다.
    traceEnabled = sdf->Get<bool>("evaluation_command_trace", false).first;
    scheduledEnabled = sdf->Get<bool>("evaluation_scheduled_apply", false).first;
    if (scheduledEnabled && !traceEnabled) throw std::invalid_argument("scheduled apply requires trace");
    scheduled.wallTimeoutNs = static_cast<int64_t>(get("evaluation_wall_timeout", 30) * 1e9);
    motor = std::make_unique<SingleMotor>(parameters, ParseZeroSpeedMode(zeroSpeedMode));
    for (const auto &name : {"rear_left_wheel_joint", "rear_right_wheel_joint",
                            "front_left_steering_joint", "front_right_steering_joint"})
    {
      const auto joint = model.JointByName(ecm, name);
      if (joint == gz::sim::kNullEntity) throw std::runtime_error("구동 관절을 찾을 수 없습니다.");
      joints.push_back(joint);
      gz::sim::Joint(joint).EnableVelocityCheck(ecm);
      gz::sim::Joint(joint).EnablePositionCheck(ecm);
    }
    chassis = model.LinkByName(ecm, "chassis");
    gz::sim::Link(chassis).EnableVelocityChecks(ecm);
    const auto prefix = "/model/" + model.Name(ecm);
    if (scheduledEnabled) {
      transport.Subscribe(prefix + "/eval/scheduled_command", &SingleMotorDrive::OnScheduled, this);
      scheduledTopic=transport.Advertise<gz::msgs::StringMsg>(prefix+"/eval/scheduled_events");
    } else {
      transport.Subscribe(prefix + "/cmd_vel", &SingleMotorDrive::OnCommand, this);
      transport.Subscribe(prefix + "/steer_angle", &SingleMotorDrive::OnSteering, this);
    }
    telemetry = transport.Advertise<gz::msgs::StringMsg>(prefix + "/drivetrain");
    odometry = transport.Advertise<gz::msgs::Odometry>(prefix + "/odometry");
    transform = transport.Advertise<gz::msgs::Pose_V>(prefix + "/tf");
    if (traceEnabled)
    {
      traceTopic = transport.Advertise<gz::msgs::StringMsg>(prefix + "/eval/command_trace");
      gzmsg << "평가 전용 명령 수신/적용 전환 계측 켜짐: " << prefix << "/eval/command_trace\n";
    }
    gzmsg << "단일 모터 토크 구동: 좌우 뒷바퀴 + 기계식 차동, 속도 강제 지정 없음\n";
  }

  void OnCommand(const gz::msgs::Twist &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commandSpeed = std::isfinite(message.linear().x()) ? message.linear().x() : 0;
    commandPending = true;
    if (traceEnabled) speedRx.Received(commandSpeed, SteadyNs(), traceIteration, traceSimNs);
  }

  void OnScheduled(const gz::msgs::StringMsg &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    scheduled.Receive(message.data(), std::max<int64_t>(0,traceSimNs/1000),SteadyNs(),traceIteration);
  }
  void OnSteering(const gz::msgs::Double &message)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commandSteering = std::isfinite(message.data()) ? message.data() : 0;
    steeringPending = true;
    if (traceEnabled) steeringRx.Received(commandSteering, SteadyNs(), traceIteration, traceSimNs);
  }

  static double JointValue(const gz::sim::EntityComponentManager &ecm, gz::sim::Entity joint,
                           bool position = false)
  {
    if (position)
    {
      const auto *c = ecm.Component<gz::sim::components::JointPosition>(joint);
      return c && !c->Data().empty() ? c->Data()[0] : 0;
    }
    const auto *c = ecm.Component<gz::sim::components::JointVelocity>(joint);
    return c && !c->Data().empty() ? c->Data()[0] : 0;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || !motor) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0 || now < lastTime)
    {
      motor->Reset();
      zeroPositiveWork = 0;
      zeroPositivePowerSteps = zeroSteps = 0;
      std::lock_guard<std::mutex> lock(mutex);
      if (scheduledEnabled) scheduled.Fault("clock_reset",static_cast<int64_t>(now*1e6));
      commandSpeed = commandSteering = 0;
      commandPending = steeringPending = false;
      lastCommand = lastSteering = -1e9;
      odomX = odomY = odomYaw = 0;
      lastPublish = -1e9;
      lastTime = now;
      for (int i = 0; i < 2; ++i) { servoReference[i] = 0; servoIntegral[i] = 0; }
      for (const auto joint : joints) gz::sim::Joint(joint).SetForce(ecm, {0});
      if (traceEnabled) applyTrace.Reset(TraceInfo(info), traceEvents);
      return;
    }
    lastTime = now;
    double speed, steering;
    ReceiveSnapshot speedSnapshot, steeringSnapshot;
    {
      std::lock_guard<std::mutex> lock(mutex);
      if (scheduledEnabled) {
        scheduled.Step(std::chrono::duration_cast<std::chrono::microseconds>(info.simTime).count(),
                       std::chrono::duration_cast<std::chrono::microseconds>(info.dt).count(),SteadyNs());
        speed=scheduled.speed; steering=scheduled.steering; stale=!scheduled.active;
        // 예약 모드의 수신 표지는 실제 적용된 envelope의 콜백 경계다. 전체 receipt는 별도 ACK가 기준이다.
        const auto &p=scheduled.applied;
        const ReceiveMark mark{static_cast<uint64_t>(p.seq),p.rxWall,p.rxIteration,p.rxSim};
        speedSnapshot=steeringSnapshot={mark,static_cast<uint64_t>(p.seq),0,0};
      } else {
      if (commandPending) { lastCommand = now; commandPending = false; }
      if (steeringPending) { lastSteering = now; steeringPending = false; }
      stale = now - lastCommand >= timeout || now - lastSteering >= timeout;
      speed = stale ? 0 : commandSpeed;
      steering = stale ? 0 : commandSteering;
      }
      if (traceEnabled)
      {
        // 같은 잠금 구역에서 읽은 값의 수신 표지만 복사합니다. 값 샘플 시점과 잠금 경계는 그대로입니다.
        if (!scheduledEnabled) {speedSnapshot = speedRx.Take(); steeringSnapshot = steeringRx.Take();}
        traceIteration = info.iterations;
        traceSimNs = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
      }
    }
    if (traceEnabled)
      applyTrace.Step(TraceInfo(info), speed, steering, stale, speedSnapshot, steeringSnapshot, traceEvents);
    state = motor->Step(speed, JointValue(ecm, joints[0]), JointValue(ecm, joints[1]), dt);
    // 토크 계산 직전 속도와 동시각 동력. PostUpdate 속도와 곱한 값과 구분합니다.
    if (state.targetSpeed == 0)
    {
      const double power = state.motorTorque * state.motorSpeed;
      zeroPositiveWork += std::max(0.0, power) * dt;
      zeroPositivePowerSteps += power > 1e-9 ? 1 : 0;
      ++zeroSteps;
    }
    gz::sim::Joint(joints[0]).SetForce(ecm, {state.leftTorque});
    gz::sim::Joint(joints[1]).SetForce(ecm, {state.rightTorque});
    const auto angles = SteeringAngles(steering, wheelbase, track, wheelLimit);
    double efforts[2]{};
    for (int index = 0; index != 2; ++index)
    {
      const auto joint = joints[index + 2];
      const double target = index == 0 ? angles.first : angles.second;
      servoReference[index] += std::clamp(target - servoReference[index], -servoRate * dt, servoRate * dt);
      const double error = servoReference[index] - JointValue(ecm, joint, true);
      const double raw = servoGain * error + servoIntegral[index] - servoDamping * JointValue(ecm, joint);
      if (std::abs(raw) < servoTorque / 2 || raw * error < 0)
        servoIntegral[index] = std::clamp(servoIntegral[index] + servoIntegralGain * error * dt,
                                          -servoTorque / 2, servoTorque / 2);
      efforts[index] = std::clamp(raw, -servoTorque, servoTorque);
    }
    // 두 조향 관절은 같은 가상 서보 명령과 총 토크 예산을 공유합니다.
    // 실물 링크 레버비/서보 전기·열 모델은 아니며 실측 후 보정해야 합니다.
    const double scale = std::max(1.0, (std::abs(efforts[0]) + std::abs(efforts[1])) / servoTorque);
    for (int index = 0; index != 2; ++index)
      gz::sim::Joint(joints[index + 2]).SetForce(ecm, {efforts[index] / scale});
  }

  void PostUpdate(const gz::sim::UpdateInfo &info, const gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || !motor) return;
    if (scheduledEnabled) {
      std::vector<std::string> events;
      {std::lock_guard<std::mutex> lock(mutex);events=scheduled.TakeEvents();}
      for (const auto &text:events) {gz::msgs::StringMsg event;event.set_data(text);scheduledTopic.Publish(event);}
    }
    if (traceEnabled && !traceEvents.empty())
    {
      // PreUpdate에서 만든 저빈도 전환 이벤트만 같은 iteration의 PostUpdate에서 발행합니다.
      for (const auto &text : traceEvents)
      {
        gz::msgs::StringMsg event;
        event.set_data(text);
        traceTopic.Publish(event);
      }
      traceEvents.clear();
    }
    const double now = std::chrono::duration<double>(info.simTime).count();
    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0) return;
    const double wl = JointValue(ecm, joints[0]), wr = JointValue(ecm, joints[1]);
    const double dl = JointValue(ecm, joints[2], true), dr = JointValue(ecm, joints[3], true);
    const double tl = std::tan(dl), tr = std::tan(dr);
    const double tangent = std::abs(tl + tr) < 1e-6 ? 0 : 2 * tl * tr / (tl + tr);
    const double wheelSpeed = parameters.radius * (wl + wr) / 2;
    const double yawRate = wheelSpeed * tangent / wheelbase;
    odomX += wheelSpeed * std::cos(odomYaw + yawRate * dt / 2) * dt;
    odomY += wheelSpeed * std::sin(odomYaw + yawRate * dt / 2) * dt;
    odomYaw += yawRate * dt;
    if (now - lastPublish < 0.02) return;
    lastPublish = now;

    gz::msgs::Odometry message;
    auto *stamp = message.mutable_header()->mutable_stamp();
    stamp->set_sec(static_cast<int64_t>(now));
    stamp->set_nsec(static_cast<int32_t>((now - std::floor(now)) * 1e9));
    auto *frame = message.mutable_header()->add_data();
    frame->set_key("frame_id"); frame->add_value("odom");
    auto *child = message.mutable_header()->add_data();
    child->set_key("child_frame_id"); child->add_value("base_link");
    message.mutable_pose()->mutable_position()->set_x(odomX);
    message.mutable_pose()->mutable_position()->set_y(odomY);
    message.mutable_pose()->mutable_orientation()->set_z(std::sin(odomYaw / 2));
    message.mutable_pose()->mutable_orientation()->set_w(std::cos(odomYaw / 2));
    message.mutable_twist()->mutable_linear()->set_x(wheelSpeed);
    message.mutable_twist()->mutable_angular()->set_z(yawRate);
    odometry.Publish(message);
    gz::msgs::Pose_V transforms;
    transforms.mutable_header()->CopyFrom(message.header());
    auto *pose = transforms.add_pose();
    pose->CopyFrom(message.pose()); pose->mutable_header()->CopyFrom(message.header());
    transform.Publish(transforms);

    const auto worldPose = gz::sim::worldPose(chassis, ecm);
    const auto velocity = gz::sim::Link(chassis).WorldLinearVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    const auto localVelocity = worldPose.Rot().RotateVectorReverse(velocity);
    const auto angular = gz::sim::Link(chassis).WorldAngularVelocity(ecm).value_or(gz::math::Vector3d::Zero);
    // 이 정답 진단은 시험 전용. 자율주행/ToF 안전층은 구독하지 않습니다.
    std::ostringstream json;
    json << std::setprecision(12) << "{\"sim_time_s\":" << now
      << ",\"motor_speed_rad_s\":" << parameters.ratio * (wl + wr) / 2
      << ",\"motor_torque_nm\":" << state.motorTorque
      << ",\"evaluation_zero_speed_mode\":\"" << zeroSpeedMode << "\""
      << ",\"applied_target_mps\":" << state.targetSpeed
      << ",\"pre_motor_speed_rad_s\":" << state.motorSpeed
      << ",\"pre_motor_power_w\":" << state.motorTorque * state.motorSpeed
      << ",\"pi_integral_before_nm\":" << state.integralBefore
      << ",\"pi_integral_after_nm\":" << state.integralAfter
      << ",\"raw_demand_nm\":" << state.rawDemand
      << ",\"limited_demand_nm\":" << state.limitedDemand
      << ",\"zero_positive_work_j\":" << zeroPositiveWork
      << ",\"zero_positive_power_steps\":" << zeroPositivePowerSteps
      << ",\"zero_steps\":" << zeroSteps
      << ",\"left_torque_nm\":" << state.leftTorque << ",\"right_torque_nm\":" << state.rightTorque
      << ",\"differential_torque_nm\":" << state.differentialTorque
      << ",\"loss_power_w\":" << state.lossPower << ",\"reference_speed_mps\":" << state.referenceSpeed
      << ",\"left_speed_rad_s\":" << wl << ",\"right_speed_rad_s\":" << wr
      << ",\"left_steering_rad\":" << dl << ",\"right_steering_rad\":" << dr
      << ",\"watchdog_stop\":" << (stale ? "true" : "false")
      << ",\"truth_x_m\":" << worldPose.Pos().X() << ",\"truth_y_m\":" << worldPose.Pos().Y()
      << ",\"truth_z_m\":" << worldPose.Pos().Z()
      << ",\"truth_roll_rad\":" << worldPose.Rot().Roll()
      << ",\"truth_pitch_rad\":" << worldPose.Rot().Pitch()
      << ",\"truth_yaw_rad\":" << worldPose.Rot().Yaw()
      << ",\"truth_longitudinal_mps\":" << localVelocity.X()
      << ",\"truth_lateral_mps\":" << localVelocity.Y()
      << ",\"truth_yaw_rate_rad_s\":" << angular.Z()
      << ",\"wheel_surface_speed_mps\":" << wheelSpeed << "}";
    gz::msgs::StringMsg diagnostic; diagnostic.set_data(json.str()); telemetry.Publish(diagnostic);
  }
private:
  static int64_t SteadyNs()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }
  static TraceStep TraceInfo(const gz::sim::UpdateInfo &info)
  {
    return {info.iterations, std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count(),
            std::chrono::duration_cast<std::chrono::nanoseconds>(info.dt).count(), SteadyNs()};
  }

  DriveParameters parameters;
  DriveState state;
  std::string zeroSpeedMode{"legacy"};
  double zeroPositiveWork{};
  uint64_t zeroPositivePowerSteps{}, zeroSteps{};
  std::unique_ptr<SingleMotor> motor;
  gz::transport::Node transport;
  gz::transport::Node::Publisher telemetry, odometry, transform;
  std::vector<gz::sim::Entity> joints;
  gz::sim::Entity chassis{gz::sim::kNullEntity};
  std::mutex mutex;
  bool commandPending{}, steeringPending{}, stale{true};
  double commandSpeed{}, commandSteering{}, lastCommand{-1e9}, lastSteering{-1e9};
  double wheelbase{}, track{}, wheelLimit{}, servoGain{}, servoIntegralGain{}, servoDamping{}, servoTorque{}, servoRate{}, timeout{};
  double servoReference[2]{}, servoIntegral[2]{};
  double lastTime{}, lastPublish{-1e9}, odomX{}, odomY{}, odomYaw{};
  // 평가 전용 추적 상태. traceIteration/traceSimNs와 *Rx는 mutex 안에서만 접근합니다.
  bool traceEnabled{};
  bool scheduledEnabled{};
  ScheduledApply scheduled;
  gz::transport::Node::Publisher scheduledTopic;
  ChannelReceiveTrace speedRx, steeringRx;
  CommandApplyTrace applyTrace;
  std::vector<std::string> traceEvents;
  uint64_t traceIteration{};
  int64_t traceSimNs{-1};
  gz::transport::Node::Publisher traceTopic;
};
}  // namespace arena

GZ_ADD_PLUGIN(arena::SingleMotorDrive, gz::sim::System,
  arena::SingleMotorDrive::ISystemConfigure, arena::SingleMotorDrive::ISystemPreUpdate,
  arena::SingleMotorDrive::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(arena::SingleMotorDrive, "arena::SingleMotorDrive")
