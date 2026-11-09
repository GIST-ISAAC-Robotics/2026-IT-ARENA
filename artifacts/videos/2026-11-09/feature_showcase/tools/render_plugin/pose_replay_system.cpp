// 영상 재렌더 전용 gz-sim 시스템. 저장된 Gazebo 자세 기록을 정적 모델에 그대로 적용하고
// 관찰 카메라 이미지를 시뮬레이션 시각과 함께 PPM으로 저장한다.
// 물리·제어기·센서 입력을 재실행하지 않으며 주행 판단을 만들지 않는다.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/image.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>

namespace showcase
{
struct Sample
{
  double t, x, y, z, roll, pitch, yaw;
};

class PoseReplay : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    const std::string poseFile = sdf->Get<std::string>("pose_file");
    frameDir = sdf->Get<std::string>("frame_dir");
    doneFile = sdf->Get<std::string>("done_file");
    endTime = sdf->Get<double>("end_time");
    const std::string topic = sdf->Get<std::string>("image_topic");
    std::ifstream in(poseFile);
    std::string line;
    while (std::getline(in, line))
    {
      if (line.empty() || line[0] == '#') continue;
      std::istringstream ss(line);
      std::string name;
      Sample s{};
      ss >> name >> s.t >> s.x >> s.y >> s.z >> s.roll >> s.pitch >> s.yaw;
      if (!ss.fail()) tracks[name].push_back(s);
    }
    for (auto &kv : tracks)
      std::sort(kv.second.begin(), kv.second.end(), [](const Sample &a, const Sample &b) { return a.t < b.t; });
    gzmsg << "[PoseReplay] tracks=" << tracks.size() << " end_time=" << endTime << std::endl;
    node.Subscribe(topic, &PoseReplay::OnImage, this);
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    const double t = std::chrono::duration<double>(info.simTime).count();
    for (auto &kv : tracks)
    {
      auto it = entities.find(kv.first);
      gz::sim::Entity entity = gz::sim::kNullEntity;
      if (it == entities.end())
      {
        entity = ecm.EntityByComponents(gz::sim::components::Name(kv.first), gz::sim::components::Model());
        if (entity == gz::sim::kNullEntity) continue;
        entities[kv.first] = entity;
      }
      else
      {
        entity = it->second;
      }
      const Sample s = Interp(kv.second, t);
      auto *pose = ecm.Component<gz::sim::components::Pose>(entity);
      if (pose == nullptr) continue;
      *pose = gz::sim::components::Pose(gz::math::Pose3d(s.x, s.y, s.z, s.roll, s.pitch, s.yaw));
      ecm.SetChanged(entity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);
    }
    if (!done && t > endTime)
    {
      std::lock_guard<std::mutex> lock(mutex);
      std::ofstream(doneFile) << "sim_time " << t << " frames " << frames << "\n";
      done = true;
    }
  }

private:
  static Sample Interp(const std::vector<Sample> &v, double t)
  {
    if (t <= v.front().t) return v.front();
    if (t >= v.back().t) return v.back();
    auto hi = std::lower_bound(v.begin(), v.end(), t, [](const Sample &a, double x) { return a.t < x; });
    auto lo = hi - 1;
    const double w = (t - lo->t) / (hi->t - lo->t);
    auto lerpAngle = [w](double a, double b) { return a + w * std::atan2(std::sin(b - a), std::cos(b - a)); };
    return Sample{t, lo->x + w * (hi->x - lo->x), lo->y + w * (hi->y - lo->y), lo->z + w * (hi->z - lo->z),
                  lerpAngle(lo->roll, hi->roll), lerpAngle(lo->pitch, hi->pitch), lerpAngle(lo->yaw, hi->yaw)};
  }

  void OnImage(const gz::msgs::Image &msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    if (done) return;
    const long long ns = static_cast<long long>(msg.header().stamp().sec()) * 1000000000LL + msg.header().stamp().nsec();
    char name[64];
    std::snprintf(name, sizeof(name), "/f_%012lld.ppm", ns);
    std::ofstream out(frameDir + name, std::ios::binary);
    out << "P6\n" << msg.width() << " " << msg.height() << "\n255\n";
    const std::string &data = msg.data();
    const unsigned int row = msg.width() * 3;
    const unsigned int step = msg.step() ? msg.step() : row;
    for (unsigned int r = 0; r < msg.height(); ++r) out.write(data.data() + r * step, row);
    ++frames;
  }

  std::map<std::string, std::vector<Sample>> tracks;
  std::map<std::string, gz::sim::Entity> entities;
  gz::transport::Node node;
  std::mutex mutex;
  std::string frameDir, doneFile;
  double endTime{0.0};
  bool done{false};
  long long frames{0};
};
}  // namespace showcase

GZ_ADD_PLUGIN(showcase::PoseReplay, gz::sim::System, showcase::PoseReplay::ISystemConfigure,
              showcase::PoseReplay::ISystemPreUpdate)
