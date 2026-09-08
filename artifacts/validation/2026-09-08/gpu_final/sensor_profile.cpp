#include <gz/rendering.hh>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

// 센서 렌더링/CPU로 결과 회수만 계측. 물리·ROS·공식 코스의 RTF가 아니다.
int main(int argc, char **argv) {
  if (argc != 3) return 2;
  const std::string mode = argv[1];
  auto *engine = gz::rendering::engine("ogre2", {{"headless", argv[2]}});
  if (!engine) return 3;
  auto scene = engine->CreateScene("sensor_benchmark");
  scene->SetAmbientLight(0.8, 0.8, 0.8);
  scene->SetBackgroundColor(0.05, 0.05, 0.05);
  auto root = scene->RootVisual();
  for (int i = 0; i < 4; ++i) {
    auto wall = scene->CreateVisual();
    wall->AddGeometry(scene->CreateBox());
    wall->SetLocalScale(1, 8, 8);
    wall->SetLocalPosition(3 * std::cos(i*M_PI/2), 3 * std::sin(i*M_PI/2), 0);
    wall->SetLocalRotation(0, 0, i*M_PI/2);
    auto material = scene->CreateMaterial();
    material->SetAmbient(0.8, 0.2, 0.1);
    material->SetDiffuse(0.8, 0.2, 0.1);
    wall->SetMaterial(material);
    root->AddChild(wall);
  }
  unsigned int callbacks = 0;
  double center = 0;
  std::vector<gz::common::ConnectionPtr> connections;
  std::vector<gz::rendering::GpuRaysPtr> rays;
  gz::rendering::CameraPtr camera;
  gz::rendering::DepthCameraPtr depth;
  gz::rendering::Image rgb;
  int iterations;
  if (mode == "rgbd") {
    iterations = 30;
    camera = scene->CreateCamera();
    depth = scene->CreateDepthCamera();
    for (auto sensor : {camera, std::static_pointer_cast<gz::rendering::Camera>(depth)}) {
      sensor->SetImageWidth(848); sensor->SetImageHeight(480);
      sensor->SetAspectRatio(848.0/480); sensor->SetHFOV(1.2);
      sensor->SetNearClipPlane(0.05); sensor->SetFarClipPlane(20);
      root->AddChild(sensor);
    }
    rgb = camera->CreateImage();
    connections.push_back(depth->ConnectNewDepthFrame(
      [&](const float *data, unsigned int w, unsigned int h, unsigned int c, const std::string &) {
        if (w != 848 || h != 480 || c != 1) throw std::runtime_error("depth shape");
        center = data[(h/2*w+w/2)*c]; ++callbacks;
      }));
  } else if (mode == "lidar500" || mode == "tof6") {
    iterations = mode == "lidar500" ? 500 : 15;
    int count = mode == "tof6" ? 6 : 1;
    for (int i = 0; i < count; ++i) {
      auto ray = scene->CreateGpuRays();
      bool tof = mode == "tof6";
      ray->SetRayCount(tof ? 8 : 500); ray->SetVerticalRayCount(tof ? 8 : 1);
      ray->SetAngleMin(tof ? -0.45 : -M_PI);
      ray->SetAngleMax(tof ? 0.45 : M_PI-2*M_PI/500);
      ray->SetVerticalAngleMin(tof ? -0.45 : 0);
      ray->SetVerticalAngleMax(tof ? 0.45 : 0);
      ray->SetNearClipPlane(0.05); ray->SetFarClipPlane(20);
      root->AddChild(ray);
      connections.push_back(ray->ConnectNewGpuRaysFrame(
        [&, tof](const float *data, unsigned int w, unsigned int h, unsigned int c, const std::string &) {
          if (w != (tof ? 8u : 500u) || h != (tof ? 8u : 1u) || c != 3)
            throw std::runtime_error("ray shape");
          center = data[(h/2*w+w/2)*c]; ++callbacks;
        }));
      rays.push_back(ray);
    }
  } else return 4;
  auto step = [&]() {
    if (camera) { camera->Capture(rgb); depth->Update(); }
    for (auto &ray : rays) ray->Update();
  };
  for (int i = 0; i < 3; ++i) step();
  callbacks = 0;
  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < iterations; ++i) step();
  auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now()-start).count();
  bool valid = callbacks == iterations * (rays.empty() ? 1 : rays.size()) &&
               std::isfinite(center) && std::abs(center-2.5) < 0.1;
  unsigned int red = 0;
  if (camera) {
    auto *pixels = rgb.Data<unsigned char>();
    auto offset = (240*848+424)*3;
    red = pixels[offset];
    valid = valid && red > pixels[offset+1] && red > pixels[offset+2] && red > 30;
  }
  std::cout << "PROFILE {\"mode\":\"" << mode << "\",\"iterations\":" << iterations
            << ",\"callbacks\":" << callbacks << ",\"wall_s\":" << elapsed
            << ",\"center_m\":" << center << ",\"rgb_center_red\":" << red
            << ",\"valid\":" << (valid ? "true" : "false") << "}" << std::endl;
  connections.clear();
  engine->DestroyScene(scene);
  scene.reset(); rays.clear(); camera.reset(); depth.reset(); root.reset();
  gz::rendering::unloadEngine("ogre2");
  std::cout << "CLEAN_EXIT" << std::endl;
  return valid ? 0 : 5;
}
