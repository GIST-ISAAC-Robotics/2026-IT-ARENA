"""평가용 0목표 변형: 기본 수식 완전 일치·수동 모드 동력 부호·실행 사본 경계."""
import importlib.util
import hashlib
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('zero_mode_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)
HEADER = 'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp'


@pytest.fixture(scope='module')
def equation_executable(tmp_path_factory):
    temp = tmp_path_factory.mktemp('zero_speed_cpp')
    old_zip = ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/official/source_snapshot.zip'
    with zipfile.ZipFile(old_zip) as archive:
        old = archive.read(HEADER)
    assert hashlib.sha256(old).hexdigest() == '9f95cfec5e5efd53c935d1db560b8ede2ccf23f16c51ef0bdee477e20d627bd0'
    (temp/'legacy.hpp').write_bytes(old)
    harness = temp/'probe.cpp'
    harness.write_text(r'''
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <iostream>
#include <random>
#define arena preserved_arena
#include "legacy.hpp"
#undef arena
#include "arena_gazebo/drivetrain.hpp"
void require(bool ok) { if (!ok) throw std::runtime_error("zero mode invariant"); }
int main(int argc, char **argv) {
  const std::string test = argv[1];
  arena::DriveParameters p;
  std::mt19937 rng(810);
  std::uniform_real_distribution<double> cmd(-6.,6.), vel(-100.,100.), step(.0001,.01);
  if (test == "legacy") {
    arena::SingleMotor now;
    preserved_arena::SingleMotor before;
    for(int i=0;i<50000;++i) {
      double t = i%5 ? cmd(rng) : 0., l=vel(rng), r=vel(rng), dt=step(rng);
      auto a=now.Step(t,l,r,dt); auto b=before.Step(t,l,r,dt);
      require(a.motorTorque==b.motorTorque && a.motorSpeed==b.motorSpeed &&
        a.leftTorque==b.leftTorque && a.rightTorque==b.rightTorque && a.lossPower==b.lossPower &&
        a.referenceSpeed==b.referenceSpeed && a.speedError==b.speedError);
      if(i%701==0) {now.Reset();before.Reset();}
    }
  } else if(test == "nonzero") {
    for(auto mode : {arena::ZeroSpeedMode::ResetIntegral, arena::ZeroSpeedMode::NoLag,
                    arena::ZeroSpeedMode::PassiveDamping}) {
      arena::SingleMotor a(p,mode), b;
      for(int i=0;i<5000;++i) {
        double t=cmd(rng); if(t==0.)t=.1;
        double l=vel(rng),r=vel(rng),dt=step(rng);
        auto x=a.Step(t,l,r,dt);auto y=b.Step(t,l,r,dt);
        require(x.motorTorque==y.motorTorque && x.leftTorque==y.leftTorque && x.lossPower==y.lossPower);
      }
    }
  } else if(test == "passive") {
    for(double efficiency : {1., .85}) {
      p.efficiency=efficiency;
      arena::SingleMotor a(p,arena::ZeroSpeedMode::PassiveDamping);
      for(int i=0;i<20000;++i) {
        double l=vel(rng),r=vel(rng);
        if(i%11==0)a.Step(cmd(rng),l,r,.001);
        auto x=a.Step(0.,l,r,step(rng));
        require(x.motorTorque*x.motorSpeed <= 1e-12);
        require(x.leftTorque*l+x.rightTorque*r <= 1e-12);
        require(x.integralAfter==0. && std::abs(x.motorTorque)<=p.brakeTorqueLimit);
      }
      auto x=a.Step(0.,0.,0.,.001);require(x.motorTorque==0.);
      x=a.Step(0.,20.,-20.,.001);require(x.motorTorque==0.); // 평균 0은 좌우 정지가 아님
      a.Reset();x=a.Step(.001,0.,0.,.001);require(x.motorTorque>0.);
    }
  } else if(test == "factors") {
    arena::SingleMotor legacy, reset(p,arena::ZeroSpeedMode::ResetIntegral), lag(p,arena::ZeroSpeedMode::NoLag);
    arena::DriveState x,y,z;
    for(int i=0;i<1000;++i) {
      x=legacy.Step(0.,30.,30.,.001);y=reset.Step(0.,30.,30.,.001);z=lag.Step(0.,30.,30.,.001);
      require(y.integralBefore==0. && y.integralAfter==0.);
      require(z.motorTorque==z.limitedDemand);
    }
    x=legacy.Step(0.,-5.,-5.,.001); y=reset.Step(0.,-5.,-5.,.001);z=lag.Step(0.,-5.,-5.,.001);
    require(x.motorTorque*x.motorSpeed>0.); // 실패 특성을 지워버리지 않음
    require(y.integralAfter==0. && y.motorTorque<0.); // reset만으로 lag가 없어지지 않음
    require(z.motorTorque==z.limitedDemand && z.integralAfter!=0.);
  } else if(test == "parse") {
    require(arena::ParseZeroSpeedMode("legacy")==arena::ZeroSpeedMode::Legacy);
    bool rejected=false;
    try { arena::ParseZeroSpeedMode("typo"); } catch(const std::invalid_argument&) {rejected=true;}
    require(rejected);
  } else return 2;
  std::cout << test << ": PASS\n";
}
''')
    exe = temp/'probe'
    subprocess.run(['g++', '-std=c++17', '-O2', '-I', str(ROOT/'src/arena_gazebo/include'),
                    str(harness), '-o', str(exe)], check=True, capture_output=True, text=True)
    return exe


@pytest.mark.parametrize('case', ['legacy', 'nonzero', 'passive', 'factors', 'parse'])
def test_equations(equation_executable, case):
    result = subprocess.run([str(equation_executable), case], check=True, capture_output=True, text=True)
    assert 'PASS' in result.stdout


MODEL = '<sdf><model name="car"><link name="body"/><plugin name="arena::SingleMotorDrive"><gear_ratio>15</gear_ratio></plugin></model></sdf>'


def test_default_xml_is_byte_identical():
    assert launch._evaluation_zero_speed_mode(MODEL, 'legacy', '') == MODEL


@pytest.mark.parametrize('mode', launch.ZERO_SPEED_MODES[1:])
def test_only_explicit_element_changes(mode):
    changed = launch._evaluation_zero_speed_mode(MODEL, mode, '/tmp/evidence')
    tree = ET.fromstring(changed)
    plugin = tree.find("model/plugin")
    node = plugin.find('evaluation_zero_speed_mode')
    assert node.text == mode
    plugin.remove(node)
    assert ET.tostring(tree) == ET.tostring(ET.fromstring(MODEL))
    with pytest.raises(ValueError):
        launch._evaluation_zero_speed_mode(MODEL, mode, '')
    with pytest.raises(ValueError):
        launch._evaluation_zero_speed_mode(changed, mode, '/tmp/evidence')


def test_bad_or_missing_plugin_rejected():
    with pytest.raises(ValueError):
        launch._evaluation_zero_speed_mode(MODEL, 'typo', '/tmp/evidence')
    with pytest.raises(ValueError):
        launch._evaluation_zero_speed_mode('<sdf><model/></sdf>', 'passive_damping', '/tmp/evidence')
