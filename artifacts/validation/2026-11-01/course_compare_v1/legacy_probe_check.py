"""test_equations[legacy] 실패 분류: 같은 매개변수를 주면 과거·현재 구동식이 같은지 임시 폴더에서 본다.

저장소 파일은 바꾸지 않는다. 시험은 양쪽을 각자 기본값으로 만들기 때문에 기본값 변경과 수식 변경을
구분하지 못한다. 여기서는 과거 기본값을 양쪽에 명시한 탐침과 시험 원본 조건을 나란히 실행한다.
"""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import zipfile

ROOT = Path(__file__).resolve().parents[4]
HEADER = 'src/arena_gazebo/include/arena_gazebo/drivetrain.hpp'
SNAPSHOT = ROOT / 'artifacts/validation/2026-10-08/braking_ab_v1/official/source_snapshot.zip'
PROBE = r'''
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
int main(int argc, char **argv) {
  const bool same = std::string(argv[1]) == "same_parameters";
  preserved_arena::DriveParameters old;
  arena::DriveParameters next;
  if (same) {
    next.radius=old.radius; next.ratio=old.ratio; next.efficiency=old.efficiency; next.torqueLimit=old.torqueLimit;
    next.brakeTorqueLimit=old.brakeTorqueLimit; next.freeSpeed=old.freeSpeed; next.responseTime=old.responseTime;
    next.speedKp=old.speedKp; next.speedKi=old.speedKi; next.maxSpeed=old.maxSpeed; next.acceleration=old.acceleration;
    next.carrierDrag=old.carrierDrag; next.differentialViscosity=old.differentialViscosity;
    next.differentialTorqueLimit=old.differentialTorqueLimit;
  }
  std::cout << "old radius=" << old.radius << " ratio=" << old.ratio << " free=" << old.freeSpeed
            << " | new radius=" << arena::DriveParameters{}.radius << " ratio=" << arena::DriveParameters{}.ratio
            << " free=" << arena::DriveParameters{}.freeSpeed << "\n";
  arena::SingleMotor now(next);
  preserved_arena::SingleMotor before(old);
  std::mt19937 rng(810);
  std::uniform_real_distribution<double> cmd(-6.,6.), vel(-100.,100.), step(.0001,.01);
  int mismatches = 0;
  for (int i=0;i<50000;++i) {
    double t = i%5 ? cmd(rng) : 0., l=vel(rng), r=vel(rng), dt=step(rng);
    auto a=now.Step(t,l,r,dt); auto b=before.Step(t,l,r,dt);
    if (!(a.motorTorque==b.motorTorque && a.motorSpeed==b.motorSpeed && a.leftTorque==b.leftTorque &&
          a.rightTorque==b.rightTorque && a.lossPower==b.lossPower && a.referenceSpeed==b.referenceSpeed &&
          a.speedError==b.speedError)) ++mismatches;
    if (i%701==0) {now.Reset();before.Reset();}
  }
  std::cout << "mismatches=" << mismatches << "\n";
  return mismatches != 0;
}
'''
with zipfile.ZipFile(SNAPSHOT) as archive:
    old = archive.read(HEADER)
result = dict(old_header_sha256=hashlib.sha256(old).hexdigest(), current_header_sha256=hashlib.sha256((ROOT / HEADER).read_bytes()).hexdigest())
with tempfile.TemporaryDirectory() as folder:
    temp = Path(folder)
    (temp / 'legacy.hpp').write_bytes(old)
    (temp / 'probe.cpp').write_text(PROBE, encoding='utf-8')
    subprocess.run(['g++', '-std=c++17', '-O1', '-I', str(temp), '-I', str(ROOT / 'src/arena_gazebo/include'),
                    str(temp / 'probe.cpp'), '-o', str(temp / 'probe')], check=True)
    for arm in ('own_defaults', 'same_parameters'):
        r = subprocess.run([str(temp / 'probe'), arm], capture_output=True, text=True)
        result[arm] = dict(returncode=r.returncode, stdout=r.stdout.strip().splitlines())
print(json.dumps(result, ensure_ascii=False, indent=1))
