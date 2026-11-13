"""합성 장착 프로필을 CLI로 내보내고 C++/Python 목표를 대조한다. 장치 I/O 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_vehicle_interface.steering_calibration import (
    CadMapping, cad_servo_goal, complete_mount_profile, record_zero, save_new_profile,
)

CPP = r'''
#include <iostream>
#include <iomanip>
#include <cstring>
#include "mounted.hpp"
#include "cad_servo_goal.hpp"
#include "cad_steering_table.hpp"
int main() {
  using namespace arena_steering;
  if(std::strcmp(kCadSha256,arena_steering_mount::kCadTableSha256))return 2;
  const auto zero=arena_steering_mount::Profile();
  const CadMapping map(kCadSamples,kCadSampleCount);
  const double pi=std::acos(-1.0);
  const double limit=std::min(std::abs(map.central(1,.128,.1239,pi/6).equivalent_rad),
                            std::abs(map.central(-1,.128,.1239,pi/6).equivalent_rad));
  if(!zero.valid() || !map.valid())return 3;
  std::cout<<std::setprecision(17);
  for(int i=-100;i<=100;++i) {
    const double angle=limit*i/100;
    const auto goal=ResolveCadServoGoal(map,zero,angle,limit,.128,.1239,pi/6);
    if(!goal.valid)return 4;
    const auto actual=map.relative(goal.servo.relative_rad);
    std::cout<<i<<' '<<goal.servo.position<<' '<<actual.left_rad<<' '
             <<actual.right_rad<<' '<<actual.equivalent_rad<<'\n';
  }
  if(ResolveCadServoGoal(map,zero,limit+.001,limit,.128,.1239,pi/6).valid)return 5;
  return 0;
}
'''


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    out=args.out.resolve()
    out.mkdir(parents=True,exist_ok=False)
    paths=[Path(__file__).resolve(), ROOT/'scripts/steering_zero_profile.py',
           ROOT/'tests/test_steering_calibration.py']
    paths+=list((ROOT/'src/arena_vehicle_interface/portable').glob('*steer*.hpp'))
    paths+=[ROOT/'src/arena_vehicle_interface/portable/cad_servo_goal.hpp']
    paths+=list((ROOT/'src/arena_vehicle_interface/arena_vehicle_interface').glob('steering_*.py'))
    before={str(p.relative_to(ROOT)):sha(p) for p in paths}
    with zipfile.ZipFile(out/'tested_sources.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for p in paths: archive.write(p,p.relative_to(ROOT))
    result=dict(scope='synthetic mounted profile export; no device or firmware output',
                output_enabled=False, firmware_uploaded=False, sources_before=before,
                checked_goals=0, maximum_radian_difference=0., profiles=[])
    try:
        unit=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests',
                             '-p','test_steering_calibration.py','-v'],cwd=ROOT,
                            text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
        (out/'unit.log').write_text(unit.stdout,encoding='utf-8')
        if unit.returncode: raise RuntimeError('Python unit tests failed')
        limit=CadMapping.default().command_limit(wheelbase=.128,track=.1239,wheel_limit=math.pi/6)
        for direction in (-1,1):
            folder=out/f'direction_{direction}'
            folder.mkdir()
            profile=record_zero(2137,assembly_id='SYNTHETIC-NOT-HARDWARE',straight_confirmed=True,
                                evidence_note='synthetic compiler verification only')
            profile=complete_mount_profile(profile,direction=direction,minimum=1600,
                                           maximum=2700,confirmed=True)
            save_new_profile(folder/'profile.json',profile)
            commands=[
                [sys.executable,str(ROOT/'scripts/steering_zero_profile.py'),'export-cpp',
                 str(folder/'profile.json'),'--assembly-id','SYNTHETIC-NOT-HARDWARE',
                 '--output',str(folder/'mounted.hpp')],
                ['g++','-std=c++17','-O2','-Wall','-Wextra','-Werror',
                 '-I',str(ROOT/'src/arena_vehicle_interface/portable'),
                 str(folder/'main.cpp'),'-o',str(folder/'verify')],
                [str(folder/'verify')],
            ]
            (folder/'main.cpp').write_text(CPP,encoding='ascii')
            outputs=[]
            for n,command in enumerate(commands):
                run=subprocess.run(command,cwd=ROOT,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
                (folder/f'step_{n}.log').write_text(run.stdout,encoding='utf-8')
                if run.returncode: raise RuntimeError(f'direction {direction} step {n} exit {run.returncode}')
                outputs.append(run.stdout)
            rows=outputs[-1].splitlines()
            if len(rows)!=201: raise RuntimeError(f'expected 201 goals, got {len(rows)}')
            for expected_i,line in zip(range(-100,101),rows):
                i,position,left,right,central=line.split()
                if int(i)!=expected_i: raise RuntimeError('sample order mismatch')
                expected=cad_servo_goal(profile,limit*expected_i/100)
                if int(position)!=expected['position']: raise RuntimeError(f'count mismatch {direction} {i}')
                error=max(abs(float(value)-expected[key]) for value,key in zip(
                    [left,right,central],['left_target_rad','right_target_rad','central_realized_rad']))
                if error>1e-11: raise RuntimeError(f'wheel angle mismatch {direction} {i}: {error}')
                result['maximum_radian_difference']=max(result['maximum_radian_difference'],error)
                result['checked_goals']+=1
            result['profiles'].append(dict(direction=direction,zero_count=2137,
                header_sha256=sha(folder/'mounted.hpp'),export=json.loads(outputs[0])))
        result['sources_unchanged']=before=={str(p.relative_to(ROOT)):sha(p) for p in paths}
        if not result['sources_unchanged']: raise RuntimeError('source changed during test')
        result['passed']=True
    except BaseException as exc:
        result.update(passed=False,error=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
        print(json.dumps({k:v for k,v in result.items() if k not in ('sources_before','profiles')}))


if __name__=='__main__': main()
