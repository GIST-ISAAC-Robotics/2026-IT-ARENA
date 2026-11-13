"""r368 안에서만 파일/정확한 patch 준비. 후보 실행/장치/빌드 기능 없음."""
import ast
import csv
import difflib
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
STAGED = HERE/'staged'


def digest(data):
    return hashlib.sha256(data).hexdigest()


def staged_copy(relative, replacements):
    source = ROOT/relative
    raw = source.read_bytes()
    text = raw.decode('utf-8')
    for before, after, count in replacements:
        if text.count(before) != count:
            raise ValueError(f'base changed: {relative}: {before[:60]!r}')
        text = text.replace(before, after)
    target = STAGED/relative
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text, encoding='utf-8', newline='\n')
    return dict(path=relative, base_sha256=digest(raw), staged_sha256=digest(target.read_bytes()))


def main():
    inputs = []
    csv_path = ROOT/'artifacts/validation/2026-11-13/steering_cad_fusion_v1/steering_table.csv'
    raw = csv_path.read_bytes()
    rows = list(csv.DictReader(raw.decode('utf-8-sig').splitlines()))
    if len(rows) != 96:
        raise ValueError('CAD row count differs')
    header = ['#pragma once', '// 원 CSV의 96행. CAD 운동이며 실물 교정표가 아니다.',
              '#include "cad_steering.hpp"', 'namespace arena_steering {',
              f'inline constexpr const char* kCadSha256="{digest(raw)}";',
              'inline constexpr CadSample kCadSamples[]={']
    for row in rows:
        triple = (math.radians(float(row['fusion_servo_deg'])),
                  float(row['left_wheel_heading_rad_left_positive']),
                  float(row['right_wheel_heading_rad_left_positive']))
        header.append('  {'+', '.join(format(value, '.17g') for value in triple)+'},')
    header += ['};', 'inline constexpr std::size_t kCadSampleCount=96;',
               '}  // namespace arena_steering', '']
    (STAGED/'src/arena_vehicle_interface/portable/cad_steering_table.hpp').write_text(
        '\n'.join(header), encoding='utf-8', newline='\n')
    (HERE/'steering_table.csv').write_bytes(raw)
    inputs.append(dict(path=str(csv_path.relative_to(ROOT)).replace('\\','/'), sha256=digest(raw)))

    changed = []
    cpp = 'src/arena_gazebo/src/single_motor_drive.cpp'
    changed.append(staged_copy(cpp, [
        ('#include "arena_gazebo/drivetrain.hpp"',
         '#include "arena_gazebo/drivetrain.hpp"\n#include "cad_steering_table.hpp"', 1),
        ('    servoGain = get("steering_gain", 0.40);',
         '    steeringModel = sdf->Get<std::string>("steering_model", "ideal_ackermann").first;\n'
         '    if (steeringModel != "ideal_ackermann" && steeringModel != "fusion_cad_2026_11_13")\n'
         '      throw std::invalid_argument("unknown steering model");\n'
         '    if (steeringModel == "fusion_cad_2026_11_13") {\n'
         '      if (std::abs(wheelbase-.128)>1e-6 || std::abs(track-.1239)>1e-6)\n'
         '        throw std::invalid_argument("CAD steering requires final geometry");\n'
         '      cadMapping = std::make_unique<arena_steering::CadMapping>(\n'
         '        arena_steering::kCadSamples,arena_steering::kCadSampleCount);\n'
         '      if (!cadMapping->valid()) throw std::invalid_argument("invalid CAD steering table");\n'
         '    }\n'
         '    servoGain = get("steering_gain", 0.40);', 1),
        ('    const auto angles = SteeringAngles(steering, wheelbase, track, wheelLimit);',
         '    auto angles = SteeringAngles(steering, wheelbase, track, wheelLimit);\n'
         '    if (cadMapping) {\n'
         '      cadGoal = cadMapping->central(steering, wheelbase, track, wheelLimit);\n'
         '      if (!cadGoal.valid) throw std::runtime_error("invalid CAD steering target");\n'
         '      angles = {cadGoal.left_rad, cadGoal.right_rad};\n'
         '    }', 1),
        ('    std::ostringstream json;\n    json << std::setprecision(12)',
         '    std::ostringstream json;\n    json << std::setprecision(12)', 1),
        ('      << ",\\\"watchdog_stop\\\":" << (stale ? "true" : "false")',
         '      << ",\\\"watchdog_stop\\\":" << (stale ? "true" : "false")', 1),
        ('  std::unique_ptr<SingleMotor> motor;',
         '  std::unique_ptr<SingleMotor> motor;\n'
         '  std::string steeringModel="ideal_ackermann";\n'
         '  std::unique_ptr<arena_steering::CadMapping> cadMapping;\n'
         '  arena_steering::CadGoal cadGoal;', 1),
    ]))
    # 기존 진단 JSON의 종료 직전에 선택형 기하 출처/목표를 추가한다.
    path = STAGED/cpp
    text = path.read_text(encoding='utf-8')
    anchor = 'telemetry.Publish('
    if text.count(anchor) != 1:
        raise ValueError('telemetry publish anchor changed')
    # 문자열 스트림 완료 형식을 별도 검사하여 삽입한다.
    before = '      << ",\\\"truth_yaw_rad\\\":" << worldPose.Rot().Yaw()'
    after = before + '\n      << ",\\\"steering_model\\\":\\\"" << steeringModel << "\\\""\n' \
        '      << ",\\\"cad_table_sha256\\\":\\\"" << (cadMapping?arena_steering::kCadSha256:"") << "\\\""\n' \
        '      << ",\\\"cad_hardware_measured\\\":false"\n' \
        '      << ",\\\"cad_relative_target_rad\\\":" << cadGoal.servo_rad\n' \
        '      << ",\\\"cad_equivalent_target_rad\\\":" << cadGoal.equivalent_rad\n' \
        '      << ",\\\"cad_target_clamped\\\":" << (cadGoal.clamped?"true":"false")'
    if text.count(before) != 1:
        raise ValueError('truth diagnostic anchor changed')
    path.write_text(text.replace(before,after), encoding='utf-8', newline='\n')
    changed[-1]['staged_sha256'] = digest(path.read_bytes())

    changed.append(staged_copy('src/arena_gazebo/CMakeLists.txt', [
        ('target_include_directories(arena-single-motor-system PRIVATE include)',
         'target_include_directories(arena-single-motor-system PRIVATE include ../arena_vehicle_interface/portable)', 1),
        ('  add_test(NAME single_motor_equations COMMAND test_drivetrain)',
         '  add_test(NAME single_motor_equations COMMAND test_drivetrain)\n'
         '  add_executable(test_cad_steering test/test_cad_steering.cpp)\n'
         '  target_compile_features(test_cad_steering PRIVATE cxx_std_17)\n'
         '  target_include_directories(test_cad_steering PRIVATE include ../arena_vehicle_interface/portable)\n'
         '  add_test(NAME cad_steering_mapping COMMAND test_cad_steering)', 1),
    ]))
    changed.append(staged_copy('src/arena_description/models/arena_car/model.sdf.xacro', [
        ('  <xacro:arg name="drive_mode" default="single_motor"/>',
         '  <xacro:arg name="drive_mode" default="single_motor"/>\n'
         '  <xacro:arg name="steering_model" default="ideal_ackermann"/>', 1),
        ('    <plugin filename="arena-single-motor-system" name="arena::SingleMotorDrive">',
         '    <plugin filename="arena-single-motor-system" name="arena::SingleMotorDrive">\n'
         '      <steering_model>$(arg steering_model)</steering_model>', 1),
    ]))
    changed.append(staged_copy('src/arena_bringup/launch/simulation.launch.py', [
        ('            "max_steering_angle": str(drivetrain["max_steering_angle_rad"]),',
         '            "max_steering_angle": str(drivetrain["max_steering_angle_rad"]),\n'
         '            "steering_model": str(drivetrain.get("simulation_steering_model", "ideal_ackermann")),', 1),
    ]))
    # staging에서도 동일 package import를 보장하는 미변경 문맥 파일. patch에는 제외한다.
    staged_copy('src/arena_vehicle_interface/arena_vehicle_interface/__init__.py', [])
    sources = [str(p.relative_to(STAGED)).replace('\\','/') for p in sorted(STAGED.rglob('*')) if p.is_file()]
    patches = []
    for relative in sources:
        path = STAGED/relative
        original = ROOT/relative
        old = original.read_text(encoding='utf-8') if original.exists() else ''
        new = path.read_text(encoding='utf-8')
        if old == new:
            continue
        patches.extend(difflib.unified_diff(old.splitlines(keepends=True), new.splitlines(keepends=True),
                        fromfile='a/'+relative if original.exists() else '/dev/null', tofile='b/'+relative))
    (HERE/'proposed.patch').write_text(''.join(patches), encoding='utf-8', newline='\n')
    py_checked = []
    for path in sorted(HERE.rglob('*.py')):
        ast.parse(path.read_text(encoding='utf-8'), filename=str(path))
        py_checked.append(str(path.relative_to(HERE)).replace('\\','/'))
    ET.parse(STAGED/'src/arena_description/models/arena_car/model.sdf.xacro')
    freeze = dict(schema='r368_staged_patch_v1', product_files_modified=False, inputs=inputs,
                  modified_base_files=changed,
                  staging=[dict(path=p, sha256=digest((STAGED/p).read_bytes())) for p in sources],
                  patch_sha256=digest((HERE/'proposed.patch').read_bytes()),
                  validation=dict(python_ast=py_checked,xacro_xml_parse=True,
                                  unit_tests_run=False,cpp_compiled=False,simulation_run=False,hardware_access=False))
    (HERE/'source_manifest.json').write_text(json.dumps(freeze, ensure_ascii=False, indent=2)+'\n',encoding='utf-8')
    print(json.dumps(dict(staged_files=len(sources), python_ast=len(py_checked), xml=True,
                          product_files_modified=False, tests_run=False)))


if __name__ == '__main__':
    main()
