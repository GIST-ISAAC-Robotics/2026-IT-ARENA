from pathlib import Path
import shutil

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
relative = Path('scripts/runtime_installed_limits.py')
for part in ('base', 'stage'):
    target = A/part/relative
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(ROOT/relative, target)
p = A/'stage'/relative
s = p.read_text(encoding='utf-8')
needle = "        value[node]['ros__parameters'][PARAMETER] = limit\n    return value"
replacement = """        value[node]['ros__parameters'][PARAMETER] = limit
    # r401 계열 base는 이 선택값이 planner 아래에만 있었다. 새 설치 파생본에만
    # 동일한 명시 구성을 기록하며, 독립 guard는 planner 결과를 입력받지 않는다.
    planner = value['local_pursuit']['ros__parameters']
    guard = value['lidar_safety']['ros__parameters']
    if (planner.get('g1_guard_observation_policy') == 'selected_r181'
            and planner.get('g1_traffic_recovery_trial') is True
            and guard.get('g1_traffic_recovery_trial') is True):
        guard['g1_guard_observation_policy'] = 'selected_r181'
    return value"""
assert s.count(needle) == 1
s = s.replace(needle, replacement).replace('runtime YAML differs beyond the two installed steering limits',
    'runtime YAML differs from installed limits and explicit guard policy selection')
p.write_text(s, encoding='utf-8', newline='\n')
p = A/'stage/src/arena_autonomy/arena_autonomy/lidar_safety.py'
s = p.read_text(encoding='utf-8')
start = s.index("            if getattr(self, 'command_observation_priority', False):\n                # planner")
stop = s.index('            cap = braking_speed', start)
s = s[:start] + """            clearance = swept_limit(points, output.drive.steering_angle, self.wheelbase,
                                    length=self.length, width=self.width)
""" + s[stop:]
p.write_text(s, encoding='utf-8', newline='\n')
p = A/'stage/src/arena_autonomy/arena_autonomy/independent_guard_route.py'
s = p.read_text(encoding='utf-8').replace('def front_evidence(scan, yaw):', '''def front_evidence(scan, yaw):
    # lidar_safety.on_scan의 mask_scan을 지난 입력이다. CAD 거리 bands 안의 자기
    # 반환은 이미 unknown이다. 각도 sector 전체를 다시 제외하면 같은 각도의
    # 더 먼 외부 반환까지 지워지므로 여기서는 현재 남아 있는 반환만 센다.''')
p.write_text(s, encoding='utf-8', newline='\n')
p = A/'stage/src/arena_autonomy/arena_autonomy/partial_observation.py'
s = p.read_text(encoding='utf-8').replace('현재 반환/두 명령의 점유·제동은 호출 guard가 검사한다.',
    '현재 반환/명령의 점유·제동은 호출 guard가 검사한다.')
p.write_text(s, encoding='utf-8', newline='\n')
print('stage refined; common unchanged')
