"""검증된 후보 함수 본문을 복사한다. 원본/기존 대상 파일을 덮어쓰지 않는다."""
import ast
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
DEST = ROOT/'src/arena_autonomy/arena_autonomy/current_path_recheck'
SOURCES = {
    'geometry': '2026-11-11/follow_rebase_r126_v1/rebase.py',
    'parallel': '2026-11-11/guard_recheck_r139_v1/rebase.py',
    'guard': '2026-11-12/adjacent_current_r151_v1/current.py',
    'present': '2026-11-12/present_guard_current_r170_v1/current.py',
    'released': '2026-11-12/released_current_r167_v1/released.py',
    'released_parallel': '2026-11-12/released_parallel_r173_v1/current.py',
    'body_gap': '2026-11-11/body_nonreturn_r136_v1/candidate.py',
    'adjacent_gap': '2026-11-12/adjacent_mount_gap_r149_v1/candidate.py',
}


def functions(label, names):
    text = (ROOT/'artifacts/validation'/SOURCES[label]).read_text(encoding='utf8')
    defs = {n.name: ast.get_source_segment(text, n) for n in ast.parse(text).body
            if isinstance(n, ast.FunctionDef)}
    return '\n\n\n'.join(defs[name] for name in names)


def write(name, imports, body, origins):
    header = ('"""현재 입력의 경로 재검사. 명시 선택용 순수 함수이며 상태 전이/발행은 없다.\n\n'
              + '원 함수: ' + ', '.join(SOURCES[x] for x in origins) + '\n'
              + '이관/동등성 근거: current_recheck_module_r176_v1.\n"""\n')
    path = DEST/name
    with path.open('x', encoding='utf8', newline='\n') as stream:
        stream.write(header+imports.strip()+'\n\n\n'+body+'\n')


def main():
    DEST.mkdir(exist_ok=False)
    write('_geometry.py', '''
import math
import numpy as np
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import pursue_path
from arena_autonomy.local_path import braking_speed, swept_limit
from arena_autonomy.occupancy_timing import per_ray_padding
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points
''', functions('geometry', ['finite', 'rebase']), ['geometry'])
    write('_parallel.py', '''
import numpy as np
from arena_autonomy.parallel_lane_trial import lateral_excursion
from . import _geometry as _module
''', functions('parallel', ['supported', 'rebase']), ['parallel'])
    write('_gaps.py', '''
import math
import numpy as np
from arena_autonomy.observation_support import source_valid
from arena_autonomy.rear_fringe_trial import _bounded_nonreturn, _query_brackets
from arena_autonomy.known_structure import sector_rays
from arena_autonomy.scan_input import scan_angles
''', functions('body_gap', ['bounded_body_nonreturn'])+'\n\n\n'
       + functions('adjacent_gap', ['bounded_adjacent_structure']).replace(
           'body.bounded_body_nonreturn(', 'bounded_body_nonreturn('), ['body_gap', 'adjacent_gap'])
    write('_guard.py', '''
import math
import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.known_structure import classify, parse_sectors
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from arena_autonomy.scan_input import audit_scan_front
from . import _geometry as geometry
from . import _gaps as body
from . import _gaps as adjacent
''', functions('guard', ['supported', 'current_forward_coverage', 'rebase']), ['guard'])
    write('_present.py', '''
import numpy as np
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from arena_autonomy.local_follow import forward_planning_speed
from arena_autonomy.rear_fringe_trial import bounded_rear_fringe
from . import _guard as previous
from . import _geometry as geometry
''', functions('present', ['supported', 'rebase']), ['present'])
    write('_released.py', '''
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from . import _geometry as base
from . import _guard as coverage
''', functions('released', ['supported', 'rebase']), ['released'])
    write('_released_parallel.py', '''
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.follow_trace import thaw
from arena_autonomy.measurement_scan import MeasurementScan
from . import _parallel as parallel
from . import _released as released
''', functions('released_parallel', ['surrogate', 'supported', 'rebase']), ['released_parallel'])
    (A/'source_manifest.json').write_text(json.dumps({
        'originals': {SOURCES[k]: hashlib.sha256((ROOT/'artifacts/validation'/SOURCES[k]).read_bytes()).hexdigest()
                      for k in SOURCES},
        'extracted': {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
                      for p in sorted(DEST.glob('*.py'))},
        'replacement': 'adjacent body alias becomes direct call; other function bodies unchanged',
    }, indent=2)+'\n', encoding='utf8')


if __name__ == '__main__':
    main()
