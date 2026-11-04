"""공식 v2026.11.01 실행 월드·2×5 점등을 쓰는 교통 시험의 명시 입력. 11/1 fixture 경로와 섞지 않는다.

신호 상태와 상대 정답은 평가기에만 전달하고 제어기는 RGB로 출발한다.
"""
from pathlib import Path

from prelap_signal_eval import ROOT, digest, sequence_evidence

WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
ENTRY_FILES = (WORLD/'world.sdf', WORLD/'scene.json', WORLD/'centerline.csv', ROOT/'scripts/official_2x5_facilities.py',
               ROOT/'src/arena_gazebo/scripts/traffic_light_controller.py',
               ROOT/'src/arena_gazebo/src/visual_material_client.cpp', Path(__file__).resolve())


def inputs(args, config, case):
    """선정 교통 구성만 허용한다. 신호 등록 코스의 차량·센서 조건을 조용히 바꾸지 않는다."""
    if (args.mode != 'g1g2' or args.handoff_reserve != 'paced' or not args.handoff_yield_progress
            or not args.handoff_rear_sweep or not args.record_follow_decisions or args.seed != 0
            or config['launch']['speed_profile'] != 'g1_open_1p5' or config['launch']['grid_slot'] not in (0, 3)
            or case.get('additional_actors')):
        raise ValueError('공식 교통 경로는 선정 g1g2 paced 구성·후열 신호 칸·단일 등록 상대만 지원한다')
    return {path.relative_to(ROOT).as_posix(): digest(path) for path in ENTRY_FILES}


def signal(output, labels, trace, scene):
    return sequence_evidence(output, labels, [dict(time=row['t'], autonomy=row['autonomy']) for row in trace],
                             scene['traffic_light']['lamp_channels'])
