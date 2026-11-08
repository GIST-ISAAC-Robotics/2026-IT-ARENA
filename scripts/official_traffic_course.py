"""공식 2×5 점등 팀 파생 실행 월드를 쓰는 교통 시험의 명시 입력. 11/1 fixture 경로와 섞지 않는다.

기본은 v2026.11.01 파생 월드(track=official)다. 다른 world/track은 호출자가 명시하며, 시나리오 설정도
같은 track으로 등록돼 있어야 한다. 옛 s좌표 각본을 새 코스로 옮겨 같은 의미로 보지 않는다.
신호 상태와 상대 정답은 평가기에만 전달하고 제어기는 RGB로 출발한다.
"""
from pathlib import Path

from prelap_signal_eval import ROOT, digest, sequence_evidence

DEFAULT_TRACK = 'official'
WORLD = ROOT/'src/arena_gazebo/worlds/it_arena_official'
FACILITY_FILES = (ROOT/'scripts/official_2x5_facilities.py',
                  ROOT/'src/arena_gazebo/scripts/traffic_light_controller.py',
                  ROOT/'src/arena_gazebo/src/visual_material_client.cpp', Path(__file__).resolve())
ENTRY_FILES = (WORLD/'world.sdf', WORLD/'scene.json', WORLD/'centerline.csv', *FACILITY_FILES)


def entry_files(world=WORLD):
    return (world/'world.sdf', world/'scene.json', world/'centerline.csv', *FACILITY_FILES)


def inputs(args, config, case, world=WORLD, track=DEFAULT_TRACK):
    """선정 교통 구성만 허용한다. 신호 등록 코스의 차량·센서 조건을 조용히 바꾸지 않는다."""
    if (args.mode != 'g1g2' or args.handoff_reserve != 'paced' or not args.handoff_yield_progress
            or not args.handoff_rear_sweep or not args.record_follow_decisions or args.seed != 0
            or config['launch']['speed_profile'] != 'g1_open_1p5' or config['launch']['grid_slot'] not in (0, 3)):
        raise ValueError('공식 교통 경로는 선정 g1g2 paced 구성·후열 신호 칸만 지원한다')
    registered = config.get('track', DEFAULT_TRACK)
    if registered != track:
        raise ValueError(f'교통 설정은 track={registered} 기준이다. {track}에서는 장면·s좌표·판정을 재등록해야 한다')
    extras = case.get('additional_actors')
    selected_multi = config.get('official_multi_actor_trial', False)
    if type(selected_multi) is not bool:
        raise ValueError('공식 복수 상대 선택은 bool이어야 한다')
    if extras or selected_multi:
        if (not selected_multi or track != 'official_v2026_11_06'
                or not getattr(args, 'actor_overlay', None)
                or not isinstance(extras, list) or not 1 <= len(extras) <= 3
                or not case.get('additional_actor_scope')):
            raise ValueError('공식 복수 상대는 새 코스의 명시 등록·overlay·범위가 필요하다')
        # 이후 기존 actor_specs/check_additional_routes 및 ExtraActors가 각 차량의
        # 기하·일정·원시 수신·접촉·평가 전용 입력을 각각 검증한다.
    return {path.relative_to(ROOT).as_posix(): digest(path) for path in entry_files(world)}


def signal(output, labels, trace, scene):
    return sequence_evidence(output, labels, [dict(time=row['t'], autonomy=row['autonomy']) for row in trace],
                             scene['traffic_light']['lamp_channels'])
