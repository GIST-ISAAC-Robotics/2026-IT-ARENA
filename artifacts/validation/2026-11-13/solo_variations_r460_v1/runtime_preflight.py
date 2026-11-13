"""후속 WSL 반환 후 실행할 import/연결 확인. ROS init/spin/발행/Gazebo 시작 없음."""
import importlib.util
import os
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT/'scripts'), str(ROOT/'tests')]
from prepare_execution import CASES, verify_connection, nominal_success
from variation_launch import write_new


def main():
    verify_connection()
    nominal_success()
    import solo_runner
    import arena_autonomy.g1_g2_node
    from run import configure
    original = solo_runner.obstacle_polygons
    original_eval = solo_runner.solo_evaluation.evaluate_run
    original_start = solo_runner.start_demo_process
    from launch_adapter import SELECTION
    records = []
    for name in CASES:
        os.environ['IT_ARENA_R460_VARIATION'] = name
        spec = importlib.util.spec_from_file_location('r460_preflight_'+name, A/'launch/simulation.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        description = module.generate_launch_description()
        assert description.entities
        configure(solo_runner, name, A/'runtime_preflight'/name)
        obstacles = solo_runner.obstacle_polygons(None)
        assert len(obstacles) == len(original(A/'connection_v1'/f'{name}.sdf'))+3
        solo_runner.obstacle_polygons = original
        solo_runner.solo_evaluation.evaluate_run = original_eval
        solo_runner.start_demo_process = original_start
        records.append(dict(case=name, launch_description_created=True, evaluator_obstacles=len(obstacles)))
    write_new(A/'runtime_preflight/result.json', dict(passed=True, cases=records, selection=SELECTION,
        ros_initialized=False, gazebo_started=False, nodes_started=False, control_publications=0,
        scope='imports/launch descriptions/geometry wiring only; no launch setup or physics execution'))


if __name__ == '__main__':
    main()
