#!/usr/bin/env python3
"""고정 조건의 평가 전용 Dantzig/PGS 대조. 기본 launch/probe/구동 소스는 변경하지 않는다.

기존 probe의 flat_world 생성 함수만 이 프로세스 안에서 감싼다. 생성된 시험 사본에
solver_type 하나만 넣고, 나머지 입력은 10/12 예약 노면 대조와 같게 고정한다.
실제 선택 확인은 gz-physics SetWorldSolver의 getBoxedLcpSolver()->getType() 로그를 쓴다.
이는 primary solver 확인이며 매 step의 secondary fallback 호출 계측은 아니다.
"""
import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOLVERS = {'dantzig': 'DantzigBoxedLcpSolver', 'pgs': 'PgsBoxedLcpSolver'}


def configure_solver(path, solver):
    if solver not in SOLVERS:
        raise ValueError('unregistered solver')
    tree = ET.parse(path)
    worlds = tree.getroot().findall('world')
    if len(worlds) != 1 or len(worlds[0].findall('physics')) != 1:
        raise ValueError('expected one world/physics')
    physics = worlds[0].find('physics')
    if physics.findtext('max_step_size') != '0.001':
        raise ValueError('registered comparison requires unchanged 1 ms')
    if len(physics.findall('dart')) > 1:
        raise ValueError('ambiguous dart settings')
    dart = physics.find('dart')
    if dart is None:
        dart = ET.SubElement(physics, 'dart')
    if dart.find('solver') is not None:
        raise ValueError('refuse overriding a preexisting solver setting')
    ET.SubElement(ET.SubElement(dart, 'solver'), 'solver_type').text = solver
    tree.write(path, encoding='utf-8', xml_declaration=True)


def probe_arguments(output):
    return ['--output', str(output), '--surface', 'flat', '--zero-speed-mode', 'passive_damping',
            '--command-schedule', 'sim_grid', '--schedule-phase-ms', '10', '--command-profile',
            str(ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json'),
            '--plugin-command-trace', '--evaluation-gc', 'defer',
            '--evaluation-startup', 'embedded_paused', '--evaluation-scheduled-apply']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--solver', choices=tuple(SOLVERS), required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output.exists():
        raise ValueError('new artifacts directory required')
    import probe_braking_contact as probe
    original_flat_world = probe.flat_world

    def with_solver(source, destination):
        count = original_flat_world(source, destination)
        configure_solver(destination, args.solver)
        return count

    # 실행 소스 snapshot에도 wrapper를 포함한다. 기존 probe 자체의 SHA는 유지한다.
    probe.SOURCE_FILES = [*probe.SOURCE_FILES, Path(__file__).relative_to(ROOT).as_posix()]
    probe.flat_world = with_solver
    sys.argv = [str(ROOT/'scripts/probe_braking_contact.py'), *probe_arguments(output)]
    return probe.main()


if __name__ == '__main__':
    raise SystemExit(main())
