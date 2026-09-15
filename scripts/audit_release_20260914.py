#!/usr/bin/env python3
"""9/14 보존 ZIP 대조와 격리된 Gazebo 엄격 파싱·200스텝 검사. WSL용."""
import json
import os
from pathlib import Path
import re
import subprocess
from zipfile import ZipFile

from build_official_track import ARCHIVE, DESTINATION, REPO, sha256, verify_archive

OUT = REPO / 'artifacts/validation/2026-09-15/official_update'
WORK = REPO / 'build/official_update_20260915'


def main():
    archive = verify_archive()
    raw = WORK / 'fixtures/raw'
    raw.mkdir(parents=True, exist_ok=False)
    with ZipFile(ARCHIVE) as current:
        current.extractall(raw)
        now = {x.filename: current.read(x) for x in current.infolist() if not x.is_dir()}
    old_path = REPO / 'assets/track/official/v2026.09.02/it_arena_track_v2026.09.02.zip'
    with ZipFile(old_path) as old:
        before = {x.filename: old.read(x) for x in old.infolist() if not x.is_dir()}
    report = {'archive': archive, 'unchanged': sorted(k for k in now.keys() & before.keys() if now[k] == before[k]),
              'changed': sorted(k for k in now.keys() & before.keys() if now[k] != before[k]),
              'added': sorted(now.keys() - before.keys()), 'removed': sorted(before.keys() - now.keys()),
              'checks': {}}
    config = WORK / 'physics.config'
    config.write_text('''<server_config><plugins>
      <plugin entity_name="*" entity_type="world" filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin entity_name="*" entity_type="world" filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    </plugins></server_config>''', encoding='utf-8')
    env = dict(os.environ, GZ_PARTITION=f'arena_release_load_{os.getpid()}', GZ_SIM_SERVER_CONFIG_PATH=str(config),
               LIBGL_ALWAYS_SOFTWARE='true')
    for case, source in [('raw', raw / 'output_final'), ('runtime', DESTINATION)]:
        results = {}
        for name, cmd in [('strict_sdf', ['gz', 'sdf', '-k', str(source / 'world.sdf')]),
                          ('physics_200_steps', ['gz', 'sim', '-s', '-r', '--iterations', '200', '-v', '4', str(source / 'world.sdf')])]:
            result = subprocess.run(cmd, cwd=source, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=90)
            log = result.stdout.decode('utf-8', errors='replace')
            (OUT / f'{case}_{name}.log').write_text(log, encoding='utf-8')
            clean = re.sub(r'\x1b\[[0-9;]*m', '', log)
            errors = [line for line in clean.splitlines() if '[Err]' in line or 'Error Code' in line or '[Wrn]' in line]
            results[name] = {'command': cmd, 'return_code': result.returncode, 'errors_warnings': errors,
                             'passed': result.returncode == 0 and not errors,
                             'iteration_limit_requested': 200 if name.startswith('physics') else None}
        report['checks'][case] = results
    report['all_checks_pass'] = all(r['passed'] for c in report['checks'].values() for r in c.values())
    (OUT / 'release_audit.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['all_checks_pass'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
