#!/usr/bin/env bash
# WSL/Linux bash 파일로 환경을 구성한다. 호출 셸이 $변수를 먼저 펼치지 않게 한다.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../../../.."
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
export PYTHONPATH="/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests${PYTHONPATH:+:$PYTHONPATH}"
python3 -B - <<'PY'
from importlib.metadata import distribution
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from arena_autonomy import turn_preview
for pkg, names in [('arena-autonomy',('local_pursuit','lidar_safety')),
                   ('arena-vehicle-interface',('virtual_mcu','actuation_host','guarded_sim_actuator','rotating_lidar','sim_wheel_encoder'))]:
    dist=distribution(pkg)
    prefix=Path(get_package_prefix(pkg.replace('-','_')))
    print(pkg,str(dist._path),str(prefix))
    for name in names:
        (ep,)=[e for e in dist.entry_points if e.group=='console_scripts' and e.name==name]
        assert callable(ep.load()) and (prefix/'lib'/pkg.replace('-','_')/name).is_file(),name
print('installed packages and child process entry points resolved:',turn_preview.__file__)
PY
exec "$@"
