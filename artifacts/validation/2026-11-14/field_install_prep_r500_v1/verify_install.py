"""Inspect frozen installation and help entrypoints without starting ROS/device IO."""
import hashlib
import importlib
import importlib.metadata
import json
import os
from pathlib import Path
import platform
import subprocess
import sys

ROOT = Path('/home/q/it-arena-field-r500-20261114-a')
hash_file = lambda p: hashlib.sha256(Path(p).read_bytes()).hexdigest()
manifest = json.loads((ROOT/'source_manifest.json').read_text())
result = {'hostname': platform.node(), 'architecture': platform.machine(),
          'python': sys.version, 'root': str(ROOT), 'pid': os.getpid(),
          'source_count': len(manifest['files']), 'source_mismatch': [],
          'installed_modules': {}, 'entrypoints': {}, 'dependencies': {},
          'cli_help': {}, 'installed_values': manifest['installed_values'],
          'device_io': False, 'ros_nodes_started': False, 'arm_or_drive': False,
          'firmware_written': False, 'benchmark_run': False}
for name, expected in manifest['files'].items():
    if hash_file(ROOT/name) != expected:
        result['source_mismatch'].append(name)
    fields = name.split('/')
    if len(fields) >= 4 and fields[0] == 'src' and fields[1] in ('arena_autonomy', 'arena_vehicle_interface') and fields[2] == fields[1] and name.endswith('.py'):
        target = ROOT/'install'/fields[1]/'lib/python3.10/site-packages'/Path(*fields[2:])
        result['installed_modules'][name] = {'path': str(target), 'sha256': hash_file(target), 'matches': hash_file(target) == expected}
for name in ('rclpy', 'ackermann_msgs.msg', 'sensor_msgs.msg', 'geometry_msgs.msg', 'std_srvs.srv', 'numpy', 'cv2', 'yaml', 'serial', 'gi'):
    try:
        module = importlib.import_module(name)
        result['dependencies'][name] = {'ok': True, 'path': str(getattr(module, '__file__', 'built-in'))}
    except Exception as error:
        result['dependencies'][name] = {'ok': False, 'error': repr(error)}
for package in ('arena_autonomy', 'arena_vehicle_interface'):
    dist = importlib.metadata.distribution(package)
    for entry in dist.entry_points:
        if entry.group != 'console_scripts':
            continue
        try:
            entry.load()
            module = importlib.import_module(entry.module)
            path = Path(module.__file__).resolve()
            result['entrypoints'][package+'/'+entry.name] = {'ok': path.is_relative_to(ROOT/'install'), 'module': entry.value, 'path': str(path), 'sha256': hash_file(path)}
        except Exception as error:
            result['entrypoints'][package+'/'+entry.name] = {'ok': False, 'error': repr(error)}
from arena_autonomy import _numeric_kernels as native
result['native'] = {'path': str(Path(native.__file__).resolve()), 'sha256': hash_file(native.__file__), 'abi': native.ABI_VERSION,
                    'functions': {name: hasattr(native, name) for name in ('depth', 'masked', 'classes', 'polygon', 'current_mask_occupied')}}
for name in ('run_runtime_actuation_host.py', 'steering_zero_profile.py', 'runtime_installed_profile.py', 'runtime_installed_limits.py', 'race_operator.py', 'development_jog.py', 'run_hardware_sensor_preview.py', 'bridge_c1_native_preview.py', 'bridge_b0390_preview.py'):
    completed = subprocess.run([sys.executable, '-B', str(ROOT/'scripts'/name), '--help'], capture_output=True, text=True, timeout=20)
    result['cli_help'][name] = {'exit': completed.returncode, 'stderr': completed.stderr[-2000:], 'sha256': hash_file(ROOT/'scripts'/name)}
result['ros_executables'] = {}
for package in ('arena_autonomy', 'arena_vehicle_interface'):
    completed = subprocess.run(['ros2', 'pkg', 'executables', package], capture_output=True, text=True, timeout=20)
    result['ros_executables'][package] = {'exit': completed.returncode, 'lines': completed.stdout.splitlines(), 'stderr': completed.stderr}
log = (ROOT/'overlay_build.log').read_text()
result['build'] = {'exit': int((ROOT/'overlay_build.exit').read_text()), 'pid': int((ROOT/'overlay_build.pid').read_text()),
                   'compiler_commands': [line for line in log.splitlines() if line.startswith(('aarch64', 'gcc ', 'g++ ', 'c++ '))]}
result['ok'] = (not result['source_mismatch'] and all(v['matches'] for v in result['installed_modules'].values())
                and all(v['ok'] for v in result['dependencies'].values()) and all(v['ok'] for v in result['entrypoints'].values())
                and all(v['exit'] == 0 for v in result['cli_help'].values()) and all(v['exit'] == 0 for v in result['ros_executables'].values())
                and native.ABI_VERSION == 1 and all(result['native']['functions'].values())
                and result['native']['sha256'] == '5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff')
(ROOT/'install_verification.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({key: value for key, value in result.items() if key not in ('installed_modules', 'dependencies', 'entrypoints')}, indent=2))
print('installed_module_count', len(result['installed_modules']), 'entrypoint_count', len(result['entrypoints']))
print('failures', {key: value for key, value in result['entrypoints'].items() if not value['ok']})
assert result['ok']
