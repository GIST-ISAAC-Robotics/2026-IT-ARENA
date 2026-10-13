"""설치 버전에 대응하는 공개 1차 소스만 내려받고 출처/해시를 보존한다."""
import hashlib
import json
from pathlib import Path
import urllib.request

BASE = Path(__file__).resolve().parent
SOURCES = {
    'WheelSlip.cc': ('gazebosim/gz-sim', 'gz-sim8_8.11.0', 'src/systems/wheel_slip/WheelSlip.cc'),
    'Physics.cc': ('gazebosim/gz-sim', 'gz-sim8_8.11.0', 'src/systems/physics/Physics.cc'),
    'SDFFeatures.cc': ('gazebosim/gz-physics', 'gz-physics7_7.6.0', 'dartsim/src/SDFFeatures.cc'),
    'ShapeFeatures.cc': ('gazebosim/gz-physics', 'gz-physics7_7.6.0', 'dartsim/src/ShapeFeatures.cc'),
    'ContactConstraint.cpp': ('dartsim/dart', 'v6.13.2', 'dart/constraint/ContactConstraint.cpp'),
    'ContactSurface.cpp': ('dartsim/dart', 'v6.13.2', 'dart/constraint/ContactSurface.cpp'),
    'BoxedLcpConstraintSolver.cpp': ('dartsim/dart', 'v6.13.2', 'dart/constraint/BoxedLcpConstraintSolver.cpp'),
    'DantzigBoxedLcpSolver.cpp': ('dartsim/dart', 'v6.13.2', 'dart/constraint/DantzigBoxedLcpSolver.cpp'),
    'lcp.cpp': ('dartsim/dart', 'v6.13.2', 'dart/external/odelcpsolver/lcp.cpp'),
    'vendor_CMakeLists.txt': ('gazebo-release/gz_dartsim_vendor', '0.0.3', 'CMakeLists.txt'),
}

if __name__ == '__main__':
    target = BASE/'upstream'
    target.mkdir(exist_ok=False)
    result = {}
    for name, (repo, tag, source) in SOURCES.items():
        url = f'https://raw.githubusercontent.com/{repo}/{tag}/{source}'
        with urllib.request.urlopen(url, timeout=45) as response:
            data = response.read()
        with (target/name).open('xb') as stream:
            stream.write(data)
        result[name] = {'url': url, 'version_tag': tag, 'sha256': hashlib.sha256(data).hexdigest()}
    with (BASE/'upstream_sources.json').open('x') as stream:
        json.dump(result, stream, indent=2)
    print('CAPTURED', len(result), 'version-pinned primary source files')
