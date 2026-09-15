"""종료 진단에 사용한 공개 소스·이슈와 설치 버전의 읽기 전용 근거를 보존합니다."""
import hashlib
import json
from pathlib import Path
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / 'artifacts/validation/2026-09-15/official_update/shutdown_sources'
URLS = {
    'SignalHandler_5.8.0.cc': 'https://raw.githubusercontent.com/gazebosim/gz-common/gz-common5_5.8.0/src/SignalHandler.cc',
    'ServerPrivate_8.11.0.cc': 'https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.11.0/src/ServerPrivate.cc',
    'Sensors_8.11.0.cc': 'https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8_8.11.0/src/systems/sensors/Sensors.cc',
    'common_issue530_timeline.json': 'https://api.github.com/repos/gazebosim/gz-common/issues/530/timeline?per_page=100',
}


def main():
    DEST.mkdir(parents=True, exist_ok=False)
    result = {'sources': {}, 'installed_libraries': []}
    for name, url in URLS.items():
        try:
            req = urllib.request.Request(url, headers={'User-Agent': 'IT-ARENA-readonly-diagnostic'})
            with urllib.request.urlopen(req, timeout=20) as response:
                data = response.read()
            (DEST / name).write_bytes(data)
            result['sources'][name] = {'url': url, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
        except Exception as exc:
            result['sources'][name] = {'url': url, 'error': str(exc)}
    for vendor, pattern in [('gz_common_vendor', 'libgz-common5.so.*'),
                            ('gz_sim_vendor', 'libgz-sim8.so.*')]:
        for library in Path('/opt/ros/jazzy/opt', vendor, 'lib').glob(pattern):
            if not library.is_symlink():
                result['installed_libraries'].append({'file': str(library),
                    'sha256': hashlib.sha256(library.read_bytes()).hexdigest()})
    (DEST / 'manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
