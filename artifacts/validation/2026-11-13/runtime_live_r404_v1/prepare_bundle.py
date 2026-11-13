"""검토 완료한 common/profile을 새 Jetson 디렉터리용으로 동결한다."""
from pathlib import Path
import hashlib
import json
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]


def main():
    files={}
    for folder in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'):
        for path in (ROOT/folder).rglob('*'):
            if (path.is_file() and (path.suffix in ('.py','.cpp','.hpp','.h','.xml','.cfg') or path.parent.name=='resource')
                    and not any(part in ('build','__pycache__','install','log') for part in path.relative_to(ROOT).parts)):
                files[path.relative_to(ROOT).as_posix()]=path
    profile=ROOT/'artifacts/validation/2026-11-13/runtime_launch_profile_r401_v1/prepared_v2'
    for name in ('runtime_profile.yaml','bindings.json','expected_header.json'):
        path=profile/name;files[path.relative_to(ROOT).as_posix()]=path
    for name in ('run_live.py','probe_runtime_inputs.py','remote_run.py'):
        path=A/name;files[path.relative_to(ROOT).as_posix()]=path
    manifest=dict(schema='runtime_live_r404_sources_v1',files={n:hashlib.sha256(p.read_bytes()).hexdigest() for n,p in sorted(files.items())},
        actual_sensor_capture_seconds=20,physical_actuation=False,firmware_written=False,encoder_replacement=False)
    raw=(json.dumps(manifest,indent=2)+'\n').encode()
    with zipfile.ZipFile(A/'bundle.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name,path in sorted(files.items()):archive.write(path,name)
        archive.writestr('source_manifest.json',raw)
    (A/'source_manifest.json').write_bytes(raw)
    print(json.dumps(dict(files=len(files),sha256=hashlib.sha256((A/'bundle.zip').read_bytes()).hexdigest())))


if __name__=='__main__':main()
