"""r91 시작 정지의 짧은 재현. 운영 코드는 그대로 두고 전체 status를 평가 측에서 보존."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PARENT = A.parent/'control_10hz_r91_v1/two_actor_run3'
FIXTURE = ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'
sys.path[:0] = [str(FIXTURE), str(ROOT/'scripts'), str(ROOT/'tests')]
import validate_g1_traffic as target
from traffic_posts_setup import configure
from rclpy.node import Node


def main():
    d = A/'run4'
    d.mkdir(exist_ok=False)
    original = json.loads((PARENT/'registered.json').read_bytes())
    args = list(original['args'])
    config = json.loads((PARENT/'recessed_config.json').read_bytes())
    args.extend(['--diagnostic-stop-after-green-s','4'])
    # 충분한 종료 시간을 유지한다. 기존 전체 랩 판정은 그대로이므로 이 짧은 진단은 합격 시험이 아니다.
    (d/'config.json').write_text(json.dumps(config, indent=2)+'\n')
    args[args.index('--config')+1] = str(d/'config.json')
    args[args.index('--output')+1] = str(d/'raw')
    overlay = json.loads((PARENT/'additional_actors.json').read_bytes())
    (d/'actors.json').write_text(json.dumps(overlay, indent=2)+'\n')
    args[args.index('--actor-overlay')+1] = str(d/'actors.json')
    files = set(subprocess.check_output(['git','ls-files','-co','--exclude-standard',
        'src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__),d/'config.json',d/'actors.json'))
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    hashes = {name: sha(ROOT/name) for name in sorted(files) if (ROOT/name).is_file()}
    with zipfile.ZipFile(d/'source.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in hashes: archive.write(ROOT/name,name)
    (d/'registered.json').write_text(json.dumps(dict(parent=str(PARENT.relative_to(ROOT)),
        args=args,sources=hashes,scope='4 s after signal; full status capture only; no operational change'),indent=2)+'\n')
    original_subscribe = Node.create_subscription
    with (d/'full_status.jsonl').open('x',encoding='utf8') as stream:
        def subscribe(node, msg_type, topic, callback, *rest, **kwargs):
            if topic == '/autonomy/status':
                base = callback
                def callback(message):
                    stream.write(json.dumps(dict(wall_s=time.monotonic(),status=json.loads(message.data)))+'\n')
                    stream.flush()
                    base(message)
            return original_subscribe(node,msg_type,topic,callback,*rest,**kwargs)
        Node.create_subscription = subscribe
        configure(target,d/'stage')
        sys.argv = args
        try:
            return target.main()
        finally:
            Node.create_subscription = original_subscribe
            (d/'integrity.json').write_text(json.dumps(dict(changed=[name for name,h in hashes.items()
                if not (ROOT/name).is_file() or sha(ROOT/name)!=h]),indent=2)+'\n')


if __name__ == '__main__':
    raise SystemExit(main())
