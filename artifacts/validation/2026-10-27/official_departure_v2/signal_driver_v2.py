"""동일3초 ACK 한도·동일 시퀀스, 재사용 Gazebo 연결. 평가용이며 제어 입력이 아니다."""
import json
from pathlib import Path
import selectors
import subprocess
import sys

import signal_driver as original

BASE=Path(__file__).resolve().parent
client=None


def apply_stage(stage,lamps,world):
    red=int(stage[-1]) if stage.startswith('red') else 0
    def on(lamp):return lamp['column']<red or (lamp['column']==4 and stage in ('green','green_probe'))
    records=[]
    try:
        for lamp in sorted(lamps,key=on):
            active=on(lamp)
            color=('0 1 0' if lamp['column']==4 else '1 0 0') if active else '.004 .004 .004'
            r,g,b=color.split();rgba=f'r:{r} g:{g} b:{b} a:1'
            glow=rgba if active else 'r:0 g:0 b:0 a:1';name=lamp['name']
            request=(f'name:"{name}_visual" parent_name:"{name}" material {{ambient {{{rgba}}} diffuse {{{rgba}}} '
                     f'emissive {{{glow}}} specular {{r:0 g:0 b:0 a:1}}}}')
            client.stdin.write(request+'\n');client.stdin.flush()
            with selectors.DefaultSelector() as poll:
                poll.register(client.stdout,selectors.EVENT_READ)
                if not poll.select(5):raise TimeoutError('persistent client response absent')
            line=client.stdout.readline()
            if not line:raise RuntimeError('persistent client exited without receipt')
            receipt=json.loads(line);records.append(dict(lamp=name,lit=active,**receipt))
            if not all(receipt.get(k) is True for k in ('parsed','executed','service_result','applied')):
                raise RuntimeError('material acknowledgement failed')
        return records
    except Exception as error:
        failure=dict(stage=stage,error=f'{type(error).__name__}: {error}',receipts=records,
                     scope='evaluation driver failure; no stage success or automatic retry')
        (Path(sys.argv[1])/'signal_driver_failure.json').write_text(json.dumps(failure,indent=2),encoding='utf-8')
        raise


def main():
    global client
    manifest=json.loads((BASE/'fixture/manifest.json').read_text(encoding='utf-8'))
    service=f"/world/{manifest['world_name']}/visual_config/blocking"
    client=subprocess.Popen([str(BASE/'native/build/material_client'),service],stdin=subprocess.PIPE,
                            stdout=subprocess.PIPE,text=True,bufsize=1)
    original.apply_stage=apply_stage
    try:original.main()
    finally:
        client.stdin.close()
        try:client.wait(timeout=5)
        except subprocess.TimeoutExpired:
            client.terminate()
            try:client.wait(timeout=3)
            except subprocess.TimeoutExpired:client.kill();client.wait()


if __name__=='__main__':main()
