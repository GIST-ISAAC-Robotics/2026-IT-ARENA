"""스캔 누락 없이 첫 판단만 남겨 호출 생략과 스캔 누락의 영향을 분리한다.

원래 궤적/속도/직전 조향을 사용하는 저장 입력 비교. 실차/폐루프 증거가 아니다.
"""
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
FROZEN=ROOT/'build/r89_pc_frozen'
sys.path[:0]=[str(FROZEN/'src/arena_autonomy'),str(FROZEN/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
from probe_follow_cadence import Cadence, run, measured_costs, earliest_scan_stamp, digest


class OncePerScan(Cadence):
    def __init__(self, sources, dt):
        self.sources,self.fixed_dt=sources,dt
        super().__init__('all','recorded')

    def reset(self):
        super().reset()
        self.last_source=None

    def offer(self, now,cost,dt):
        super().offer(now,cost,dt)  # 기존 시계/유한성 경계 유지
        source=self.sources[now]
        if source == self.last_source:
            return None
        self.last_source=source
        return self.fixed_dt,now+cost


def main():
    manifest=json.loads((ROOT/'artifacts/validation/2026-11-11/occluder_cpu_r89_v1/candidate_bundle_manifest.json').read_bytes())
    unchanged=lambda: all(digest(FROZEN/name)==r['sha256'] for name,r in manifest['files'].items())
    assert unchanged()
    trace=FROZEN/manifest['trace']
    costs_path=ROOT/'artifacts/validation/2026-11-11/occluder_cpu_r89_v1/jetson_stages.json'
    cost_report=json.loads(costs_path.read_bytes())
    before=digest(trace),digest(costs_path)
    assert before[0]==cost_report['trace_sha256']
    costs=measured_costs(cost_report)
    sources={}
    with trace.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='call':
                data=event['data']['input']
                assert data['reference_s'] not in sources
                sources[data['reference_s']]=(data['epoch'],earliest_scan_stamp(data))
    for dt in (.02,.1):
        d=A/('fixed_20ms' if dt==.02 else 'fixed_100ms')
        d.mkdir(exist_ok=False)
        with trace.open() as stream,(d/'admitted.jsonl').open('x') as sink:
            result=run(stream,costs,OncePerScan(sources,dt),sink)
        rows=[json.loads(l) for l in (d/'admitted.jsonl').read_text().splitlines()]
        processed={sources[r['reference_s']] for r in rows}
        result.update(fixed_dt_s=dt,unique_input_scans=len(set(sources.values())),
            processed_scans=len(processed),missed_scans=len(set(sources.values())-processed),
            inputs_unchanged=before==(digest(trace),digest(costs_path)),source_unchanged=unchanged(),
            script_sha256=digest(__file__),trace_sha256=before[0],costs_sha256=before[1],
            scope='saved trajectory first decision per scan, no missed source; no output hold or closed-loop / real-time proof')
        (d/'report.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:result[k] for k in ('fixed_dt_s','admitted','missed_scans',
            'command_mismatch_count','newly_zero_command_seqs','source_unchanged')}) ,flush=True)


if __name__=='__main__':main()
