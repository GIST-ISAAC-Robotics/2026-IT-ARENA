"""실행 소스 동결 중 별도 사본만 만든다. 실제 질의와 무관한 후방 공백 분리."""
from pathlib import Path
import hashlib,json,ast
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
p=ROOT/'src/arena_autonomy/arena_autonomy/same_path_gap_trial.py';raw=p.read_bytes();s=raw.decode().replace('\r\n','\n')
old="""    if reason is not None:
        return {**result, 'reason':reason}
    if proof['reason']=='bounded_forward_missing':"""
new="""    if reason is not None:
        if reason != 'unknown_outside_side_scope':
            return {**result, 'reason':reason}
        # 측방 증거는 전방 결측을 허가하지 않는다. 후방 결측도 실제 이동
        # 영역에 필요하면 허가하지 않는다. 다만 필요하지 않은 후방 광선까지
        # 정면 회전의 같은-경로 증거를 무효화하는 데 쓰지 않는다.
        half_step=math.pi/len(scene.rays)
        unknown=[r for r in scene.rays if r.status=='unknown']
        if any(abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<=math.radians(30)+half_step
               for r in unknown):
            return {**result, 'reason':reason}
        checked=_bounded_nonreturn(scene,raw_scan,q,pad,result)
        if not checked['accepted']:
            return {**result, 'reason':reason}
        needed={i for gap in checked['gaps'] for i in range(gap['first'],gap['last']+1)}
        if not needed or any(not math.radians(30)+half_step<abs(math.atan2(
                scene.rays[i].direction_xy[1],scene.rays[i].direction_xy[0]))<=math.radians(100)
                for i in needed):
            return {**result, 'reason':reason}
        # 모든 이동 질의가 실제 양옆 반환/원시 +inf/팽창/좁은 각도 검사를
        # 통과했다. 범위 밖 광선을 자유로 바꾸거나 해당 방향의 이동을 허가하지 않는다.
        return {**checked,'reason':'bounded_same_path_nonreturn_risk',
            'query_local_side_scope':True,'unqueried_rear_unknown_count':sum(
                abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))>math.radians(100) for r in unknown)}
    if proof['reason']=='bounded_forward_missing':"""
assert s.count(old)==1
s=s.replace(old,new);ast.parse(s)
out=s.replace('\n','\r\n' if b'\r\n' in raw else '\n').encode()
(A/'same_path_gap_trial.py').write_bytes(out)
(A/'candidate_manifest.json').write_text(json.dumps(dict(path=p.relative_to(ROOT).as_posix(),before=hashlib.sha256(raw).hexdigest(),after=hashlib.sha256(out).hexdigest(),scope='artifact-only candidate'),indent=2)+'\n')
print('staged query-local side scope; runtime unchanged')
