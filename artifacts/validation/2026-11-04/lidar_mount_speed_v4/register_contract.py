"""Preserve the pilot predecessor and the C speed correction as separate revisions."""
import hashlib,json,subprocess,zipfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
def sha(b):return hashlib.sha256(b).hexdigest()
def load(p):return json.loads((ROOT/p).read_text(encoding='utf8'))
def save_contract(name,parent,snapshot,changes,scope):
    with zipfile.ZipFile(snapshot) as z:
        rows={p:dict(before=sha(z.read(p)),after=sha(b),reason=reason) for p,b,reason in changes}
    result=dict(schema_version=1,scope=scope,parent_contract=parent,
                parent_contract_sha256=sha((ROOT/parent).read_bytes()),
                before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
                approved_changes={p:r for p,r in rows.items() if p.startswith('src/')},
                approved_evaluation_scripts={p:r for p,r in rows.items() if not p.startswith('src/')},new_dependencies={})
    dst=ROOT/name; content=json.dumps(result,ensure_ascii=False,indent=2)+'\n'
    if dst.exists():assert dst.read_text(encoding='utf8')==content
    else:dst.write_text(content,encoding='utf8',newline='\n')
def main():
    p2='config/tests/lidar_mount_adapted_r2_sha256.json'
    p3='config/tests/lidar_mount_pilot_r3_sha256.json'
    p4='config/tests/lidar_mount_speed_r4_sha256.json'
    sim='src/arena_bringup/launch/simulation.launch.py';ev='scripts/validate_basic_autonomy.py'
    with zipfile.ZipFile(HERE/'runtime_before.zip') as z:
        sim3=z.read(sim); ev3=z.read(ev);follow3=z.read('src/arena_autonomy/arena_autonomy/local_follow.py')
    t=sim3.decode('utf8').replace('\r\n','\n')
    t=t.replace('"scan_plane_posts", "scan_plane_wires",','"scan_plane_posts",')
    start=t.index('    wires = [tuple(');end=t.index('    sectors = []',start)
    t=t[:start]+t[end:]
    t=t.replace('in [*posts, *wires]:','in posts:')
    start=t.index('    pilot_marker_z = LaunchConfiguration(');end=t.index('    rear_marker_rpy =',start)
    t=t[:start]+t[end:]
    t=t.replace('            DeclareLaunchArgument("pilot_marker_z", default_value="",\n'
                '                                  description="파일럿 전용 ID49 판 중심 높이 덮어쓰기; 빈 값은 기본 설정"),\n','')
    wanted=load(p2)['approved_changes'][sim]['after']
    versions=[t.encode(),t.replace('\n','\r\n').encode()]
    sim2=next(b for b in versions if sha(b)==wanted)
    ev2=subprocess.run(['git','show','HEAD:'+ev],cwd=ROOT,capture_output=True,check=True).stdout
    # Restore the original Windows newline convention only when it matches the preserved pilot.
    candidates=[ev2,ev2.replace(b'\n',b'\r\n')]
    et=ev3.decode('utf8').replace('\r\n','\n')
    et=et.replace('    parser.add_argument(\'--pilot-marker-z\', type=float, default=None, help=\'파일럿 전용 후면 ID49 판 높이 덮어쓰기\')\n','')
    s=et.index('    if args.pilot_marker_z is not None:');e=et.index('    if args.g1_race_path:',s)
    et=et[:s]+et[e:]
    with zipfile.ZipFile(ROOT/'artifacts/validation/2026-11-04/lidar_mount_adapted_v2/runtime_before.zip') as z:
        ev2=z.read(ev)
    assert et.encode('utf8')==ev2.replace(b'\r\n',b'\n')==candidates[0].replace(b'\r\n',b'\n')
    before=HERE/'pilot_before.zip'
    if not before.exists():
        with zipfile.ZipFile(before,'x',compression=zipfile.ZIP_DEFLATED) as z:
            z.writestr(sim,sim2);z.writestr(ev,ev2)
    save_contract(p3,p2,before,[(sim,sim3,'v3 시험 전용 배선 단면과 후면 마커 높이 전달; 제어기 동일'),
                              (ev,ev3,'v3 검사기의 opt-in 후면 마커 높이 인자')],
                  '이미 완료한 v3 원본 소스 보존. 기본 채택·실차 승인 아님.')
    path='src/arena_autonomy/arena_autonomy/local_follow.py'
    assert sha(follow3)==load(p2)['approved_changes'][path]['after']
    save_contract(p4,p3,HERE/'runtime_before.zip',[(path,(ROOT/path).read_bytes(),
                  'C 구조 적응 경로에서 전방 전광선 증속 전제 제거. 후보 속도/조향으로 기존 결측·그림자 보호 최종 검사')],
                  'C/JR 공통 고속 후보 및 정지 빈도 비교. 기본 정책 변경·실차 승인 아님.')
    print(p3,p4)
if __name__=='__main__':main()
