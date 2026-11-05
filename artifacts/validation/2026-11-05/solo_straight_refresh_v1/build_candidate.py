"""11/3 단독 직선 속도 후보를 현재(r13) 트리 사본에 다시 적용한다. 작업트리는 건드리지 않는다(WSL python3 -I -B).

사용: build_candidate.py [--base base --cand cand --out .]
입력
  BASE  ~/arena_scratch/straight_refresh/<base> : OUT/base_file_list.txt의 트리 사본(OUT/base_sha256.txt로 검증)
  OLD   ~/arena_scratch/straight_refresh/old_after : 11/3 staged_deployment.zip 압축 해제(새 모듈 원본)
출력
  CAND  ~/arena_scratch/straight_refresh/<cand> : BASE + 후보 변경
  OUT   candidate.patch, before_rN.zip, solo_straight_budget_rN_sha256.json, candidate_manifest.json
부모 계약은 BASE 출처 시험의 마지막 M3_SUCCESSORS 원소이고 N은 그 번호 + 1이다. 핵심 변경 바이트가
VERIFIED_AFTER(r13 기준에서 재생·회귀를 마친 값)와 다르면 실패한다.

11/3 차이 중 외부 녹화 카메라(model.sdf.xacro)는 이번 범위에서 제외한다. 각 치환은 정확히 한 번 일치해야 한다.
"""
import difflib
import hashlib
import json
import os
from pathlib import Path
import shutil
import zipfile

HERE = Path(__file__).resolve().parent
REL = 'artifacts/validation/2026-11-05/solo_straight_refresh_v1'
W = Path(os.path.expanduser('~/arena_scratch/straight_refresh'))
BASE, OLD, CAND = W/'base', W/'old_after', W/'cand'
NL = chr(10)
LF = 'src/arena_autonomy/arena_autonomy/local_follow.py'
LP = 'src/arena_autonomy/arena_autonomy/local_pursuit.py'
LAUNCH = 'src/arena_bringup/launch/g1_follow.launch.py'
VALIDATOR = 'scripts/validate_basic_autonomy.py'
ENTRY = 'scripts/run_current_solo.py'
PROV = 'tests/test_source_provenance_r0_plus_b3.py'
MODULE = 'src/arena_autonomy/arena_autonomy/solo_straight_budget.py'
TEST = 'tests/test_solo_straight_budget.py'
REASON = '기본 OFF 단독 직선 속도 후보(11/3 solo_straight_budget 재적용). 조향·HOLD·보호·가속률 불변, 근거 부족 시 원래 목표.'

EDITS = {
    LF: [
        ("    ray_wall_continuity: bool = False  # 단독 선택형. 먼 옆 벽의 단절을 연속 광선 직선 근거로 보완.\n",
         "    ray_wall_continuity: bool = False  # 단독 선택형. 먼 옆 벽의 단절을 연속 광선 직선 근거로 보완.\n"
         "    solo_straight_budget_trial: bool = False  # 현재 양쪽 직선 벽을 속도 근거로만 쓴다.\n"),
        ("    def __post_init__(self):\n",
         "    def __post_init__(self):\n"
         "        if (type(self.solo_straight_budget_trial) is not bool or\n"
         "                (self.solo_straight_budget_trial and not (\n"
         "                    self.solo_route_only and self.blend_open_speed_by_length and self.boundary_bridge\n"
         "                    and self.preview_above_base_speed and not self.ray_wall_continuity\n"
         "                    and self.occupancy_timing_mode == 'per_ray_candidate'\n"
         "                    and self.partial_observation_policy == 'bounded_local_v4'))):\n"
         "            raise ValueError('straight speed budget requires the selected solo observed route configuration')\n"),
        ("if k not in ('known_structure_policy', ", "if k not in ('solo_straight_budget_trial', 'known_structure_policy', "),
        ("def pursue_path(path, speed, config, age, *, speed_cap=None):\n",
         "def pursue_path(path, speed, config, age, *, speed_cap=None, shape_diagnostic=None):\n"),
        ("    cap = min(config.max_speed_mps if speed_cap is None else speed_cap, braking_speed(remaining, age), float(np.min(preview)),\n"
         "              math.sqrt(config.lateral_acceleration_mps2/max(.15, abs(curvature))))\n",
         "    shape_cap = min(float(np.min(preview)), math.sqrt(config.lateral_acceleration_mps2/max(.15, abs(curvature))))\n"
         "    if shape_diagnostic is not None:\n"
         "        shape_diagnostic['original_curve_speed_cap_mps'] = shape_cap\n"
         "    cap = min(config.max_speed_mps if speed_cap is None else speed_cap, braking_speed(remaining, age), shape_cap)\n"),
        ("                                        raw_scan=raw_scan)\n"
         "        return signed_feedback_detail(output, speed, planning)\n",
         "                                        raw_scan=raw_scan)\n"
         "        if self.config.solo_straight_budget_trial:\n"
         "            from arena_autonomy.solo_straight_budget import evaluate\n"
         "            new_speed, reason = evaluate(output, scene, previous_steering, speed, self.config)\n"
         "            output = new_speed, output[1], {**output[2], 'solo_straight_budget': reason,\n"
         "                                             'original_speed_mps': output[0]}\n"
         "        return signed_feedback_detail(output, speed, planning)\n"),
        ("                                                  speed_cap=cruise if open_route else None)\n",
         "                                                  speed_cap=cruise if open_route else None,\n"
         "                                                  **({'shape_diagnostic': detail} if cfg.solo_straight_budget_trial and open_route else {}))\n"),
    ],
    LP: [
        ("            ray_wall_continuity=bool(self.declare_parameter('g1_ray_wall_continuity',False).value),\n",
         "            ray_wall_continuity=bool(self.declare_parameter('g1_ray_wall_continuity',False).value),\n"
         "            solo_straight_budget_trial=bool(self.declare_parameter('g1_solo_straight_budget_trial',False).value),\n"),
    ],
    LAUNCH: [
        ("        DeclareLaunchArgument('g1_standing_wall_reacquire', default_value='false', choices=['true', 'false'],\n",
         "        DeclareLaunchArgument('g1_solo_straight_budget_trial', default_value='false', choices=['true', 'false'],\n"
         "                              description='단독 직선 시험: 양쪽 벽을 현재 조향의 속도 거리 근거로만 사용'),\n"
         "        SetParameter(name='g1_solo_straight_budget_trial', value=LaunchConfiguration('g1_solo_straight_budget_trial')),\n"
         "        DeclareLaunchArgument('g1_standing_wall_reacquire', default_value='false', choices=['true', 'false'],\n"),
    ],
    VALIDATOR: [
        ('    parser.add_argument("--disable-speed-bump", action="store_true")\n',
         '    parser.add_argument("--disable-speed-bump", action="store_true")\n'
         "    parser.add_argument('--g1-solo-straight-budget', action='store_true',\n"
         "                        help='선정 단독 구성에서 현재 두 옆벽의 직선 관측 길이로만 증속하는 선택 시험')\n"),
        ("            'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial')]\n"
         "    if args.local_sensor_config is not None:\n",
         "            'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial')]\n"
         "    if args.g1_solo_straight_budget:\n"
         "        if not (args.g1_race_path and args.speed_profile == 'g1_open_10kmh_solo_gap05_arc_preview_bridge'\n"
         "                and args.g1_boundary_evidence == 'corner_distance_candidate'\n"
         "                and args.g1_occupancy_timing == 'per_ray_candidate'):\n"
         "            raise ValueError('straight speed budget requires the recorded solo bridge profile and per-ray occupancy')\n"
         "        extra_arguments.append('g1_solo_straight_budget_trial:=true')\n"
         "    if args.local_sensor_config is not None:\n"),
        ("        rel='src/arena_autonomy/arena_autonomy/turn_preview.py'\n"
         "        report['input_sha256'][rel]=hashlib.sha256((REPO/rel).read_bytes()).hexdigest()\n"
         "    recorder = recorder_log = None\n",
         "        rel='src/arena_autonomy/arena_autonomy/turn_preview.py'\n"
         "        report['input_sha256'][rel]=hashlib.sha256((REPO/rel).read_bytes()).hexdigest()\n"
         "    if args.g1_solo_straight_budget:\n"
         "        rel='src/arena_autonomy/arena_autonomy/solo_straight_budget.py'\n"
         "        report['input_sha256'][rel]=hashlib.sha256((REPO/rel).read_bytes()).hexdigest()\n"
         "    recorder = recorder_log = None\n"),
    ],
    ENTRY: [
        ('    parser.add_argument("--previous-jr50", action="store_true", help="이전 JR50 프리셋 재현(신규 개발 기준 아님)")\n',
         '    parser.add_argument("--previous-jr50", action="store_true", help="이전 JR50 프리셋 재현(신규 개발 기준 아님)")\n'
         '    parser.add_argument("--straight-budget", action="store_true",\n'
         '                        help="기본 OFF 단독 직선 속도 후보(현재 양쪽 벽 관측 길이로만 증속) 선택")\n'),
        ('        "--pilot-marker-z", MARKER_Z,\n    ]\n',
         '        "--pilot-marker-z", MARKER_Z,\n    ]\n'
         '    if opts.straight_budget:\n'
         '        args.append("--g1-solo-straight-budget")\n'),
    ],
}
# 2026-11-05 r13 기준(base)에서 재생·회귀를 마친 핵심 바이트. 다른 기준으로 다시 만들 때 이 값과 같아야 한다.
VERIFIED_AFTER = {
    LF: '5f0c196c09986d4b7bc920d0aaca41d3ae80d2d1f6630f4666aca397db4fa06d',
    LP: '2e3fe76402064bc8e50cae39766823abfb6f8984de886554cfe7bd7b56c7cc8f',
    LAUNCH: 'c67314798e29a091a96f724586ceab0d6843166643f391063aae654c9c7b39f3',
    VALIDATOR: '0e5bd1e637f98d0689c90f2778bd69bc618600b3c20bd8ed1b21fd2cb7c20281',
    ENTRY: 'a2444fb5fac7ddcb1451cefb93de3fa5e2251983165d03e3c9e2c1ab92c5535c',
    MODULE: '961b12c9fdefe6044e819a01a0abeb0d46609d96bfc1e2e23465cd776d440917',
    TEST: '3ff99065b8b9c872bc41e43868bd4fb7ad77b31e8471e8d220eedb8046f4a4ec',
}


def chain_tail(text):
    """출처 시험의 마지막 M3_SUCCESSORS 원소와 그 원소가 끝나는 줄. 부모 계약과 다음 번호를 정한다."""
    import re
    start = text.rindex('M3_SUCCESSORS +=')
    statement = text[start:text.index("')", start)+2]
    names = re.findall(r"'(config/tests/[a-z0-9_]+_r(\d+)_sha256\.json)'", statement)
    parent, revision = names[-1][0], int(names[-1][1])+1
    anchor = f"'{parent}')"
    assert text.count(anchor+chr(13)+NL)+text.count(anchor+NL) == 1, anchor
    return parent, revision, anchor
CHANGED_SOURCES = (LF, LP, LAUNCH)
CHANGED_SCRIPTS = (VALIDATOR, ENTRY)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def verify_base(base, out):
    bad = []
    for line in (out/'base_sha256.txt').read_text(encoding='utf-8').splitlines():
        digest, path = line.split('  ', 1)
        if sha((base/path).read_bytes()) != digest:
            bad.append(path)
    if bad:
        raise SystemExit(f'base copy differs from base_sha256.txt: {bad[:5]}')


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--base', default='base')
    parser.add_argument('--cand', default='cand')
    parser.add_argument('--out', default='.')
    opts = parser.parse_args()
    base, cand, out = W/opts.base, W/opts.cand, (HERE/opts.out).resolve()
    out_rel = out.relative_to(HERE.parents[3]).as_posix()
    verify_base(base, out)
    if cand.exists():
        shutil.rmtree(cand)
    shutil.copytree(base, cand, symlinks=True)
    prov_text = (base/PROV).read_bytes().decode('utf-8')
    parent, revision, anchor = chain_tail(prov_text)
    contract_rel = f'config/tests/solo_straight_budget_r{revision}_sha256.json'
    edits = dict(EDITS)
    edits[PROV] = [(anchor+NL, anchor+NL+
                    f"# 11/5 단독 직선 속도 후보 재적용(기본 OFF). 일반 후속 계약 검사가 r{revision-1} 다음으로 확인한다.{NL}"
                    f"M3_SUCCESSORS += ('{contract_rel}',){NL}")]
    patch, rows = [], {}
    for rel, items in edits.items():
        text = (base/rel).read_bytes().decode('utf-8')
        eol = chr(13)+NL if chr(13)+NL in text else NL
        new = text
        for old, rep in items:
            old, rep = old.replace(NL, eol), rep.replace(NL, eol)
            assert new.count(old) == 1, (rel, old[:70], new.count(old))
            new = new.replace(old, rep)
        (cand/rel).write_bytes(new.encode('utf-8'))
        rows[rel] = dict(before=sha(text.encode('utf-8')), after=sha(new.encode('utf-8')))
        patch.extend(difflib.unified_diff(text.splitlines(True), new.splitlines(True), f'a/{rel}', f'b/{rel}'))
    # 새 모듈은 11/3 원본 바이트 그대로(내용 변경 없음). 시험도 11/3 원본.
    for rel in (MODULE, TEST):
        data = (OLD/rel).read_bytes()
        (cand/rel).write_bytes(data)
        patch.extend(difflib.unified_diff([], data.decode('utf-8').splitlines(True), '/dev/null', f'b/{rel}'))
        rows[rel] = dict(before=None, after=sha(data))
    mismatch = {rel: rows[rel]['after'] for rel, digest in VERIFIED_AFTER.items() if rows[rel]['after'] != digest}
    if mismatch:
        raise SystemExit(f'candidate bytes differ from the verified r13 build: {mismatch}')
    before_zip = out/f'before_r{revision}.zip'
    with zipfile.ZipFile(before_zip, 'w', zipfile.ZIP_DEFLATED) as archive:
        for rel in (*CHANGED_SOURCES, *CHANGED_SCRIPTS):
            info = zipfile.ZipInfo(rel, date_time=(2026, 11, 5, 0, 0, 0))
            archive.writestr(info, (base/rel).read_bytes())
    contract = {
        'schema_version': 1,
        'scope': '단독 직선 속도 후보(기본 OFF) 재적용. 11/3 외부 녹화 카메라 변경 제외. 기본 채택·실차 승인 아님.',
        'parent_contract': parent,
        'parent_contract_sha256': sha((base/parent).read_bytes()),
        'before_snapshot': f'{out_rel}/before_r{revision}.zip',
        'before_snapshot_sha256': sha(before_zip.read_bytes()),
        'approved_changes': {rel: {**rows[rel], 'reason': REASON} for rel in CHANGED_SOURCES},
        'approved_evaluation_scripts': {rel: {**rows[rel], 'reason': REASON} for rel in CHANGED_SCRIPTS},
        'new_dependencies': {MODULE: rows[MODULE]['after']},
    }
    data = (json.dumps(contract, ensure_ascii=False, indent=2)+NL).encode('utf-8')
    (out/Path(contract_rel).name).write_bytes(data)
    (cand/contract_rel).write_bytes(data)
    rows[contract_rel] = dict(before=None, after=sha(data))
    patch.extend(difflib.unified_diff([], data.decode('utf-8').splitlines(True), '/dev/null', f'b/{contract_rel}'))
    (out/'candidate.patch').write_bytes(''.join(patch).encode('utf-8'))
    manifest = dict(base=opts.base, base_list=f'{out_rel}/base_file_list.txt', base_sha256=f'{out_rel}/base_sha256.txt',
                    parent_contract=parent, contract=contract_rel, files=rows,
                    patch_sha256=sha((out/'candidate.patch').read_bytes()),
                    excluded_from_11_3='src/arena_description/models/arena_car/model.sdf.xacro (외부 녹화 카메라)')
    (out/'candidate_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=1)+NL, encoding='utf-8', newline=NL)
    print(json.dumps(dict(parent=parent, contract=contract_rel, files=len(rows)), ensure_ascii=False))


if __name__ == '__main__':
    main()
