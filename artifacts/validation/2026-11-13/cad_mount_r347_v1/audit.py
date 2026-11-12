"""확정 CAD 센서 프로필의 실행 마스크·출처·무변경 범위를 정적으로 검사."""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from arena_autonomy.cad_mount import mount

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
SOURCE=ROOT/'outputs/cad_final_2026_11_13'
CONFIG=ROOT/'src/arena_description/config'


def resolved(label):
    p=CONFIG/label
    data=yaml.safe_load(p.read_text(encoding='utf8'))
    data['lidar']['scan_plane_meshes']=[str((CONFIG/name).resolve())for name in data['lidar']['scan_plane_meshes']]
    return data


def main():
    assert not (A/'audit.json').exists()
    source=json.loads((SOURCE/'source.json').read_text())
    assert source['sha256']=='c9c203b5a66585ccde71313ea888c16804a1ddb71e6e91514a46bebededbfce6'
    rows=json.loads((SOURCE/'assembly.json').read_text(encoding='utf8'))
    b=np.asarray([r['bbox'] for r in rows if r['depth']==1 and r['bbox'] is not None])
    low=b[:,:3].min(axis=0);high=b[:,3:].max(axis=0)
    size=high-low
    old=resolved('b0390_c1_cad_20261112_frontwire_4mm.yaml')
    result=[]
    for name in ('nowire','frontwire_6mm','inboardwire_6mm'):
        p=resolved(f'b0390_c1_cad_20261113_final_{name}.yaml')
        for key in ('xyz_m','rate_hz'):
            assert p['lidar'][key]==old['lidar'][key]
        assert p['camera']==old['camera'] and p['imu']==old['imu']
        assert p['lidar']['mount_motion_margin_m']==.003
        masks={}
        for marker in (False,True):
            _,text=mount('<sdf><model name="test"><link name="chassis"/></model></sdf>',p,
                .0325,.06,[-.1005,0.,.14],.05,marker)
            mask=json.loads(text)
            assert len(mask['sectors_deg'])==(5 if marker else 4)
            masks['marker140' if marker else 'structure_only']=mask
            if marker:(A/f'{name}_nominal_mask.json').write_text(json.dumps(mask,indent=2)+'\n')
        result.append(dict(profile=name,masks=masks,
            summed_halfwidths_deg=sum(2*h for _,h in masks['structure_only']['sectors_deg']),
            note='sum includes angular and motion reserve; not physical opaque angle'))
    files=[SOURCE/'source.json',SOURCE/'sections.json',SOURCE/'assembly.json',
           ROOT/'scripts/build_final_cad_sensor_profile.py',ROOT/'scripts/review_c1_self_returns.py',
           ROOT/'src/arena_autonomy/arena_autonomy/cad_mount.py']
    files+=list(CONFIG.glob('b0390_c1_cad_20261113_final_*.yaml'))
    (A/'audit.json').write_text(json.dumps(dict(source=source,profiles=result,
        cad_overall_length_width_height_mm=[size[2],size[0],size[1]],
        sensor_centres_unchanged_from_prior_profile=True,old_geometry_and_results_preserved=True,
        vehicle_drivetrain_and_mass_changed=False,hardware_calibrated=False,
        source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in files}),indent=2)+'\n')
    print(json.dumps(dict(overall_mm=[size[2],size[0],size[1]],
        mask_sum_deg={r['profile']:r['summed_halfwidths_deg']for r in result})))


if __name__=='__main__':main()
