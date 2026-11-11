"""동일 명령에 한정한 기존 구조 가림 정책의 연결 후보. 운영 기본 연결 아님."""
import math
import numpy as np

import arena_autonomy.known_structure as ks
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.scan_input import scan_first_stamp
from arena_autonomy.observation_support import source_valid
from arena_autonomy.recovery_geometry import sweep_rectangles

ORIGINAL_STRUCTURE = ks.StructureAwareMixin._structure
ORIGINAL_SAME = ps.same_path_risk


def structure_with_scope(self, scan, valid, rows, points, history, now, source, speed,
                         steering, previous_steering, wheelbase, length, width, *, shadows=None):
    if shadows is None:
        shadows = self._shadow(scan, rows, history, now, source)
    result = ORIGINAL_STRUCTURE(self, scan, valid, rows, points, history, now, source, speed,
        steering, previous_steering, wheelbase, length, width, shadows=shadows)
    result['same_command_scope'] = dict(reference_s=now, source_first_s=source,
        checked_speed_mps=speed, steering_rad=steering, previous_steering_rad=previous_steering,
        wheelbase_m=wheelbase, length_m=length, width_m=width,
        shadows=[dict(origin=s['origin'].tolist(), low=s['low'], high=s['high'],
                      hidden_width_range_m=s['hidden_width_range_m'],
                      center_deg=s['center_deg']) for s in shadows if s['explained']])
    return result


def structure_risk(scene, raw_scan, queries, padding, baseline, speed, target, steer, previous, cfg):
    out = dict(accepted=False, reason='invalid_structure_proof', unknown_filled=False,
        residual_hidden_object_risk=True, policy='same_command_structure_shadow_r135')
    def reject(reason): return {**out, 'reason':reason}
    proof = baseline.get('partial_observation') or {}
    evidence = proof.get('known_structure') or {}
    checked = evidence.get('same_command_scope') or {}
    if (raw_scan is None or not source_valid(scene) or proof.get('allowed') is not True
            or evidence.get('policy') != 'structure_aware_v1' or evidence.get('evaluated') is not True
            or evidence.get('unknown_filled') is not False
            or evidence.get('accepted_hidden_width_m') != ks.ACCEPTED_HIDDEN_WIDTH_M):
        return out
    fields = ('reference_s','source_first_s','checked_speed_mps','steering_rad',
              'previous_steering_rad','wheelbase_m','length_m','width_m')
    if any(type(checked.get(k)) not in (int, float) or not math.isfinite(checked[k]) for k in fields):
        return reject('missing_checked_motion')
    if (checked['reference_s'] != scene.t_ref_s or checked['source_first_s'] != scene.source_first_s
            or scene.source_first_s != scan_first_stamp(raw_scan)
            or baseline.get('source_epoch') != scene.source_epoch
            or baseline.get('reference_stamp_s') != scene.t_ref_s
            or raw_scan.header.frame_id != scene.source_frame_id):
        return reject('different_scene')
    if (not all(math.isfinite(v) for v in (speed, target, steer, previous))
            or not 0 < target <= .35 or speed < 0
            or max(speed,target) > checked['checked_speed_mps'] + 1e-9
            or abs(steer-checked['steering_rad'])>1e-9
            or abs(previous-checked['previous_steering_rad'])>1e-9
            or any(checked[k] != getattr(cfg,k) for k in ('wheelbase_m','length_m','width_m'))):
        return reject('different_or_larger_motion')
    for key in ('speed_cap_mps','shadow_cap_mps','memory_cap_mps'):
        cap = evidence.get(key)
        if cap is not None and (type(cap) not in (int,float) or not math.isfinite(cap)
                               or max(speed,target)>cap+1e-9):
            return reject('existing_structure_stop')
    q = np.asarray(queries, dtype=float).reshape(-1,2)
    pad = np.asarray(padding, dtype=float)
    if not len(q) or not np.all(np.isfinite(q)) or pad.shape != (len(scene.rays),) or not np.all(np.isfinite(pad)) or np.any(pad<0):
        return reject('invalid_queries')
    inside = np.zeros(len(q), dtype=bool)
    for center, axes, half in sweep_rectangles(checked['checked_speed_mps'], scene.t_ref_s-scene.source_first_s,
            steer, previous, cfg.wheelbase_m, cfg.length_m, cfg.width_m):
        inside |= np.all(abs((q-center)@axes.T)<=half+1e-12, axis=1)
    if not np.all(inside): return reject('outside_checked_sweep')
    # 현재 원시 구조 각도와 양옆 반환을 다시 대조한다. 일반 결측은 포함하지 않는다.
    _, _, rows = ks.classify(raw_scan, ks.parse_sectors(cfg.known_structure_sectors_json))
    by_center = {r['center_deg']:r for r in rows if r['explained']}
    supported = np.zeros(len(q), dtype=bool)
    for shadow in checked.get('shadows', []):
        row = by_center.get(shadow.get('center_deg'))
        if row is None: continue
        origin = np.asarray(shadow['origin'], dtype=float)
        delta = q-origin
        bearing = np.arctan2(delta[:,1],delta[:,0])
        width = shadow['high']-shadow['low']
        offset = (bearing-shadow['low']+math.pi)%math.tau-math.pi
        # 기존5 cm 그림자 한도보다 안쪽이며 양옆 실측 반환보다 앞선 질의만 분류.
        valid = (offset>=0)&(offset<=width)&(np.linalg.norm(delta,axis=1)+np.max(pad)<shadow['hidden_width_range_m'])
        for index in row['flanks']:
            ray = scene.rays[int(index)]
            if ray.status != 'return': valid[:] = False; break
            radius = np.linalg.norm(q-np.asarray(ray.ray_origin_xy_m),axis=1)
            depth = math.dist(ray.ray_origin_xy_m,ray.hit_xy_m)
            valid &= radius+pad[int(index)]<depth
        supported |= valid
    if not np.all(supported): return {**reject('outside_explained_structure'), 'covered_queries':int(supported.sum()), 'queries':len(q)}
    return {**out, 'accepted':True, 'reason':'same_command_existing_structure_risk',
            'queries':len(q), 'maximum_checked_speed_mps':checked['checked_speed_mps']}


def same_path_with_structure(owner, scene, raw_scan, region, context, speed, target, steer, previous, baseline):
    original = ORIGINAL_SAME(owner, scene, raw_scan, region, context, speed, target, steer, previous, baseline)
    if original['accepted'] or not context or not region['unobserved'] or owner.follow_config.known_structure_policy == 'off':
        return original
    q = context['queries'][~context['free'] & ~context['masked']]
    result = structure_risk(scene,raw_scan,q,context['padding'],baseline,speed,target,steer,previous,owner.follow_config)
    return result if result['accepted'] else {**original,'structure_recheck':result}


def install():
    ks.StructureAwareMixin._structure = structure_with_scope
    ps.same_path_risk = same_path_with_structure


def uninstall():
    ks.StructureAwareMixin._structure = ORIGINAL_STRUCTURE
    ps.same_path_risk = ORIGINAL_SAME
