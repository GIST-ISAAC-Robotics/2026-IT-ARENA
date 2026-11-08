"""직전 긴 외벽의 실제 관측점과 현재 짧은 반환을 대조하는 미연결 가설.

이력은 점유점/자유 공간을 만들지 않는다. 현재 실제 반환 중 이전에 긴 외벽으로
보았던 위치와 일치한 점만 비교 대상으로 표시한다. 이동 물체 신원 증명이 아니다.
"""
from collections import deque
import numpy as np
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.known_structure import classify
from arena_autonomy.wall_reference_support import observed_rear_wall_face


class WallReturnMemory:
    def __init__(self):
        self.epoch=None
        self.rows={'left':deque(maxlen=4),'right':deque(maxlen=4)}

    def compare(self, scene, config, side, wall, history, raw_scan, sectors):
        evidence=dict(accepted=False, reason='current_and_previous_wall_support_absent',
            current_indices=[], unknown_filled=False, static_identity_verified=False,
            previous_points_added_to_occupancy=False)
        if side not in self.rows or not sectors or raw_scan is None or history is None:
            return evidence
        if scene.source_epoch!=self.epoch:
            self.rows={'left':deque(maxlen=4),'right':deque(maxlen=4)}
            self.epoch=scene.source_epoch
        rows=self.rows[side]
        if rows and scene.source_first_s<rows[-1]['source']:
            rows.clear()
        while rows and scene.t_ref_s-rows[0]['source']>=config.memory_ttl_s:
            rows.popleft()
        if wall.get('reason')!='current_connected_straight_wall_fit': return evidence
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        if fit[3]-fit[2]<.65 or wall.get('actual_return_count',0)<12: return evidence
        points=np.asarray(scene.occupied_endpoints_xy_m)
        ray_ids=np.asarray([r.source_index for r in scene.rays if r.status=='return'])
        _,_,structure=classify(raw_scan,sectors)
        explained={frozenset(r['rays'].tolist()) for r in structure if r['explained']}
        # .20 m는 기존 후륜 경로 지지용 현재 면 조건이다. 이것만으로 정적으로 분류하지 않는다.
        rear=observed_rear_wall_face(scene,config,side,fit,wall,explained,
            minimum_rear_span_m=.20,rear_limit_m=-1.5)
        if rear is not None:
            current=np.flatnonzero(np.isin(ray_ids,rear['source_ray_indices']))
            supported=np.zeros(len(current),dtype=bool)
            previous=[]
            for row in rows:
                if row['source']>=scene.source_first_s: continue
                try:
                    old=transport_points(row['points'],history,row['reference'],scene.t_ref_s)
                except (ValueError,MotionUnavailable): continue
                # 현재 선의 양 끝이 아니라 실제 이전 반환점까지의 거리다.
                distance=np.min(np.linalg.norm(points[current,None,:]-old[None,:,:],axis=2),axis=1)
                matches=distance<=config.measurement_padding_m+.015
                supported|=matches
                if matches.any(): previous.append(dict(source=row['source'],points=int(matches.sum()),
                    age_s=scene.t_ref_s-row['source'],maximum_distance_m=float(distance[matches].max())))
            if supported.any():
                evidence.update(accepted=True,reason='current_return_matches_previous_long_wall',
                    current_indices=ray_ids[current[supported]].tolist(), previous=previous,
                    current_rear_proof=rear['proof'],
                    matching_distance_m=config.measurement_padding_m+.015)
        # 새로 분류한 짧은 조각을 다시 기억의 씨앗으로 쓰지 않는다. 긴 현재 주면만 저장한다.
        if not rows or scene.source_first_s>rows[-1]['source']:
            ids=np.isin(ray_ids,wall['source_ray_indices'])
            if np.count_nonzero(ids)>=12:
                rows.append(dict(source=scene.source_first_s,reference=scene.t_ref_s,points=points[ids].copy()))
        return evidence
