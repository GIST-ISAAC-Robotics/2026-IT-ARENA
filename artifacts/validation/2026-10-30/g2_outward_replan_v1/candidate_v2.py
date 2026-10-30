"""오프라인 G2 후보: 관측된 외벽 안에서 같은 방향으로 한 번만 작게 재계획한다. v2는 실제 계산된 점 여유를 사용한다."""
import math
import numpy as np
from arena_autonomy.bypass_entry_trial import EntryGuide,entry_path,_compare_entries


class OutwardReplanGuide(EntryGuide):
    def reset(self):
        super().reset()
        self.outward_replanned=False

    def evaluate(self,scene,history,speed,previous,dt):
        fresh=self.last_source is None or (scene.source_first_s is not None and scene.source_first_s>self.last_source)
        original=super().evaluate(scene,history,speed,previous,dt)
        if (not self.stationary_outer_boundary or not fresh or self.outward_replanned or self.phase!='entry' or self.path is None
                or not self.offset or self.wall_confirmations<2
                or original.get('guide_mode')!='measured_motion_transport_current_outer_wall'):
            return original
        chosen=original.get('selected')
        before_speed=0. if chosen is None else chosen['speed_mps']
        if before_speed>=self.entry.speed_cap_mps-1e-9:return original
        rows=original.get('candidates',[])
        if len(rows)!=1:return original
        before_clearance=rows[0].get('command_clearance_m')
        if before_clearance is None or not math.isfinite(before_clearance):return original
        reference=original.get('reference',{})
        wall=reference.get('walls',{}).get(self.side)
        if wall is None:return original
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        offsets=[x for x in self.entry.offsets_m if x*self.offset>0
                 and 1e-9<abs(x)-abs(self.offset)<=.03+1e-9]
        trials=[];accepted=[]
        for offset in offsets:
            for transition in self.entry.transitions_m:
                try:path=entry_path(fit,offset,self.config,self.entry,transition)
                except ValueError:
                    trials.append(dict(offset_m=offset,transition_m=transition,reason='path_unavailable'));continue
                result=_compare_entries(scene,scene.t_ref_s,speed,previous,dt,config=self.config,entry=self.entry,
                    stationary_boundaries=True,
                    guide=dict(reference=reference,path=path,offset_m=offset,transition_m=transition,
                        outer_side=self.side,stationary_outer_boundary=self.stationary_outer_boundary))
                row=result['candidates'][0] if result['candidates'] else None
                trials.append(dict(offset_m=offset,transition_m=transition,reason=result['reason'],
                    clearance_m=None if row is None else row.get('command_clearance_m'),
                    preview_gap_m=None if row is None else row.get('preview_point_gap_m'),
                    preview_gap_evaluated=row is not None and 'preview_point_gap_m' in row))
                if (row is not None and result['selected'] is not None and row['eligible']
                        and row['transition_reaches_offset'] and row['preview_point_gap_m']>=0
                        and row['command_clearance_m']>=before_clearance+.04-1e-9
                        and result['selected']['speed_mps']>before_speed+1e-9):
                    accepted.append((result,row))
        diagnostic=dict(applied=False,old_offset_m=self.offset,before_speed_mps=before_speed,
            before_clearance_m=before_clearance,trials=trials,free_space_assumed=False,
            full_bypass_verified=False,current_scan_only=True)
        if not accepted:return dict(original,outward_replan=diagnostic)
        result,row=max(accepted,key=lambda pair:(
            math.floor(pair[1]['command_clearance_m']/.04),
            math.floor(pair[1]['preview_return_progress_m']/.02),
            math.floor(pair[1]['preview_point_gap_m']/.005),pair[1]['transition_m']))
        self.path=np.asarray(row['path_points'],dtype=float)
        self.offset,self.transition=row['offset_m'],row['transition_m']
        self.reference_s=scene.t_ref_s
        self.rejoin_confirmations=self.center_confirmations=0
        self.outward_replanned=True
        diagnostic.update(applied=True,new_offset_m=self.offset,new_transition_m=self.transition,
            source_first_s=scene.source_first_s,reference_s=scene.t_ref_s)
        return {**original,**result,'outward_replan':diagnostic,
            'guide_mode':'one_small_outward_replan_current_observation'}
