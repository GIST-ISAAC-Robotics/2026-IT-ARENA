"""같은 관측 면이 앞으로 빠질 때 감속만 제한적으로 유지하는 미연결 후보."""
from observer import RelativeYieldObserver, finite


class PersistentYieldObserver(RelativeYieldObserver):
    maximum_continuation_s=5.
    maximum_observed_front_m=1.2
    wait_for_rejoin=False

    def reset(self):
        super().reset()
        self.yield_since=None
        self.anchor=None

    def stop_advice(self,reason):
        self.reset()
        self.detail['reason']=reason
        return self.detail

    def update(self,row):
        if self.yield_since is None:
            result=super().update(row)
            if result.get('suggest_yield'):
                self.yield_since=row['source'];self.anchor=dict(row)
            return result
        if not isinstance(row,dict):
            return self.stop_advice('continuation_observation_missing')
        required=('source','now','heading','speed','normal','along_low','along_high')
        if (not all(finite(row.get(k)) for k in required)
                or row.get('phase')!='entry' or row.get('side')!=self.anchor['side']
                or row.get('method')!='parallel_face_median' or row.get('wall_confirmations',0)<2
                or not 0<=row['speed']<=1 or not 0<=row['now']-row['source']<=self.scan_gap_s
                or abs(row['heading'])>.08 or not .10<=row['along_high']-row['along_low']<=.50
                or row['along_high']<-.10 or row['along_low']>self.maximum_observed_front_m):
            return self.stop_advice('continuation_current_evidence_invalid')
        if row.get('band_supported') and not self.wait_for_rejoin:
            return self.stop_advice('continuation_current_band_clear')
        if not 0<=row['now']-self.yield_since<=self.maximum_continuation_s:
            return self.stop_advice('continuation_budget_expired')
        key=(row.get('epoch'),row['source'])
        if key[0]!=self.key[0] or key[1]<self.key[1] or key[1]-self.key[1]>self.scan_gap_s:
            return self.stop_advice('continuation_source_changed')
        old=self.rows[-1];anchor=self.anchor
        width=lambda r:r['along_high']-r['along_low']
        if (abs(row['normal']-anchor['normal'])>.06 or abs(width(row)-width(anchor))>.06
                or abs(row['heading']-anchor['heading'])>.04
                or min(row['along_high'],old['along_high'])-max(row['along_low'],old['along_low'])<-.06):
            return self.stop_advice('continuation_face_changed')
        if key!=self.key:
            self.rows.append(dict(row))
            while len(self.rows)>self.minimum_scans:self.rows.popleft()
            self.key=key
        self.detail=dict(suggest_yield=True,reason='bounded_current_face_yield_continuation',
            trial_speed_cap_mps=self.trial_speed_cap_mps,elapsed_s=row['now']-self.yield_since,
            scans=len(self.rows),current_band_occupied=bool(row.get('blocked_by_current_returns')),
            current_band_supported=bool(row.get('band_supported')),current_face_along_m=[row['along_low'],row['along_high']],
            control_authorized=False,moving_object_identity_proven=False,unknown_filled=False)
        return self.detail
