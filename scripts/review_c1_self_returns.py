"""장착 완료 C1 정지 기록에서 거리 한정 자기 반환 마스크 후보를 만든다. 자동 적용 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path

from arena_autonomy.local_scene import blind_parameter
from arena_autonomy.native_lidar_input import decode_measurements


def review(records, nominal, *, stationary, clear_nearby, frame_id,
           residual_motion_m=.003, range_noise_m=.002, angular_noise_deg=.5):
    if not stationary or not clear_nearby:
        raise ValueError('external stationary and clear surrounding space confirmation required')
    if not frame_id or not isinstance(frame_id,str):raise ValueError('explicit sensor frame required')
    if (not .003<=residual_motion_m<=.01 or not .002<=range_noise_m<=.01
            or not .5<=angular_noise_deg<=3.):raise ValueError('invalid calibration reserve')
    spec=blind_parameter(0.,json.dumps(nominal))
    sectors,bands=spec['sectors_deg'],spec.get('range_limits_m')
    if not bands or len(sectors)!=len(bands):raise ValueError('distance-limited nominal mask required')
    hits=[[] for _ in sectors]
    frames=[set() for _ in sectors]
    first=last=None
    epoch=None
    accepted=warmup=outside=0
    for row in records:
        payload=row['payload']
        if payload is None:
            warmup+=1
            continue
        scan=decode_measurements(json.dumps(payload),expected_time_basis=payload['time_basis'])
        if scan.source_frame_id!=frame_id:raise ValueError('sensor frame changed')
        if epoch is None:epoch=scan.source_epoch
        if epoch!=scan.source_epoch:raise ValueError('source epoch changed; use one boot/placement')
        stamp=row['receipt_monotonic_ns']
        if type(stamp) is not int or stamp<=0 or (last is not None and stamp<=last):
            raise ValueError('non-increasing host receipt clock')
        first=stamp if first is None else first
        last=stamp
        for angle,distance in zip(scan.angles_rad,scan.ranges_m):
            if not math.isfinite(distance) or not scan.range_min_m<=distance<=scan.range_max_m:continue
            bearing=math.degrees(angle)
            matches=[]
            for i,((center,half),(lo,hi)) in enumerate(zip(sectors,bands)):
                offset=math.remainder(bearing-center,360.)
                # CAD+배선 예약에서 ±3도/±20mm 안의 근접 반환만 후보화한다.
                if abs(offset)<=half+3. and max(.0,lo-.02)<=distance<=hi+.02:
                    matches.append((i,offset))
            if len(matches)==1:
                i,offset=matches[0]
                hits[i].append((offset,distance));frames[i].add(accepted)
            else:
                outside+=1
        accepted+=1
    span=0. if first is None else (last-first)/1e9
    if accepted<100 or span<10.:raise ValueError('need at least 100 accepted scans and 10 seconds')
    output_sectors,output_bands,details=[],[],[]
    complete=True
    for i,((center,half),(lo,hi)) in enumerate(zip(sectors,bands)):
        enough=len(frames[i])>=20
        if enough:
            offsets,distances=zip(*hits[i])
            nearest=min(distances)
            if nearest<=residual_motion_m+range_noise_m:raise ValueError('self return too close for reserve')
            extra=angular_noise_deg+math.degrees(math.asin(residual_motion_m/nearest))
            # 관측 최소/최대에 추가 여유를 붙이되 기존 CAD 예약을 좁히지 않는다.
            new_half=max(half,max(abs(x) for x in offsets)+extra)
            new_lo=min(lo,nearest-residual_motion_m-range_noise_m)
            new_hi=max(hi,max(distances)+residual_motion_m+range_noise_m)
            if new_half>=90:raise ValueError('candidate angular sector exceeds local scope')
        else:
            complete=False
            new_half,new_lo,new_hi=half,lo,hi
        output_sectors.append([center,new_half]);output_bands.append([new_lo,new_hi])
        details.append(dict(sector=i,observed_scans=len(frames[i]),hits=len(hits[i]),
                            enough_observations=enough,nominal_retained=True))
    mask=dict(sectors_deg=output_sectors,range_limits_m=output_bands)
    blind_parameter(0.,json.dumps(mask))
    return dict(schema='c1_self_return_review_v1',applied=False,
        candidate_ready=complete,source_frame_id=frame_id,source_epoch=epoch,
        accepted_scans=accepted,warmup_rows_skipped=warmup,host_span_s=span,
        external_confirmations=dict(stationary=stationary,nearby_clear=clear_nearby),
        residual_motion_m=residual_motion_m,range_noise_m=range_noise_m,angular_noise_deg=angular_noise_deg,
        unmatched_or_ambiguous_returns=outside,details=details,mask=mask,
        unknown_filled=False,empty_sector_does_not_mean_clear=True,
        scope='review candidate only; observed extrema plus reserve, not a vibration or range-accuracy bound')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input',type=Path,required=True,help='bridge_c1_native_preview converted.jsonl, unmasked')
    p.add_argument('--nominal-mask',type=Path,required=True)
    p.add_argument('--frame-id',required=True)
    p.add_argument('--stationary-confirmed',action='store_true')
    p.add_argument('--nearby-clear-confirmed',action='store_true')
    p.add_argument('--residual-motion-mm',type=float,default=3.)
    p.add_argument('--range-noise-mm',type=float,default=2.)
    p.add_argument('--angular-noise-deg',type=float,default=.5)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():p.error('output exists; preserve previous evidence')
    with a.input.open(encoding='utf8') as stream:
        result=review((json.loads(line) for line in stream),json.loads(a.nominal_mask.read_text(encoding='utf8')),
            stationary=a.stationary_confirmed,clear_nearby=a.nearby_clear_confirmed,frame_id=a.frame_id,
            residual_motion_m=a.residual_motion_mm/1000.,range_noise_m=a.range_noise_mm/1000.,
            angular_noise_deg=a.angular_noise_deg)
    result['input_sha256']=hashlib.sha256(a.input.read_bytes()).hexdigest()
    result['nominal_sha256']=hashlib.sha256(a.nominal_mask.read_bytes()).hexdigest()
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(applied=False,candidate_ready=result['candidate_ready'],output=str(a.output))))


if __name__=='__main__':main()
