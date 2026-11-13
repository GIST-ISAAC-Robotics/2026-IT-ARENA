"""저장 JSON/각도만 읽는 stdlib 분석. 정책·numpy·ROS를 실행하지 않는다."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
BASE=A.parent
INPUTS={
    'r406':BASE/'final_runtime_drive_r406_v1/run1/follow_decisions.jsonl',
    'r377':BASE/'released_cad_drive_r377_v1/run1/follow_decisions.jsonl',
    'r396_current':BASE/'current_corner_runtime_r405_v1/inputs/current.jsonl',
}


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def number(value):return float(value['nonfinite']) if type(value) is dict else value
def angle(value):return (value+math.pi)%math.tau-math.pi


def scan_arrays(data):
    scan=data['scan'];native=scan.get('representation')=='native_measurements_v1'
    if native:
        m=scan['measurements'];values=list(map(number,m['ranges_m']));angles=m['angles_rad']
        low,high=m['range_min_m'],m['range_max_m']
        gaps=[abs(math.remainder(b-a,math.tau)) for a,b in zip(angles,angles[1:])]
        cells=[(gaps[max(0,i-1)]+gaps[min(len(gaps)-1,i)])/2 for i in range(len(values))]
    else:
        values=list(map(number,scan['ranges']));angles=[scan['angle_min']+i*scan['angle_increment'] for i in range(len(values))]
        low,high=scan['range_min'],scan['range_max'];cells=[scan['angle_increment']]*len(values)
    return native,values,angles,cells,low,high


def edge_records(data,header):
    native,values,raw_angles,cells,low,high=scan_arrays(data)
    yaw=header['motion_config']['lidar_yaw_rad'];angles=[angle(x+yaw) for x in raw_angles]
    valid=[math.isfinite(x) and low<=x<=high for x in values]
    order=sorted(range(len(values)),key=lambda i:angles[i]);front=[i for i in order if abs(angles[i])<=math.radians(100)+1e-6]
    sectors=json.loads(header['follow_config'].get('known_structure_sectors_json') or '{}')
    if type(sectors) is list:sectors={'sectors_deg':sectors}
    sector_members=[];excused=set()
    for center,width in sectors.get('sectors_deg',[]):
        members={i for i,a in enumerate(raw_angles) if abs(angle(a-math.radians(center)))<=math.radians(width)+1e-12}
        flank=sorted({(i+d)%len(values) for i in members for d in (-1,1)}-members)
        explained=bool(flank) and all(valid[i] for i in flank) and not(native and (0 in members or len(values)-1 in members))
        if explained:excused.update(members)
        sector_members.append((center,members))
    rows=[];seen=set()
    for edge in (front[0],front[-1]):
        if valid[edge] or edge in excused:continue
        at=order.index(edge);lo=hi=at
        while lo>0 and not valid[order[lo-1]]:lo-=1
        while hi+1<len(order) and not valid[order[hi+1]]:hi+=1
        whole=order[lo:hi+1]
        if tuple(whole) in seen:continue
        seen.add(tuple(whole));inside=[i for i in whole if i in front]
        bounded=lo>0 and hi+1<len(order)
        flank=[order[lo-1],order[hi+1]] if bounded else []
        chain=[flank[0],*whole,flank[1]] if bounded else []
        source_adjacent=bounded and all(abs(a-b)==1 for a,b in zip(chain,chain[1:]))
        row=dict(edge_source_index=edge,whole_unknown_source_indices=whole,inside_unknown_source_indices=inside,
            full_gap_cell_deg=math.degrees(sum(cells[i] for i in whole)),inside_gap_cell_deg=math.degrees(sum(cells[i] for i in inside)),
            full_angle_deg=[math.degrees(angles[whole[0]]),math.degrees(angles[whole[-1]])],
            full_bounded_by_finite=bounded,source_adjacent_without_seam=source_adjacent,
            outside_unknown_count=len(whole)-len(inside),
            candidate_flank_only=bounded and source_adjacent and len(whole)==len(inside),
            missing_kinds=dict(Counter('nan' if math.isnan(values[i]) else 'inf' if values[i]==math.inf else '-inf' if values[i]==-math.inf else 'finite_out_of_range' for i in whole)),
            known_structure_centers_deg=[center for center,members in sector_members if any(i in members for i in whole)],
            flanks=[dict(source_index=i,angle_deg=math.degrees(angles[i]),range_m=values[i],
                acquisition_sensor_xy_m=[values[i]*math.cos(raw_angles[i]),values[i]*math.sin(raw_angles[i])]) for i in flank])
        rows.append(row)
    return rows


def trace_rows(path):
    for line in path.open(encoding='utf8'):
        event=json.loads(line)
        if event['kind']=='header':header=event['data']
        if event['kind']!='call':continue
        row=event['data'];normal=row['output'][2].get('normal',row['output'][2])
        yield dict(trace_seq=event['seq'],input=row['input'],header=header,
            stored_output_speed_mps=row['output'][0],stored_reason=normal.get('reason'),
            stored_partial=normal.get('partial_observation'),
            target_speed_before_hold_mps=normal.get('target_speed_before_hold_mps'))


def summarize(rows):
    touched=[row for row in rows if row['edges']]
    eligible=[row for row in touched if all(edge['candidate_flank_only'] for edge in row['edges'])]
    return dict(records=len(rows),edge_missing_records=len(touched),flank_only_records=len(eligible),
        edge_record_reasons=dict(Counter(row['stored_reason'] for row in touched)),
        flank_only_reasons=dict(Counter(row['stored_reason'] for row in eligible)))


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    destination=args.output.resolve();assert destination.is_relative_to(A) and not destination.exists()
    sources={k:digest(p) for k,p in INPUTS.items()};all_rows={}
    for group in ('r406','r377'):
        rows=[]
        for record in trace_rows(INPUTS[group]):
            data=record.pop('input');header=record.pop('header')
            rows.append(dict(**record,reference_s=data['reference_s'],measured_speed_mps=data['speed_mps'],
                edges=edge_records(data,header)))
        all_rows[group]=rows
    header_path=BASE/'current_corner_runtime_r405_v1/inputs/header.json';header=json.loads(header_path.read_bytes())
    rows=[]
    for line in INPUTS['r396_current'].open(encoding='utf8'):
        record=json.loads(line);data=record['data']['current']
        rows.append(dict(index=record['index'],trace_seq=record['trace_seq'],reference_s=data['reference_s'],
            measured_speed_mps=data['speed_mps'],stored_reason='current_input_not_planner_result',edges=edge_records(data,header)))
    all_rows['r396_current']=rows
    focus=[row for row in all_rows['r406'] if row['reference_s']>=25]
    missing=[row for row in focus if row['stored_reason']=='follow_unbounded_missing']
    assert len(focus)==90 and len(missing)==77
    assert all(len(row['edges'])==1 and row['edges'][0]['full_bounded_by_finite'] for row in missing)
    split=Counter('finite_flank_lost_only' if row['edges'][0]['candidate_flank_only'] else 'unknown_continues_outside' for row in missing)
    assert split=={'finite_flank_lost_only':27,'unknown_continues_outside':50}
    report=dict(scope='stored geometry only; no policy replay or permission inference',
        input_sha256={str(INPUTS[k].relative_to(ROOT)).replace('\\','/'):v for k,v in sources.items()},
        header_sha256=digest(header_path),summaries={k:summarize(v) for k,v in all_rows.items()},
        r406_after25=dict(records=90,reasons=dict(Counter(r['stored_reason'] for r in focus)),unbounded=77,
            split=dict(split),all_missing_kinds_inf=True,
            known_structure_intersection=any(e['known_structure_centers_deg'] for row in missing for e in row['edges'])),
        selected_examples=[missing[0],next(row for row in missing if row['edges'][0]['candidate_flank_only'])],
        interpretation='27 rows lose only an immediately adjacent finite source flank. 50 rows contain real outside unknown and remain excluded; no range/unknown interpolation.')
    assert all(e['missing_kinds']=={'inf':len(e['whole_unknown_source_indices'])} for row in missing for e in row['edges'])
    assert all(digest(INPUTS[k])==v for k,v in sources.items())
    destination.mkdir()
    for group,rows in all_rows.items():
        with (destination/(group+'_rows.jsonl')).open('x',encoding='utf8') as f:
            for row in rows:f.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
    (destination/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')
    print(json.dumps(dict(summaries=report['summaries'],r406_after25=report['r406_after25']),ensure_ascii=False))


if __name__=='__main__':main()
