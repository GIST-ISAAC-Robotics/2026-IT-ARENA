"""r543의 순수 복원 함수만 원문 재사용. ROS/encoder/writer를 설치하거나 실행하지 않는다.

원 stage_integration/guard_status_payload.py SHA256:
7971b36808de9fb288c17ffa348ee9b69f08b22dbdcfda978eeb2704c335d693
"""
import copy
import hashlib
import json
from pathlib import Path


def load_details(path):
    records={}
    for line in Path(path).read_text(encoding='utf-8').splitlines():
        row=json.loads(line)
        if row.get('schema')!='guard_status_context_v1':raise ValueError('unknown detail schema')
        data=json.dumps(row['data']).encode()
        if len(data)!=row['bytes'] or hashlib.sha256(data).hexdigest()!=row['sha256']:
            raise ValueError('detail integrity mismatch')
        key=(row['archive_id'],row['record_id'])
        if key in records:raise ValueError('duplicate detail key')
        records[key]=row
    return records


def restore_status(status, records):
    result=copy.deepcopy(status)
    metadata=result.pop('status_detail',None)
    if metadata is None:return result
    if metadata.get('schema')!='guard_status_compact_v1':raise ValueError('unknown compact schema')
    evidence=(result.get('partial_observation') or {}).get('latch_evidence') or {}
    ref=evidence.get('correspondence_context_ref')
    if ref is not None:
        row=records[(ref['archive_id'],ref['record_id'])]
        if row['sha256']!=ref['sha256'] or row['bytes']!=ref['bytes']:
            raise ValueError('referenced detail mismatch')
        evidence.pop('correspondence_context_ref')
        evidence['correspondence_context']=copy.deepcopy(row['data'])
    return result
