"""검토용 opt-in 상태 인코더. 제어/관측 객체를 변경하지 않고 정적 좌표만 상세 파일로 참조한다.

현재 PartialObservationPolicy가 최초 래치 때 새로 만든 뒤 변경하지 않는
correspondence_context 객체에만 적용한다. 소유자가 객체를 교체하면 새 레코드다.
full 기본 경로는 기존 json.dumps와 동일하며 상세 파일/스레드를 만들지 않는다.
"""
import copy
import hashlib
import json
from pathlib import Path
import threading
import uuid

from arena_vehicle_interface.queued_log import QueuedLog


class DetailArchive:
    """완료 문자열만 기존 QueuedLog에 전달. write+flush 완료 전 참조를 허용하지 않는다."""
    def __init__(self, path):
        self.path=Path(path)
        if not self.path.is_absolute():
            raise ValueError('explicit absolute detail path required')
        self.stream=self.path.open('x',encoding='utf-8',newline='\n')
        self.archive_id=uuid.uuid4().hex
        self.lock=threading.Lock()
        self.results={}
        self.sequence=0
        try:
            self.log=QueuedLog(self._write,capacity=4)
        except Exception:
            self.stream.close()
            raise

    def _write(self, text):
        key=json.loads(text)['record_id']
        try:
            self.stream.write(text+'\n')
            self.stream.flush()
        except Exception:
            with self.lock:self.results[key]='failed'
            raise
        with self.lock:self.results[key]='written'

    def submit(self, context):
        data=json.dumps(context)
        self.sequence+=1
        ref=dict(archive_id=self.archive_id,record_id=self.sequence,
            sha256=hashlib.sha256(data.encode()).hexdigest(),bytes=len(data.encode()))
        text=json.dumps(dict(schema='guard_status_context_v1',**ref,data=context))
        with self.lock:self.results[self.sequence]='pending'
        accepted=self.log.submit(text)
        if not accepted:
            with self.lock:self.results[self.sequence]='dropped'
        return ref

    def state(self, ref):
        with self.lock:return self.results.get(ref['record_id'],'missing')

    def snapshot(self):
        return dict(path=self.path.name,archive_id=self.archive_id,**self.log.snapshot())

    def close(self):
        complete=self.log.close()
        if complete:self.stream.close()
        return complete


class StatusEncoder:
    def __init__(self, mode='full', archive=None):
        if mode not in ('full','compact_v1'):
            raise ValueError('unknown safety_status_detail')
        if mode=='compact_v1' and archive is None:
            raise ValueError('compact status requires explicit recoverable detail archive')
        self.mode,self.archive=mode,archive
        self._context=self._ref=None

    def encode(self, status):
        if self.mode=='full':
            return json.dumps(status)
        result=dict(status)
        partial=status.get('partial_observation')
        evidence=partial.get('latch_evidence') if isinstance(partial,dict) else None
        context=evidence.get('correspondence_context') if isinstance(evidence,dict) else None
        state='no_context'
        if isinstance(context,dict):
            if context is not self._context:
                self._context=context
                self._ref=self.archive.submit(context)
            state=self.archive.state(self._ref)
            if state in ('failed','dropped','missing'):
                raise RuntimeError('detail archive '+state)
            if state=='written':
                # 나머지 latch 증거·동적 진단·현재 원시점유 결과는 그대로 전달한다.
                compact_evidence={k:v for k,v in evidence.items() if k!='correspondence_context'}
                compact_evidence['correspondence_context_ref']=dict(self._ref)
                result['partial_observation']={**partial,'latch_evidence':compact_evidence}
        result['status_detail']=dict(schema='guard_status_compact_v1',mode=self.mode,
            context_storage=state,full_context_in_message=state!='written',archive=self.archive.snapshot())
        # pending/failed/dropped에는 원 상세를 그대로 발행한다. 구동 출력은 건드리지 않는다.
        return json.dumps(result)


def _warning(warn, reason):
    try:
        warn('guard compact status disabled; full status retained: '+reason)
    except Exception:
        pass  # 진단 경고의 실패도 구동 콜백으로 전달하지 않는다.


class SafeStatusEncoder:
    """compact 구현/저장 오류는 한 번 경고하고 원래 full JSON 경로로 돌아간다."""
    def __init__(self, archive, warn):
        self.archive,self.warn=archive,warn
        self.encoder=StatusEncoder('compact_v1',archive)
        self.disabled_reason=None

    def __call__(self, status):
        if self.disabled_reason is None:
            try:
                return self.encoder.encode(status)
            except Exception as error:
                self.disabled_reason=type(error).__name__+': '+str(error)
                _warning(self.warn,self.disabled_reason)
        return json.dumps(status)

    def close(self):
        try:
            if not self.archive.close():
                _warning(self.warn,'detail writer did not finish')
        except Exception as error:
            _warning(self.warn,'detail close '+type(error).__name__+': '+str(error))


def make_encoder(mode='full', path='', warn=lambda _:None):
    # full에서는 경로 검사·파일·스레드·metadata를 추가하지 않는다.
    if mode=='full':
        return None
    try:
        if mode!='compact_v1':
            raise ValueError('unknown safety_status_detail')
        return SafeStatusEncoder(DetailArchive(path),warn)
    except Exception as error:
        _warning(warn,type(error).__name__+': '+str(error))
        return None


def close_encoder(encoder):
    if encoder is not None:
        encoder.close()


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
