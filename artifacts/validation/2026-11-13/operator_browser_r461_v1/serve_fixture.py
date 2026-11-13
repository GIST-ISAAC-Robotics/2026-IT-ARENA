"""실제 패널 HTML의 브라우저 검수용 합성 HTTP. ROS/장치/구동 연결 없음."""
from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
import struct
import threading
import time
import zlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
PAGE_PATH = ROOT / 'src/arena_autonomy/arena_autonomy/race_ops_page.py'
if '--stage' in __import__('sys').argv:
    PAGE_PATH = OUT / 'stage/race_ops_page.py'
spec = importlib.util.spec_from_file_location('fixture_page', PAGE_PATH)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
PAGE = module.PAGE.replace('<body>', '<body><div style="background:#624b00;padding:6px">화면 검수용 합성 입력 · 실제 차량/ROS/센서 미연결</div>')
TOKEN = 'local-ui-fixture'
OPERATIONS = ['manual_visual_start', 'recovery_reset', 'resume_start', 'prepare_start', 'new_race', 'restore_race', 'set_speed_limit']


def make_state(mode):
    return dict(state='setup', state_label='준비', vehicle_id='ISAAC-UI',
        binding=dict(race_id='fixture-race', race_epoch=1, intervention=0, session='fixture-session'),
        connection=dict(status_age_s=.03, status_stale=False, mcu_age_s=.02),
        race={}, speeds=dict(plan_mps=0., protected_mps=0., mcu_target_mps=0., motor_shaft_equiv_mps=0., motor_feedback_valid=True),
        driving=dict(state='WAIT_GREEN', reason='fixture_only', behavior='LOCAL_FOLLOW', side='left', marker_ids=[0,45], safety_reason='stopped', clearance_m=.8),
        mcu=dict(state='DISARMED', reason='fixture_only', generation=1),
        sensors=[dict(name=n, age_s=a, limit_s=l, ok=True) for n,a,l in [('C1',.03,.25),('IMU',.01,.1),('엔코더',.02,.15),('RGB',.05,.5)]],
        commands=[], events=[dict(sim_s=None, kind='fixture', text='브라우저 확인용 합성 상태. 실제 실행 증거 아님.')], log={}, restore_offer=None,
        controls={op:dict(allowed=op in ['prepare_start','set_speed_limit','new_race'], reason='fixture_only') for op in OPERATIONS},
        development=dict(enabled=True, limit_mps=1.5, maximum_mps=1.5),
        manual=dict(mode=mode, state='idle', prepared=False, reason='fixture_only', limit_mps=.15, maximum_mps=1.5, pulse_limit_mps=.15,
                    steering_limit_deg=11.459, commands=[], binding=dict(session='fixture-manual', generation=1)),
        preview=dict(config=dict(lidar=False,camera=False,lidar_type='native',lidar_topic='/scan/native',camera_topic='/camera/color/image_raw'),
                     active=False,errors={},lidar=None,camera=None))


def fixture_png():
    width,height=640,360
    rows=[]
    for y in range(height):
        row=bytearray([0])
        for x in range(width):
            color=(180,180,180) if (x//40+y//40)%2 else (45,45,45)
            if 180<x<460 and 120<y<240: color=(25,160,60)
            row.extend(color)
        rows.append(row)
    def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(b''.join(rows)))+chunk(b'IEND',b'')


IMAGE=fixture_png()


class Handler(BaseHTTPRequestHandler):
    def log_message(self,*args):pass
    def send_payload(self,value,status=200,ctype='application/json'):
        data=value if isinstance(value,bytes) else json.dumps(value,ensure_ascii=False).encode()
        self.send_response(status)
        self.send_header('Content-Type',ctype)
        self.send_header('Content-Length',str(len(data)))
        self.send_header('Cache-Control','no-store')
        self.send_header('X-Arena-Role','operator')
        self.end_headers()
        self.wfile.write(data)
    def authorized(self):
        return self.headers.get('X-Arena-Token')==TOKEN and self.headers.get('X-Arena-Vehicle')=='ISAAC-UI'
    def do_GET(self):
        path=urlsplit(self.path).path
        if path=='/':return self.send_payload(PAGE.encode(),ctype='text/html; charset=utf-8')
        if not self.authorized():return self.send_payload(dict(error='fixture_auth'),401)
        if path=='/api/state':
            with self.server.lock:
                s=copy.deepcopy(self.server.state)
            p=s['preview']
            if p['config']['lidar']:
                points=[[2.,i/30,math.hypot(2.,i/30)] for i in range(-45,46)]
                p['lidar']=dict(sequence=1,frame='synthetic_fixture_scan',receipt_age_s=.02,drawn_count=len(points),raw_count=len(points),invalid_count=0,points=points,stale=False)
            if p['config']['camera']:
                p['camera']=dict(sequence=1,frame='synthetic_fixture_image',receipt_age_s=.05,width=640,height=360,stale=False)
            return self.send_payload(s)
        if path=='/api/preview.jpg':return self.send_payload(IMAGE,ctype='image/png')
        return self.send_payload(dict(error='fixture_route_not_supported'),404)
    def do_POST(self):
        if not self.authorized():return self.send_payload(dict(error='fixture_auth'),401)
        value=json.loads(self.rfile.read(int(self.headers.get('Content-Length','0'))))
        path=urlsplit(self.path).path
        with self.server.lock:
            s=self.server.state
            with (OUT/'requests.jsonl').open('a',encoding='utf8') as f:f.write(json.dumps(dict(at=time.time(),mode=self.server.mode,path=path,body=value),ensure_ascii=False)+'\n')
            if path=='/api/preview':
                s['preview']['config']=value['config'];s['preview']['active']=bool(value['config']['lidar'] or value['config']['camera'])
                return self.send_payload(s['preview'])
            if path not in ['/api/command','/api/manual']:return self.send_payload(dict(error='fixture_route_not_supported'),404)
            op=value['op']
            if path=='/api/manual':
                m=s['manual']
                if op=='set_speed_limit':m['limit_mps']=value['value'];m['pulse_limit_mps']=min(.2,value['value'])
                if op=='prepare':m['prepared']=True;m['state']='prepared'
                if op in ['run','stop']:m['prepared']=False;m['state']='idle'
                receipt=dict(op=op,stage='fixture_received',reason='실제 구동 없음')
                m['commands'].insert(0,receipt)
                return self.send_payload(receipt)
            if op=='set_speed_limit':s['development']['limit_mps']=value['value']
            if op=='prepare_start':
                s['state']='signal';s['state_label']='신호 대기';s['controls']['manual_visual_start']['allowed']=True
            if op=='manual_visual_start':
                s['state']='running';s['state_label']='주행(합성)';s['controls']['manual_visual_start']['allowed']=False
                s['race']['start_source']='manual_visual';s['speeds']['plan_mps']=.3
            if op=='stop':s['state']='operator_stop';s['state_label']='정지';s['controls']['resume_start']['allowed']=True
            receipt=dict(op=op,stage='policy_accepted',adapter_reason='합성 API 수신만 확인')
            s['commands'].insert(0,receipt)
            return self.send_payload(receipt)


if __name__=='__main__':
    servers=[]
    for mode in ['autonomous','manual']:
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        server.mode=mode;server.state=make_state(mode);server.lock=threading.Lock()
        threading.Thread(target=server.serve_forever,daemon=True).start()
        servers.append(server)
    receipt=dict(pid=__import__('os').getpid(),page_sha256=hashlib.sha256(PAGE_PATH.read_bytes()).hexdigest(),
                 urls={s.mode:f'http://127.0.0.1:{s.server_port}/' for s in servers},vehicle='ISAAC-UI',token=TOKEN,device_connection=False)
    (OUT/'server.json').write_text(json.dumps(receipt,indent=2),encoding='utf8')
    print(json.dumps(receipt),flush=True)
    try:
        while not (OUT/'STOP').exists():time.sleep(.2)
    finally:
        for s in servers:s.shutdown();s.server_close()
        (OUT/'server_stopped.json').write_text(json.dumps(dict(pid=receipt['pid'],normal_shutdown=True)),encoding='utf8')
