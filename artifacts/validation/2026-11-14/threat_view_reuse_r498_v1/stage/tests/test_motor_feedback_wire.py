"""선택형 모터축 주기 피드백의 wire 계약. 합성 카운트이며 실제 보드 시험 아님."""
import json
from pathlib import Path
import subprocess

import pytest

from arena_vehicle_interface.actuation_wire import Decoder
from arena_vehicle_interface.mcu_status_clock import McuStatusClock

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module', params=['optimized', 'sanitized'])
def binary(tmp_path_factory, request):
    stage = tmp_path_factory.mktemp('motor-feedback')
    cpp, exe = stage/'fixture.cpp', stage/'fixture'
    cpp.write_text(r'''
#include "actuation_tx.hpp"
#include <cstdio>
#include <cstdlib>
#include <algorithm>
namespace aa=arena_actuation;
int main(int argc, char**argv) {
  const int mode=std::atoi(argv[1]);
  aa::ReceiverConfig cfg; cfg.feedback_mode=aa::FeedbackMode::kMotorShaft;
  cfg.ticks_per_rev=28; cfg.gear_ratio=2.5*81./17.; cfg.wheel_radius_m=.0325;
  if(mode==3)cfg.feedback_mode=aa::FeedbackMode::kLegacyRearPair;
  const std::uint32_t boot[]={'t','e','s','t'};
  aa::ActuationReceiverCore r(cfg,boot,4);
  aa::TxConfig c;c.offer_period_us=20000;c.max_consecutive_events=2;
  c.offer_status_v1=mode!=2;c.offer_motor_feedback_v1=mode!=0;
  aa::ActuationTxScheduler tx(c,r);
  std::fprintf(stderr,"%d %zu\n",int(tx.fault()),tx.worst_payload(aa::TxKind::kOffer));
  if(mode==2 || mode==3)return tx.fault()==aa::TxFault::kInvalidConfig?0:1;
  std::size_t cap=mode==4?5:aa::kMaxFrame;
  auto sink=[&](const std::uint8_t* p,std::size_t n)->std::size_t{
    n=std::min(n,cap);return std::fwrite(p,1,n,stdout);
  };
  for(std::int64_t now=0;now<=100000;now+=1000){
    if(now==10000 || now==20000){
      const auto count=now==10000?5:-1;
      if(!r.update_motor_encoder(now,aa::IntInput::of(now/10000),aa::IntInput::of(now-1000),aa::IntInput::of(count)))return 2;
    }
    if(mode==4 && now==30000)
      if(!r.update_motor_encoder(now,aa::IntInput::of(3),aa::IntInput::of(29000),aa::IntInput::of(7)))return 4;
    r.tick(now);tx.observe(now);tx.pump(now,sink);
  }
  for(int i=0;i<1000 && tx.frame_in_flight();++i)tx.pump(100000,sink);
  return tx.fault()==aa::TxFault::kNone?0:3;
}
''', encoding='utf-8')
    flags = ['-Os'] if request.param == 'optimized' else ['-O1', '-g', '-fsanitize=address,undefined', '-fno-omit-frame-pointer', '-fno-pie', '-no-pie']
    subprocess.run(['g++', '-std=c++17', *flags, '-I', str(ROOT/'src/arena_vehicle_interface/portable'), str(cpp), '-o', str(exe)], check=True, capture_output=True)
    return exe


def run(binary, mode):
    result = subprocess.run([str(binary), str(mode)], check=True, capture_output=True)
    fault, worst = map(int, result.stderr.split())
    decoder = Decoder()
    frames = []
    for i in range(0, len(result.stdout), 7):
        frames.extend(decoder.feed(result.stdout[i:i+7]))
    return fault, worst, [r for r in frames if r['type']=='offer']


def test_default_has_no_new_fields(binary):
    fault, worst, offers = run(binary, 0)
    assert fault == 0 and worst <= 1536 and offers
    assert all('motor_feedback_v1' not in r for r in offers)


def test_original_capture_signed_count_and_expiry(binary):
    fault, worst, offers = run(binary, 1)
    assert fault == 0 and worst <= 1536
    first = offers[0]['motor_feedback_v1']
    assert first == dict(valid=False, sequence=None, capture_us=None, counts=None)
    fresh = next(r for r in offers if r['status_now_us']==20000)
    assert fresh['motor_feedback_v1'] == dict(valid=True, sequence=2, capture_us=19000, counts=-1)
    stale = next(r for r in offers if r['status_now_us']==80000)
    assert stale['motor_feedback_v1'] == dict(valid=False, sequence=2, capture_us=19000, counts=-1)
    assert all(r['output_ready'] is False and r['state']=='DISARMED' for r in offers)


@pytest.mark.parametrize('mode', [2,3])
def test_incompatible_config_rejected(binary, mode):
    fault, _, offers = run(binary, mode)
    assert fault == 1 and not offers


def test_partial_write_never_refreshes_materialized_capture(binary):
    fault, worst, offers = run(binary, 4)
    assert fault == 0 and worst <= 1536 and len(offers)>=2
    # 첫 state 프레임이 먼저 나간다. 뒤 offer의 materialize 후 카운트가 바뀌어도
    # 이미 일부 전송한 offer는 원 capture/count 그대로다.
    assert 20000<=offers[0]['status_now_us']<30000
    assert offers[0]['motor_feedback_v1']==dict(valid=True, sequence=2, capture_us=19000, counts=-1)
    assert offers[-1]['motor_feedback_v1']['counts']==7
    assert offers[-1]['motor_feedback_v1']['capture_us']==29000


def convert(offers, delay_ns=0):
    clock = McuStatusClock(warmup_ns=0)
    rows = []
    for offer in offers:
        t=offer['status_now_us']*1000
        rows += clock.convert_batch([offer],monotonic_ns=10_000_000_000+t+delay_ns,
                                    realtime_ns=1_790_000_000_000_000_000+t+delay_ns)
    return clock,rows


def test_native_wire_to_observation_preserves_invalid_and_capture(binary):
    _,_,offers=run(binary,1)
    _,rows=convert(offers)
    first=rows[0]['motor_feedback']
    assert first['source_valid'] is False and first['motor_counts'] is None
    assert first['estimated_capture_ns'] is None
    fresh=next(r for r in rows if r['mcu_now_us']==20000)
    sample=fresh['motor_feedback']
    assert sample['source_valid'] and sample['motor_counts']==-1
    assert sample['estimated_capture_ns']==fresh['now_us']*1000-1000000
    stale=next(r for r in rows if r['mcu_now_us']==80000)['motor_feedback']
    assert not stale['source_valid'] and stale['mcu_capture_us']==19000
    assert stale['estimated_capture_ns']==sample['estimated_capture_ns']
    assert not stale['time_contract_verified']


@pytest.mark.parametrize('bad', [dict(valid=True,sequence=None,capture_us=None,counts=None),
    dict(valid=True,sequence=1,capture_us=20001,counts=0),
    dict(valid=True,sequence=1,capture_us=19000,counts=True),
    dict(valid=True,sequence=1,capture_us=None,counts=0),
    dict(valid=True,sequence=True,capture_us=19000,counts=0)])
def test_invalid_feedback_does_not_replace_independent_status(bad):
    offer=dict(type='offer',v=1,boot='t',generation=0,state='DISARMED',reason='startup',
               output_ready=False,estop_held=False,status_now_us=20000,lease_issued_us=20000,
               motor_feedback_v1=bad)
    clock,rows=convert([offer])
    assert len(rows)==1 and rows[0]['state']=='DISARMED'
    assert 'motor_feedback' not in rows[0] and 'motor_feedback_error' in rows[0]
    assert clock.counts['invalid_motor_feedback']==1
