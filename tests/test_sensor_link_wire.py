import struct
from pathlib import Path
import subprocess
import zlib

import pytest

from arena_vehicle_interface.actuation_wire import Decoder, encode
from arena_vehicle_interface.sensor_link_wire import SensorLinkDecoder


def imu(payload=b'IA\x10\x00IMU1'):
    raw=struct.pack('<IHBBIIQQIHBB16sII',0x31554d49,68,1,1,123,456,100,110,12,3,0,8,payload.ljust(16,b'\x00'),0,0)
    return raw+struct.pack('<I',zlib.crc32(raw))


@pytest.mark.parametrize('chunk', [1,2,3,4,7,64,68,129,4096])
def test_split_and_mixed_frames_do_not_parse_embedded_magic(chunk):
    ia=encode(dict(type='offer', text='IMU1 IA'))
    source=b'boot ROM\r\n'+imu()+ia+imu()
    d=SensorLinkDecoder(); rows=[]
    for i in range(0,len(source),chunk): rows.extend(d.feed(source[i:i+chunk]))
    assert rows==[('imu',imu()),('control',ia),('imu',imu())]
    assert d.counters['nonframe_bytes']==10
    assert not d.counters['crc_errors']
    assert not d.buffer


def test_corrupt_and_reconnect():
    good=imu(b'no magic'); bad=bytearray(good);bad[40]^=1
    d=SensorLinkDecoder()
    assert d.feed(bad+good)==[('imu',good)]
    assert d.counters['crc_errors']==1
    assert not d.feed(good[:25]); d.reset()
    assert d.feed(good)==[('imu',good)]


def test_header_length_and_bounded_input():
    good=imu(b'no magic'); bad=bytearray(good);bad[4:6]=b'\xff\xff'
    d=SensorLinkDecoder()
    assert d.feed(b'IA\x00\x00'+bad+good)==[('imu',good)]
    assert d.counters['invalid_header']==2
    assert d.feed(b'x'*65537)==[] and not d.buffer
    assert d.counters['input_limit']==1


@pytest.fixture(scope='module')
def mux_exe(tmp_path_factory):
    root=Path(__file__).resolve().parents[1]
    stage=tmp_path_factory.mktemp('sensor-mux')
    cpp=stage/'test.cpp'; exe=stage/'test'
    cpp.write_text(r'''
#include "sensor_link_tx.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
namespace aa=arena_actuation;
int main(int argc,char**argv){
  aa::ReceiverConfig cfg; aa::TxConfig tx;
  tx.offer_period_us=20000;tx.max_consecutive_events=2;
  std::uint32_t boot[]={'t'};
  aa::ActuationBenchEndpoint ep(cfg,boot,1,30000,aa::PayloadLayout::kEnvelope,tx);
  ep.connect(0);
  aa::SensorLinkTx mux;
  std::array<std::uint8_t,68> frame{};
  frame[0]='I';frame[1]='M';frame[2]='U';frame[3]='1';frame[4]=68;frame[6]=1;frame[7]=1;
  // 프레임 본문에 magic이 있어도 분리되지 않아야 한다.
  frame[40]='I';frame[41]='A';
  auto crc=aa::crc32(frame.data(),64);
  for(int i=0;i<4;++i)frame[64+i]=(crc>>(8*i))&255;
  for(int i=0;i<64;++i)if(!mux.push(frame.data(),68))return 1;
  if(mux.push(frame.data(),68)||mux.dropped()!=1)return 2;
  auto max=static_cast<std::size_t>(std::atoi(argv[1]));
  std::uint64_t calls=0;
  for(std::int64_t t=0;t<100000;++t){
    if(t%5000==0)ep.control_tick(t);
    auto sink=[&](const std::uint8_t*p,std::size_t n)->std::size_t{
      if(++calls%5==0)return 0;
      n=std::min(n,max);return std::fwrite(p,1,n,stdout);
    };
    if(!mux.pump(t,ep,sink))return 3;
    if(mux.completed()==64&&!ep.tx().frame_in_flight())return 0;
  }
  return 4;
}
''',encoding='utf-8')
    subprocess.run(['g++','-std=c++17','-O2','-I',str(root/'src/arena_vehicle_interface/portable'),str(cpp),'-o',str(exe)],check=True,capture_output=True)
    return exe


@pytest.mark.parametrize('accepted', [1,7,64,128,4096])
def test_native_mux_partial_zero_writes_and_queue_bound(mux_exe, accepted):
    raw=subprocess.run([str(mux_exe),str(accepted)],check=True,capture_output=True).stdout
    d=SensorLinkDecoder(); rows=d.feed(raw)
    assert d.counters['imu']==64 and d.counters['control']>=1
    assert not d.counters['nonframe_bytes'] and not d.counters['crc_errors'] and not d.buffer
    for kind,p in rows:
        if kind=='control':
            payload=Decoder().feed(p)
            assert len(payload)==1
            if payload[0]['type']=='offer': assert payload[0]['output_ready'] is False
