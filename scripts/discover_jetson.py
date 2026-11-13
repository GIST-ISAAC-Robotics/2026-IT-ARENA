"""IPv4 mDNS로 찾고 기존 SSH 키로 확인한다. 인증/명령/설정 변경은 하지 않는다."""
import argparse
import ipaddress
import json
from pathlib import Path
import re
import socket
import struct
import sys
import time


def read_name(data, offset, seen=None):
    seen=set() if seen is None else seen
    if offset in seen or len(seen)>128:raise ValueError('DNS pointer cycle or depth')
    seen.add(offset);labels=[]
    while True:
        size=data[offset];offset+=1
        if size==0:return '.'.join(labels),offset
        if size&0xc0==0xc0:
            pointer=((size&0x3f)<<8)|data[offset]
            tail,_=read_name(data,pointer,seen)
            return '.'.join(labels+[tail]),offset+1
        if size>63 or offset+size>len(data):raise ValueError('invalid DNS label')
        labels.append(data[offset:offset+size].decode('ascii'));offset+=size


def ipv4_answers(data, host, network, source):
    """알려진 이름·로컬 응답·유효 A 레코드만 후보로 반환한다. 아직 신원 확인은 아니다."""
    if source[1]!=5353 or ipaddress.ip_address(source[0]) not in network:return set()
    addresses=set()
    try:
        _,flags,qd,an,ns,ar=struct.unpack('!HHHHHH',data[:12]);offset=12
        if not flags&0x8000 or flags&0x020f or qd+an+ns+ar>512:return set()
        for _ in range(qd):_,offset=read_name(data,offset);offset+=4
        for _ in range(an+ns+ar):
            name,offset=read_name(data,offset)
            kind,cls,ttl,size=struct.unpack('!HHIH',data[offset:offset+10]);offset+=10
            value=data[offset:offset+size];offset+=size
            if len(value)!=size:raise ValueError('truncated DNS data')
            if name.lower()!=host.lower() or kind!=1 or cls&0x7fff!=1 or ttl<=0 or size!=4:continue
            address=ipaddress.ip_address(value)
            if address in network and not (address.is_multicast or address.is_unspecified
                or address.is_loopback or address in (network.network_address,network.broadcast_address)):
                addresses.add(str(address))
    except (ValueError,IndexError,struct.error,UnicodeDecodeError):return set()
    return addresses


def discover(host, interface, wait_s=2.):
    if not re.fullmatch(r'[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?\.local',host):
        raise ValueError('single device label ending in .local required')
    address=ipaddress.IPv4Interface(interface)
    if not 0<wait_s<=10:raise ValueError('discovery timeout outside (0,10]')
    name=b''.join(bytes([len(label)])+label.encode('ascii') for label in host.split('.'))+b'\0'
    # RFC6762 legacy one-shot query: 임시 UDP 포트에 unicast 응답을 받는다.
    query=struct.pack('!HHHHHH',0x1a11,0,1,0,0,0)+name+struct.pack('!HH',1,1)
    found=set()
    with socket.socket(socket.AF_INET,socket.SOCK_DGRAM,socket.IPPROTO_UDP) as stream:
        stream.bind((str(address.ip),0))
        stream.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_IF,socket.inet_aton(str(address.ip)))
        stream.setsockopt(socket.IPPROTO_IP,socket.IP_MULTICAST_TTL,255)
        stream.sendto(query,('224.0.0.251',5353));deadline=time.monotonic()+wait_s
        while time.monotonic()<deadline:
            stream.settimeout(max(.001,deadline-time.monotonic()))
            try:data,source=stream.recvfrom(65535)
            except socket.timeout:break
            found.update(ipv4_answers(data,host,address.network,source))
    if len(found)>8:raise ValueError('too many candidates; inspect duplicate names')
    return sorted(found)


def verify(candidates, host, port, known_hosts):
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'build/jetson_tools'))
    import paramiko
    keys=paramiko.HostKeys(str(known_hosts));known=keys.lookup(host if port==22 else f'[{host}]:{port}')
    if not known:raise ValueError('no existing pinned SSH identity for device name')
    verified=[];rejected=[]
    for address in candidates:
        transport=connection=None
        try:
            connection=socket.create_connection((address,port),timeout=3.)
            transport=paramiko.Transport(connection)
            transport.start_client(timeout=3.)
            key=transport.get_remote_server_key()
            expected=known.get(key.get_name())
            if expected is not None and expected==key:verified.append(address)
            else:rejected.append(dict(address=address,reason='host_key_mismatch'))
        except (OSError,paramiko.SSHException,EOFError) as error:
            rejected.append(dict(address=address,reason=type(error).__name__))
        finally:
            if transport is not None:transport.close()
            elif connection is not None:connection.close()
    return verified,rejected


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host',required=True)
    source=parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--interface',help='현재 Wi-Fi IPv4/접두 길이')
    source.add_argument('--address',help='수동으로 확인할 IPv4 주소. 호스트 키 검사는 유지한다.')
    parser.add_argument('--known-hosts',type=Path,required=True)
    parser.add_argument('--port',type=int,default=22)
    args=parser.parse_args()
    try:
        if not 1<=args.port<=65535:raise ValueError('invalid SSH port')
        candidates=([str(ipaddress.IPv4Address(args.address))] if args.address
                    else discover(args.host,args.interface))
        verified,rejected=verify(candidates,args.host,args.port,args.known_hosts)
        accepted=len(verified)==1
        print(json.dumps(dict(ok=accepted,host_key_name=args.host,
            verified_address=verified[0] if accepted else None,
            discovery_method='explicit_ipv4' if args.address else 'mdns',candidates=candidates,
            verified_count=len(verified),rejected=rejected,
            reason='verified_unique_device' if accepted else 'missing_or_ambiguous_verified_device',
            authenticated=False,settings_changed=False)))
        return 0 if accepted else 2
    except (OSError,ValueError) as error:
        print(json.dumps(dict(ok=False,verified_address=None,reason=str(error),authenticated=False,settings_changed=False)))
        return 2


if __name__=='__main__':raise SystemExit(main())
