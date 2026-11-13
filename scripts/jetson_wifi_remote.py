"""일회성 root helper. PSK는 stdin→root keyfile로만 전달하고 출력하지 않는다."""
from pathlib import Path
import json,os,re,stat,subprocess,sys,time,uuid
PROFILE_DIR=Path('/etc/NetworkManager/system-connections')
STATE_DIR=Path('/run/it-arena-wifi')

def command(argv,check=True):
    result=subprocess.run(argv,capture_output=True,text=True,timeout=40,check=False)
    if check and result.returncode:raise RuntimeError('command_failed')
    return result

def canonical_uuid(value):return str(uuid.UUID(str(value)))
def escaped(value):
    return str(value).replace('\\','\\\\').replace(' ','\\s').replace('\n','\\n').replace('\r','\\r').replace('\t','\\t')

def keyfile(data,connection_uuid):
    ssid=data['ssid'];psk=data['psk'];name=data.get('profile_name','IT-ARENA Venue')
    if not isinstance(ssid,str) or not 1<=len(ssid.encode('utf-8'))<=32 or '\0' in ssid:raise ValueError('invalid_ssid')
    if not isinstance(name,str) or not 1<=len(name)<=128 or '\0' in name:raise ValueError('invalid_name')
    security=data['security']
    if security not in ('WPA2','WPA3'):raise ValueError('unsupported_security')
    ascii_pass=isinstance(psk,str) and 8<=len(psk)<=63 and all(32<=ord(c)<=126 for c in psk)
    hex_psk=isinstance(psk,str) and security=='WPA2' and bool(re.fullmatch('[0-9A-Fa-f]{64}',psk))
    if not (ascii_pass or hex_psk):raise ValueError('invalid_personal_key')
    # 문법이 SSID의 세미콜론/개행을 설정 구분자로 해석하지 않도록 byte-list를 사용한다.
    ssid_bytes=''.join(str(b)+';' for b in ssid.encode('utf-8'))
    return ('[connection]\nid='+escaped(name)+'\nuuid='+canonical_uuid(connection_uuid)+'\ntype=wifi\nautoconnect=false\n'
        '\n[wifi]\nmode=infrastructure\nssid='+ssid_bytes+'\n'
        '\n[wifi-security]\nkey-mgmt='+('sae' if security=='WPA3' else 'wpa-psk')+'\npsk='+escaped(psk)+'\npsk-flags=0\n'
        '\n[ipv4]\nmethod=auto\n\n[ipv6]\nmethod=auto\n')

def private_write(path,text):
    descriptor=os.open(path,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
    with os.fdopen(descriptor,'w',encoding='utf-8') as stream:
        os.fchmod(stream.fileno(),0o600);os.fchown(stream.fileno(),0,0)
        stream.write(text);stream.flush();os.fsync(stream.fileno())

def state_directory():
    STATE_DIR.mkdir(mode=0o700,exist_ok=True)
    meta=STATE_DIR.lstat()
    if not stat.S_ISDIR(meta.st_mode) or meta.st_uid!=0 or stat.S_IMODE(meta.st_mode)!=0o700:raise RuntimeError('private_state_directory_required')

def active_uuid(interface):
    return canonical_uuid(command(['nmcli','-g','GENERAL.CON-UUID','device','show',interface]).stdout.strip())

def timer_active(unit):return command(['systemctl','is-active','--quiet',unit+'.timer'],check=False).returncode==0

def start_transition(data,target):
    token=data['transition_id']
    if not isinstance(token,str) or not re.fullmatch('[0-9a-f]{32}',token):raise ValueError('invalid_transition_id')
    seconds=data.get('rollback_seconds',180)
    if type(seconds)!=int or not 60<=seconds<=600:raise ValueError('invalid_rollback_seconds')
    interface=data['interface'];fallback=active_uuid(interface)
    if fallback==target:raise ValueError('target_is_already_active')
    state_directory()
    for path in STATE_DIR.glob('*.json'):
        old=json.loads(path.read_text())
        if timer_active(old['rollback_unit']):raise RuntimeError('previous_transition_still_pending')
    unit='it-arena-wifi-rollback-'+token
    state=dict(transition_id=token,target_uuid=target,fallback_uuid=fallback,interface=interface,
        rollback_unit=unit,deadline_monotonic_s=time.monotonic()+seconds)
    private_write(STATE_DIR/(token+'.json'),json.dumps(state))
    # 복귀 예약·활성 확인이 성공한 뒤에만 전환을 예약한다. 실패 시 타이머를 지우지 않는다.
    command(['systemd-run','--quiet','--unit='+unit,'--on-active='+str(seconds)+'s',
        '--timer-property=AccuracySec=1s','/usr/bin/nmcli','--wait','30','connection','up','uuid',fallback,'ifname',interface])
    if not timer_active(unit):raise RuntimeError('rollback_timer_not_active')
    command(['systemd-run','--quiet','--unit=it-arena-wifi-switch-'+token,'--on-active=3s',
        '--timer-property=AccuracySec=1s','/usr/bin/nmcli','--wait','30','connection','up','uuid',target,'ifname',interface])
    return dict(ok=True,status='activation_scheduled',connection_uuid=target,transition_id=token,
        fallback_uuid=fallback,rollback_seconds=seconds,confirmation_required=True)

def confirm(data):
    token=data.get('transition_id')
    if data.get('user_confirmed') is not True or not isinstance(token,str) or not re.fullmatch('[0-9a-f]{32}',token):raise ValueError('explicit_confirmation_required')
    state_directory();path=STATE_DIR/(token+'.json');state=json.loads(path.read_text())
    if active_uuid(state['interface'])!=state['target_uuid']:raise RuntimeError('target_not_active')
    unit=state['rollback_unit']
    if time.monotonic()>=state['deadline_monotonic_s'] or not timer_active(unit):raise RuntimeError('rollback_already_due_or_inactive')
    # 새 망의 명시 확인 뒤 부팅 시에도 대회장 프로필을 선택할 수 있게 한다.
    command(['nmcli','connection','modify','uuid',state['target_uuid'],'connection.autoconnect','yes','connection.autoconnect-priority','100'])
    if time.monotonic()>=state['deadline_monotonic_s'] or not timer_active(unit):raise RuntimeError('rollback_already_due_or_inactive')
    command(['systemctl','stop',unit+'.timer'])
    if command(['systemctl','is-active','--quiet',unit+'.service'],check=False).returncode==0:raise RuntimeError('rollback_service_already_running')
    path.unlink()
    return dict(ok=True,status='confirmed',connection_uuid=state['target_uuid'],autoconnect=True,autoconnect_priority=100,rollback_cancelled=True)

def dispatch(data):
    if not re.fullmatch('[A-Za-z0-9_.-]{1,32}',data.get('interface','')):raise ValueError('invalid_interface')
    action=data.get('action')
    if action=='confirm':return confirm(data)
    if action=='register':
        target=str(uuid.uuid4());text=keyfile(data,target)
        path=PROFILE_DIR/('it-arena-venue-'+target+'.nmconnection')
        temporary=PROFILE_DIR/('.it-arena-venue-'+target+'.tmp')
        try:
            private_write(temporary,text)
            if path.exists():raise FileExistsError('profile_already_exists')
            temporary.rename(path)
        finally:
            if temporary.exists():temporary.unlink()
        command(['nmcli','connection','load',str(path)])
    elif action=='select':
        target=canonical_uuid(data['connection_uuid'])
        if not data.get('activate'):raise ValueError('select_requires_activate')
        command(['nmcli','-g','connection.uuid','connection','show','uuid',target])
    else:raise ValueError('unsupported_action')
    if data.get('activate'):return start_transition(data,target)
    return dict(ok=True,status='stored_only',connection_uuid=target,autoconnect=False,activation_scheduled=False)

def main():
    try:
        if os.geteuid()!=0:raise PermissionError('root_required')
        # sudo가 암호를 소비했거나 기존 인증으로 생략한 두 경우 모두 처리한다.
        # 남은 SSH 암호 줄은 버리고, 명시 프레임만 JSON으로 읽는다. 파일/로그 저장 없음.
        prefix='IT_ARENA_WIFI_JSON_V1 '
        for _ in range(2):
            line=sys.stdin.readline()
            if line.startswith(prefix):break
        else:raise ValueError('missing_input_frame')
        result=dispatch(json.loads(line[len(prefix):]))
    except Exception as error:
        # 예외 문자열/원 입력/nmcli stderr는 PSK를 포함할 수 있어 공개하지 않는다.
        result=dict(ok=False,status='failed',error_type=type(error).__name__,rollback_cancelled=False,
            next_action='If activation was requested, keep the previous Wi-Fi available and allow rollback. Do not repeat before checking.')
    print(json.dumps(result));return 0 if result['ok'] else 1

if __name__=='__main__':raise SystemExit(main())
