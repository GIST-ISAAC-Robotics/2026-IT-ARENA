"""등록 전 조회 또는 단 한 번의 등록 뒤 확인/정확한 자기 UUID 회수."""
from pathlib import Path
import base64,hashlib,json,os,stat,subprocess,sys,uuid
D=Path('/etc/NetworkManager/system-connections')
data=json.loads(base64.b64decode(sys.argv[1]));f=data['fixture']
def cmd(args,check=True):
    r=subprocess.run(['nmcli',*args],capture_output=True,text=True,timeout=15)
    if check and r.returncode:raise RuntimeError('nmcli_command_failed')
    return r
def active():return str(uuid.UUID(cmd(['-g','GENERAL.CON-UUID','device','show',f['interface']]).stdout.strip()))
def profiles():return sorted(p.name for p in D.glob('it-arena-venue-*.nmconnection'))
def fingerprints(exclude=None):
    return {hashlib.sha256(p.name.encode()).hexdigest():hashlib.sha256(p.read_bytes()).hexdigest()
            for p in D.iterdir() if p.is_file() and p!=exclude}
def escaped(s):return s.replace('\\','\\\\').replace(' ','\\s')
if data['mode']=='before':
    absent=f['profile_name'] not in cmd(['-g','NAME','connection','show']).stdout.splitlines()
    result=dict(active_uuid=active(),probe_name_absent=absent,profile_files=profiles(),fingerprints=fingerprints(),pid=os.getpid())
    assert absent and result['active_uuid']==f['initial_active_uuid']
    print(json.dumps(result));raise SystemExit(0)
before=data['before'];registration=data['registration'];checks={};cleanup={};path=None;target=None;error=None
try:
    added=set(profiles())-set(before['profile_files'])
    if registration.get('ok'):
        target=str(uuid.UUID(registration['connection_uuid']))
        name='it-arena-venue-'+target+'.nmconnection'
        assert added=={name},'unexpected_added_profiles'
    else:
        assert len(added)<=1,'unexpected_added_profiles'
        name=next(iter(added),None)
        if name:target=str(uuid.UUID(name[len('it-arena-venue-'):-len('.nmconnection')]))
    if target is not None:
        path=D/('it-arena-venue-'+target+'.nmconnection')
        contents=path.read_text();lines=contents.splitlines()
        assert 'uuid='+target in lines and 'id='+escaped(f['profile_name']) in lines,'profile_not_owned'
        meta=path.stat()
        checks['root_owned_0600']=meta.st_uid==0 and meta.st_gid==0 and stat.S_IMODE(meta.st_mode)==0o600
        if registration.get('ok'):
            def read(field,secret=False):
                args=(['--show-secrets'] if secret else [])+['--escape','no','-g',field,'connection','show','uuid',target]
                return cmd(args).stdout.removesuffix('\n')
            checks.update(profile_id_matches=read('connection.id')==f['profile_name'],
                ssid_matches=read('802-11-wireless.ssid')==f['ssid'],
                psk_matches=read('802-11-wireless-security.psk',True)==f['psk'],
                autoconnect_false=read('connection.autoconnect')=='no',
                active_uuid_unchanged_after_store=active()==before['active_uuid'],
                existing_files_unchanged_after_store=fingerprints(path)==before['fingerprints'])
except BaseException as exc:error=type(exc).__name__
finally:
    if path is not None and target is not None:
        # UUID와 정확한 파일의 공개 시험 ID를 다시 확인한 뒤 자기 프로필만 삭제한다.
        lines=path.read_text().splitlines() if path.exists() else []
        if 'uuid='+target in lines and 'id='+escaped(f['profile_name']) in lines:
            deleted=cmd(['connection','delete','uuid',target],check=False)
            cleanup['nmcli_delete_exit']=deleted.returncode
            if path.exists():path.unlink()
            cleanup['exact_file_absent']=not path.exists()
            cleanup['uuid_absent']=cmd(['-g','connection.uuid','connection','show','uuid',target],check=False).returncode!=0
        else:cleanup['ownership_check_failed']=True
    else:cleanup['no_created_profile_found']=True
    cleanup['active_uuid_unchanged']=active()==before['active_uuid']
    cleanup['profile_files_restored']=profiles()==before['profile_files']
    cleanup['existing_files_unchanged']=fingerprints()==before['fingerprints']
result=dict(registration_ok=registration.get('ok',False),connection_uuid=target,checks=checks,cleanup=cleanup,error_type=error,
    pid=os.getpid(),activate_requested=False,background_process_started=False,user_secret_used=False)
result['passed']=registration.get('ok') is True and error is None and all(checks.values()) and all(
    cleanup.get(k) is True for k in ('exact_file_absent','uuid_absent','active_uuid_unchanged','profile_files_restored','existing_files_unchanged'))
print(json.dumps(result));raise SystemExit(0 if result['passed'] else 1)
