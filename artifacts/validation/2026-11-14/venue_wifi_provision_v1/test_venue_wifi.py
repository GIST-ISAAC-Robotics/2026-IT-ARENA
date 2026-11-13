"""Windows 순수 모의 검사. 실제 credential/NIC/SSH/WSL/Jetson을 사용하지 않는다."""
from pathlib import Path
import configparser,contextlib,importlib.util,io,json,os,sys,tempfile,unittest
from types import SimpleNamespace
from unittest.mock import patch,MagicMock
A=Path(__file__).resolve().parent;S=A/'stage/scripts';ROOT=A.parents[3]
sys.path[:0]=[str(S),str(ROOT/'build/jetson_tools')]
import jetson_wifi as client
import jetson_wifi_remote as remote
import jetson_access as access
OLD='11111111-1111-4111-8111-111111111111';NEW='22222222-2222-4222-8222-222222222222';TOKEN='3'*32
DUMMY='dummy $();`"\\; key'  # 공개 합성 fixture이며 실제 암호가 아니다.

class FakeCommands:
    def __init__(self):self.calls=[];self.current=OLD;self.timers=set();self.failure=None
    def __call__(self,argv,check=True):
        self.calls.append(argv)
        if argv[:3]==['nmcli','-g','GENERAL.CON-UUID']:return SimpleNamespace(returncode=0,stdout=self.current+'\n')
        if argv[:2]==['systemd-run','--quiet']:
            unit=next(x[7:] for x in argv if x.startswith('--unit='))
            is_rollback='rollback' in unit
            if self.failure==('rollback' if is_rollback else 'switch'):raise RuntimeError('synthetic failure')
            self.timers.add(unit+'.timer')
        if argv[:2]==['systemctl','is-active']:
            return SimpleNamespace(returncode=0 if argv[-1] in self.timers else 3,stdout='')
        if argv[:2]==['systemctl','stop']:self.timers.discard(argv[-1])
        if argv[:3]==['nmcli','connection','modify'] and self.failure=='modify':raise RuntimeError('synthetic failure')
        return SimpleNamespace(returncode=0,stdout=NEW+'\n')

class WiFiTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory(prefix='mock_',dir=A)
        self.directory=Path(self.temporary.name).resolve();assert self.directory.is_relative_to(A.resolve())
        self.commands=FakeCommands();self.stack=contextlib.ExitStack()
        self.stack.enter_context(patch.object(remote,'PROFILE_DIR',self.directory))
        self.stack.enter_context(patch.object(remote,'STATE_DIR',self.directory/'state'))
        self.stack.enter_context(patch.object(remote,'command',self.commands))
        self.stack.enter_context(patch.object(remote,'state_directory',lambda:(self.directory/'state').mkdir(exist_ok=True)))
        self.stack.enter_context(patch.object(remote,'private_write',lambda path,text:path.write_text(text,encoding='utf-8')))
        self.stack.enter_context(patch.object(remote.time,'monotonic',return_value=10))
    def tearDown(self):self.stack.close();self.temporary.cleanup()
    def selected(self):return dict(action='select',connection_uuid=NEW,activate=True,interface='wlP1p1s0',transition_id=TOKEN,rollback_seconds=180)
    def registered(self):return dict(action='register',ssid='test;한글\n[evil]',psk=DUMMY,security='WPA3',profile_name='Name\n[evil]=x',activate=False,interface='wlP1p1s0')
    def test_default_store_never_activates_or_arms_timer(self):
        result=remote.dispatch(self.registered())
        self.assertEqual(result['status'],'stored_only');self.assertFalse(result['autoconnect'])
        self.assertEqual([c[:3] for c in self.commands.calls],[['nmcli','connection','load']])
        self.assertFalse(list(self.directory.glob('*.tmp')))
        self.assertNotIn(DUMMY,json.dumps(result));self.assertNotIn(DUMMY,json.dumps(self.commands.calls))
    def test_keyfile_escaping_cannot_inject_sections_and_preserves_ssid_bytes(self):
        d=self.registered();text=remote.keyfile(d,NEW);cfg=configparser.ConfigParser(interpolation=None);cfg.read_string(text)
        self.assertEqual(set(cfg.sections()),{'connection','wifi','wifi-security','ipv4','ipv6'})
        self.assertEqual(bytes(int(v) for v in cfg['wifi']['ssid'].split(';') if v).decode(),d['ssid'])
        self.assertEqual(cfg['wifi-security']['key-mgmt'],'sae');self.assertEqual(cfg['connection']['autoconnect'],'false')
        self.assertIn('\\n[evil]',cfg['connection']['id']);self.assertIn('\\\\',cfg['wifi-security']['psk'])
    def test_invalid_ssid_or_key_rejected_before_any_write_or_command(self):
        for changes in ({'ssid':'한'*11},{'psk':'short'},{'psk':'line\npassword'},{'security':'open'}):
            with self.subTest(changes=list(changes)):
                with self.assertRaises(ValueError):remote.dispatch(self.registered()|changes)
        self.assertEqual(self.commands.calls,[]);self.assertEqual(list(self.directory.iterdir()),[])
    def test_rollback_armed_before_switch_and_no_secret_in_timer_commands(self):
        result=remote.dispatch(self.selected());commands=self.commands.calls
        run=[c for c in commands if c[0]=='systemd-run'];self.assertEqual(len(run),2)
        self.assertIn('rollback',run[0][2]);self.assertIn(OLD,run[0]);self.assertIn(NEW,run[1])
        self.assertEqual(result['fallback_uuid'],OLD);self.assertNotIn('stop',[c[1] for c in commands if c[0]=='systemctl'])
    def test_timer_failure_prevents_activation(self):
        self.commands.failure='rollback'
        with self.assertRaises(RuntimeError):remote.dispatch(self.selected())
        self.assertFalse(any('switch-' in ' '.join(c) for c in self.commands.calls))
    def test_activation_failure_keeps_rollback_timer(self):
        self.commands.failure='switch'
        with self.assertRaises(RuntimeError):remote.dispatch(self.selected())
        self.assertIn('it-arena-wifi-rollback-'+TOKEN+'.timer',self.commands.timers)
        self.assertFalse(any(c[:2]==['systemctl','stop'] for c in self.commands.calls))
    def test_unconfirmed_or_old_network_cannot_cancel(self):
        remote.dispatch(self.selected())
        for confirmed in (False,True):
            with self.assertRaises((ValueError,RuntimeError)):remote.dispatch(dict(action='confirm',interface='wlP1p1s0',transition_id=TOKEN,user_confirmed=confirmed))
        self.assertFalse(any(c[:2]==['systemctl','stop'] for c in self.commands.calls))
    def test_confirm_target_enables_autoconnect_100_then_cancels(self):
        remote.dispatch(self.selected());self.commands.current=NEW
        result=remote.dispatch(dict(action='confirm',interface='wlP1p1s0',transition_id=TOKEN,user_confirmed=True))
        calls=self.commands.calls;modify=next(i for i,c in enumerate(calls) if c[:3]==['nmcli','connection','modify'])
        stop=next(i for i,c in enumerate(calls) if c[:2]==['systemctl','stop'])
        self.assertLess(modify,stop);self.assertEqual(calls[modify][-4:],['connection.autoconnect','yes','connection.autoconnect-priority','100'])
        self.assertTrue(result['rollback_cancelled']);self.assertFalse((self.directory/'state'/f'{TOKEN}.json').exists())
    def test_failed_autoconnect_update_keeps_rollback(self):
        remote.dispatch(self.selected());self.commands.current=NEW;self.commands.failure='modify'
        with self.assertRaises(RuntimeError):remote.dispatch(dict(action='confirm',interface='wlP1p1s0',transition_id=TOKEN,user_confirmed=True))
        self.assertFalse(any(c[:2]==['systemctl','stop'] for c in self.commands.calls))
    def test_expired_or_overlapping_transition_not_cancelled_or_replaced(self):
        remote.dispatch(self.selected())
        with self.assertRaises(RuntimeError):remote.dispatch(self.selected()|dict(transition_id='4'*32))
        self.commands.current=NEW
        with patch.object(remote.time,'monotonic',return_value=200):
            with self.assertRaises(RuntimeError):remote.dispatch(dict(action='confirm',interface='wlP1p1s0',transition_id=TOKEN,user_confirmed=True))
        self.assertFalse(any(c[:2]==['systemctl','stop'] for c in self.commands.calls))
    def test_transport_secret_only_in_stdin_no_log_or_temp_path(self):
        request=dict(host='unit-test.local',user='q',port=22,password='dummy-ssh',known_hosts='existing_known_hosts',connect_address=None,wifi=self.registered())
        result=client.transport_request(request)
        self.assertNotIn(DUMMY,result['command']);self.assertIn(DUMMY,json.loads(result['stdin_payload'].split(' ',1)[1])['psk'])
        self.assertIsNone(result['log']);self.assertEqual(result['action'],'run');self.assertTrue(result['sudo'])
        self.assertNotIn('destination',result)
    def test_remote_error_and_cached_sudo_line_do_not_echo_secrets(self):
        payload='IT_ARENA_WIFI_JSON_V1 '+json.dumps(self.registered())+'\n'
        for prefix in ('','dummy-ssh-password\n'):
            output=io.StringIO()
            with patch.object(remote.os,'geteuid',return_value=0,create=True),patch.object(remote.sys,'stdin',io.StringIO(prefix+payload)),contextlib.redirect_stdout(output):
                self.assertEqual(remote.main(),0)
            self.assertNotIn(DUMMY,output.getvalue());self.assertNotIn('dummy-ssh-password',output.getvalue())
        output=io.StringIO()
        with patch.object(remote.os,'geteuid',return_value=0,create=True),patch.object(remote.sys,'stdin',io.StringIO(payload)),patch.object(remote,'dispatch',side_effect=RuntimeError(DUMMY)),contextlib.redirect_stdout(output):
            self.assertEqual(remote.main(),1)
        self.assertNotIn(DUMMY,output.getvalue());self.assertFalse(json.loads(output.getvalue())['rollback_cancelled'])

class TransportTests(unittest.TestCase):
    def test_private_file_creation_is_root_0600_exclusive(self):
        stream=MagicMock();stream.__enter__.return_value=stream;stream.fileno.return_value=37
        with patch.object(remote.os,'open',return_value=37) as opened,patch.object(remote.os,'fdopen',return_value=stream),patch.object(remote.os,'fchmod',create=True) as chmod,patch.object(remote.os,'fchown',create=True) as chown,patch.object(remote.os,'fsync'):
            remote.private_write(Path('/safe/root-only/file'),'dummy')
        self.assertEqual(opened.call_args.args[2],0o600);self.assertTrue(opened.call_args.args[1]&os.O_EXCL)
        chmod.assert_called_once_with(37,0o600);chown.assert_called_once_with(37,0,0)
    def test_existing_access_keeps_host_key_validation_and_transmits_after_sudo_line(self):
        stdin=io.StringIO();stdin.channel=MagicMock();channel=MagicMock()
        channel.recv_ready.return_value=False;channel.recv_stderr_ready.return_value=False;channel.exit_status_ready.return_value=True;channel.recv_exit_status.return_value=0
        ssh=MagicMock();ssh.exec_command.return_value=(stdin,SimpleNamespace(channel=channel),None)
        request=dict(host='unit-test.local',user='q',port=22,password='dummy-ssh',known_hosts='existing_known_hosts',connect_address=None,
            action='run',command='python3 -c safe',sudo=True,timeout=5,log=None,stdin_payload='IT_ARENA_WIFI_JSON_V1 '+json.dumps({'psk':DUMMY})+'\n')
        with patch.object(access.paramiko,'SSHClient',return_value=ssh),patch.object(access.sys,'stdin',io.StringIO(json.dumps(request))):
            self.assertEqual(access.main(),0)
        ssh.load_host_keys.assert_called_once_with('existing_known_hosts')
        self.assertIsInstance(ssh.set_missing_host_key_policy.call_args.args[0],access.paramiko.RejectPolicy)
        self.assertNotIn(DUMMY,ssh.exec_command.call_args.args[0]);self.assertTrue(stdin.getvalue().startswith('dummy-ssh\nIT_ARENA_WIFI_JSON_V1 '))
        stdin.channel.shutdown_write.assert_called_once();ssh.close.assert_called_once()
        # 기존 호출은 선택 필드가 없으므로 원래 sudo 암호 한 줄만 전달한다.
        request.pop('stdin_payload');stdin.seek(0);stdin.truncate(0)
        with patch.object(access.paramiko,'SSHClient',return_value=ssh),patch.object(access.sys,'stdin',io.StringIO(json.dumps(request))):
            self.assertEqual(access.main(),0)
        self.assertEqual(stdin.getvalue(),'dummy-ssh\n')

if __name__=='__main__':unittest.main()
