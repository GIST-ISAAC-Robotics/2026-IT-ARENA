"""기존 SSH 도구에 비밀 stdin과 비밀 없는 원격 helper 코드를 연결한다."""
import base64,io,json,shlex,sys
from pathlib import Path

def transport_request(request):
    wifi=request['wifi']
    if wifi.get('action') not in ('register','select','confirm'):raise ValueError('unsupported action')
    source=Path(__file__).with_name('jetson_wifi_remote.py').read_bytes()
    encoded=base64.b64encode(source).decode('ascii')
    code='import base64;exec(compile(base64.b64decode('+repr(encoded)+'),"jetson_wifi_remote.py","exec"))'
    return {k:request[k] for k in ('host','user','port','password','known_hosts','connect_address')} | dict(
        action='run',command='python3 -c '+shlex.quote(code),sudo=True,timeout=60,log=None,
        stdin_payload='IT_ARENA_WIFI_JSON_V1 '+json.dumps(wifi,ensure_ascii=True)+'\n')

def main():
    try:
        request=json.load(sys.stdin)
        sys.path.insert(0,str(Path(request['repository_root'])/'build/jetson_tools'))
        import jetson_access
        transported=transport_request(request)
        sys.stdin=io.StringIO(json.dumps(transported))
        return jetson_access.main()
    except Exception as error:
        # 입력/원격 stderr를 예외에 붙이지 않는다. 이미 예약된 타이머는 취소하지 않는다.
        print(json.dumps(dict(ok=False,error='transport_failed',error_type=type(error).__name__,
            rollback='if armed, left running; reconnect via previous Wi-Fi after the deadline')))
        return 1

if __name__=='__main__':raise SystemExit(main())
