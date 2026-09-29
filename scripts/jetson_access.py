"""Windows DPAPI wrapper의 stdin을 사용하는 시험용 SSH/SFTP 도구. 비밀 출력 금지."""
import json
import ipaddress
from pathlib import Path
import shlex
import sys
import time
import socket

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'build/jetson_tools'))
import paramiko


def main():
    request = json.load(sys.stdin)
    client = paramiko.SSHClient()
    client.load_host_keys(request['known_hosts'])
    client.set_missing_host_key_policy(paramiko.RejectPolicy())
    connection = None
    try:
        if request.get('connect_address'):
            # 명시적 숫자 주소로 운반 경로만 선택한다. 키 검증 이름은 기존 host 그대로다.
            address = str(ipaddress.ip_address(request['connect_address']))
            connection = socket.create_connection((address, request['port']), timeout=10)
        client.connect(request['host'], port=request['port'], username=request['user'],
                       password=request['password'], look_for_keys=False, allow_agent=False,
                       timeout=10, auth_timeout=15, banner_timeout=15, sock=connection)
    except BaseException:
        client.close()
        if connection:
            connection.close()
        raise
    client.get_transport().set_keepalive(20)
    try:
        if request['action'] in ('put', 'get'):
            with client.open_sftp() as sftp:
                if request['action'] == 'put':
                    try:
                        sftp.stat(request['destination'])
                    except FileNotFoundError:
                        pass
                    else:
                        raise FileExistsError('Remote destination already exists')
                    sftp.put(request['source'], request['destination'])
                else:
                    target = Path(request['destination'])
                    if target.exists():
                        raise FileExistsError('Local destination already exists')
                    target.parent.mkdir(parents=True, exist_ok=True)
                    sftp.get(request['source'], str(target))
            print(json.dumps({'action': request['action'], 'transferred': True}))
            return 0
        command = request['command']
        if request.get('sudo'):
            command = 'sudo -S -p ' + shlex.quote('') + ' -- bash -lc ' + shlex.quote(command)
        stdin, stdout, _ = client.exec_command(command, get_pty=False)
        if request.get('sudo'):
            stdin.write(request['password'] + '\n')
            stdin.flush()
        stdin.channel.shutdown_write()
        channel = stdout.channel
        deadline = time.monotonic() + request['timeout']
        log = None
        if request.get('log'):
            path = Path(request['log'])
            path.parent.mkdir(parents=True, exist_ok=True)
            log = path.open('xb')
        try:
            while True:
                for ready, recv in ((channel.recv_ready, channel.recv),
                                    (channel.recv_stderr_ready, channel.recv_stderr)):
                    if ready():
                        data = recv(65536)
                        sys.stdout.buffer.write(data)
                        sys.stdout.buffer.flush()
                        if log:
                            log.write(data)
                            log.flush()
                if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                    return channel.recv_exit_status()
                if time.monotonic() > deadline:
                    raise TimeoutError('SSH command timed out; inspect remote process before retrying')
                time.sleep(.02)
        finally:
            if log:
                log.close()
    finally:
        client.close()


if __name__ == '__main__':
    raise SystemExit(main())
