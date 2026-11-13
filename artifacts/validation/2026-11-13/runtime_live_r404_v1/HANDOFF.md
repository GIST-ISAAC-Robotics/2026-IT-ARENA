# r404 실행 인계

r401 pure6·설정대조·실제ROS1은통과했다. Jetson/센서포트는실행담당만소유한다. bundle SHA256 `2b326451f69063cf6142a72d34660476ea453b29a22eece8b8baf33d2387f013`, 650소스. 기존정책r377이며r405/r406은미포함이다.

기존jetson_access.ps1로 bundle.zip와remote_run.py를새 `/home/q/r404-runtime-inputs-20261113.zip`, `/home/q/r404_runtime_inputs_20261113.py`로전송한다. 기존목적지덮어쓰기금지. 다음명령을한번만실행한다.

```bash
source /opt/ros/humble/setup.bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH="/home/q/it_arena_imu_20261111/tools_venv/lib/python3.10/site-packages:${PYTHONPATH:-}"
python3 -B /home/q/r404_runtime_inputs_20261113.py --archive /home/q/r404-runtime-inputs-20261113.zip --sha256 2b326451f69063cf6142a72d34660476ea453b29a22eece8b8baf33d2387f013 --root /home/q/it-arena-r404-20261113-a
```

빌드120초/import15초/live85초,외부360초상한이며실센서수집20초다. 종료신호뒤supervisor정리시간을허용하며SSH도구timeout을원격종료로간주하지않는다. 실패시자동재시험/flash/출력host추가없음.

`/home/q/r404-runtime-inputs-20261113-results.zip`을 local remote_result에회수한다. execution.json·actual/supervisor.json·node/summary.json·before/after진단비활성·producer각summary·source650불변·자기PID/포트반환을확인한다. 원시배열은대화출력하지않고Git제외폴더에보존한다.

무엔코더·무actuationhost이므로양수주행0은예상이다. 양쪽ROS명령stage수신과각node입력/종료만확인하며실차주행/최신계획CPU성능으로평가하지않는다.
