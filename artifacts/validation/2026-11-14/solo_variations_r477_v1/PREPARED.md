# r477 preflight·source 동결 완료

r475 원 성공/1랩/정상 종료·입력 불변·소유 PID0 gate를 통과한 뒤, 배정된 Ubuntu-24.04에서 기존 preflight와 freeze를 **한 묶음, exit0**으로 수행했다. 빌드·전체 회귀·ROS init/spin/발행·Gazebo·주행·Jetson 작업은 하지 않았다. 현재 첫 조건의 root 검토를 기다린다.

- `execution_prepared.json`: `9018abc2a1b92e76f6a52416f350d133d1d6a9554b315f0c1570f075fc47b8e6`
- `execution_sources.zip`: `cd68335b79157965315d1f93305d7173bdae5b9047706dede3824e8b31d4e684`
- 동결 입력1,409개. Windows에서도 최종 파일/아카이브 SHA를 대조해 일치했다.
- 두 변형의 기존 launch description과 evaluator 시설 연결을 확인했다. 실제 launch setup/물리 실행이 아니며 새 native 검사는 하지 않았다. 동일 소스와 r475 설치/빌드 검증을 재사용한다.
- 같은 실행 셸의 종료 감사에서 `remaining_owned_preflight_work=[]`, session exit0이다. 이는 r477 preflight/freeze 소유 프로세스 범위이며 별도 전체 WSL 전역 감사가 아니다.

첫 root review 경로는 `cases/start_left20_yaw_plus3/root_execution_review.json`이다. 필드는 다음과 같으며 **이 문서 자체가 review 파일이나 실행 승인은 아니다.**

```json
{
  "root_reviewed": true,
  "case": "start_left20_yaw_plus3",
  "execution_limit": 1,
  "repeat_on_failure": false,
  "wsl_exclusive_owner_confirmed": true,
  "prepared_sha256": "9018abc2a1b92e76f6a52416f350d133d1d6a9554b315f0c1570f075fc47b8e6",
  "source_archive_sha256": "cd68335b79157965315d1f93305d7173bdae5b9047706dede3824e8b31d4e684"
}
```

root가 실제 검토 파일을 작성하고 시작을 지시하면 단독 소유 담당이 다음 명령으로 첫 조건을1회 실행한다. 현재는 실행하지 않았다.

```powershell
wsl.exe -d Ubuntu-24.04 -- bash '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-14/solo_variations_r477_v1/execution/execute_once.sh' start_left20_yaw_plus3
```

`wall_surface_inward10`은 첫 결과 후 root 판단까지 보류하며 review 파일도 만들지 않았다. 기존 속도/진행/충돌/노면/STOP/정상 종료 판정·8 sim초 무진행 STOP·최대80 sim초·조건별1회 기준을 유지한다.

근거: [준비 최종 결과](preflight_v1/result.json), [실행 출력](preflight_v1/windows_call.log), [회수](preflight_v1/completion.json), [실제 import/연결 결과](runtime_preflight/result.json), [동결 manifest](execution_prepared.json). 원 Windows 준비 기록과 r475/r460 원 자료는 보존했다. 공통 소스는 변경하지 않았다.
