# run48 wrapper 종료 오류 기록

- 실행 대상: `probe_encounter_prefix.py` (한 번 실행)
- 내부 결과: `run48_encounter_prefix/summary.json`의 `integrity_pass=true`, `sources_unchanged=true`; `run48_exit.txt` 내용은 `0`이며 Python 진단은 정상 완료했습니다.
- 로그: `run48_stdout.txt`, `run48_stderr.txt`, `run48_start.txt`, `run48_end.txt`에 기록했습니다. stderr는 비어 있습니다.
- 외부 실행 결과: WSL bash wrapper는 종료 코드 `2`를 반환했습니다.
- 실패한 wrapper 명령: `exit "$result"` (wrapper 15행)
- 실제 오류: `/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-10-26/m5_start_constraints_v1/run48_wrapper.sh: line 15: exit: 0\r: numeric argument required`
- 원인: PowerShell `Set-Content -Encoding ascii`가 wrapper 스크립트에 CRLF 줄바꿈을 기록하여, bash가 `result` 값 뒤의 carriage return을 숫자에 포함된 문자로 처리했습니다. 따라서 진단 프로그램의 종료 코드 `0`과 wrapper의 종료 코드 `2`는 서로 다른 상태입니다.
- 임시 wrapper는 결과 회수 뒤 삭제했습니다. 현재 보존된 wrapper 원본은 없습니다. 위 명령과 오류는 실행 도구 출력에서 확인한 기록입니다.
- 실행 재시도는 하지 않았습니다.
