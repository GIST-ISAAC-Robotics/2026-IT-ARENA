# r388 r1 고정 인계

원r0 실패를 보존한 fresh 준비본이다. 공통 적용·실행은 아직 하지 않았다. 지정 실행자에게 자원을 배정한 뒤 아래 명령1회를 실행한다.

- `bundle_sources.zip`: 2,730,324bytes, SHA256 `f0d6b8496b02a41686adef66f62114e21b98c77d928cc9e780f8a35524a25a40`
- `source_manifest.json`: `1e1c663af5e5d0d02653438495d4b706954b076f01533e5e4b086e7e9ace99e5`
- `execute_isolated.py`: `5d24da0da1e318cea7c36cd97a0e2847b31bcc54644e13a6c1587f1bb00c436d`
- `geometry_contract.patch`: `65bf8f0d00ae26385815e63cf1255401dd21c3abf5d8f3953502ecec6ef044af`
- bundle542파일, 원r377소스256개·staged9개. 원r0 archive SHA는 `223c2df288f25f2147b78ef26a47bc235ee7929cd3be3b346ec43fae0bab265e` 그대로다.

```bash
python3 -B '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/final_geometry_contract_r388_v1/revision_r1/execute_isolated.py' --archive '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/final_geometry_contract_r388_v1/revision_r1/bundle_sources.zip' --sha256 f0d6b8496b02a41686adef66f62114e21b98c77d928cc9e780f8a35524a25a40 --root "$HOME/it-arena-r388-r1-geometry-20261113-a"
```

새 루트/새 출력만 허용한다. native1회, 회귀1회(49개 예상), frozen/legacy/explicit 각1회, aggregate1회이며 각단계120s 제한이다. 기존 실패원/번들/루트/ZIP은 건드리지 않는다. 실패하면 후속단계 중단하고 자기 자식 group만 종료한다.

새 결과: `$HOME/it-arena-r388-r1-geometry-20261113-a/r388/results_native_v1/`. 자동 회수 ZIP은 **revision_r1의** `bundle_sources-results.zip`이며, r0의 동명 ZIP과 경로가 다르다. revision_r1/remote_result로 회수한다. `execution.json`, 단계 로그, `comparison.json`, explicit의 `geometry_precondition.json`, arm별 전체 결과와 실제 질의 배열을 보존한다.

허용 변경은 기하3필드뿐이다. `wheelbase_m/max_steering_rad` 저장값·원 출력/시각·계보는 불변이어야 한다. 원 float32 식을 적용한 exact비교가 실패하면 수치를 기록하고 중단한다. tolerance 추가나 steering/YAML 값 동시 변경은 허용하지 않는다. 원r0 pipeline 실패는 r1 성공 여부와 별도로 계속 보존한다.

준비자는 Windows에서 원문/저장결과 감사·순수 CAD 산술 확인·AST/ZIP/해시만 수행했다. 제어 현재검사·테스트·native 빌드·WSL/Jetson·ROS·장치 실행은0이다. 이 인계문은 archive 생성 후 해시를 기록한 외부 문서이며 archive inventory에 포함되지 않는다.
