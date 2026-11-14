# r501 동일 ARM native 비교 결과

2026-11-14, GPT-6 Astra. **r498 후보의 명령·판단·지정 진단시간을 분리한 상태가 같은 ARM baseline과 508/508 일치했다.** 한 쌍에서 p95·최대 지연과 100ms 초과 개수는 줄었지만 중앙값은 거의 같다. 후보에도 100ms 초과가 27개 남아 있으므로 전체 10Hz/실시간 완료로 판정하지 않는다. 공통 적용은 주 담당의 최종 판단까지 보류했다.

## 실행·무결성·동등성

주 담당이 r500 정상 회수 후 Jetson 단독 소유권을 넘겼다. `scripts/jetson_access.ps1`로 독립 경로 `/home/q/it-arena-r501-a`에 전송·실행했고 baseline508→candidate508→비교를 **단 한 쌍** 수행했다. 원 입력 순서·header(`native_v1`, `r396_v1`)를 유지하고 generation356 최초 reset 1회만 했다. 새 정책/과거 상태 주입, warmup, 반복 측정, cProfile은 없다. r493의 과거 정책이 아닌 준비 시점 `b59efcab` 공통과 검토한 r498 차이만 비교했다.

Python3.10.12·NumPy1.21.5·동일 native binary이며 각 variant를 새 Python process에서 실행했다. 모든 arena import가 해당 동결 variant 아래임을 검사했다. 원 inputs SHA `909c107fe700cf751e0a8c09b154bec09bff99140cd78b6b449014f8dfa829ac`, source manifest·동결 입력·원 native와 양쪽 native 사본의 불변을 확인했다. 판단/기억/TTL/보호 조건이나 기존 r500 설치는 수정하지 않았다.

| 비교 | 일치 |
|---|---:|
| 명령 exact | 508/508 |
| strict 전체 출력 | 507/508 |
| 지정 processing_ms만 분리한 판단 | 508/508 |
| strict 전체 상태 | 437/508 |
| 지정 cached processing_ms만 분리한 상태 | 508/508 |

원 strict 출력 차이는 seq1304의 `normal.partial_observation.surface_correspondence.processing_ms` **25.952359→25.851026ms** 한 건이다. 같은 cached fit 시간값이 상태에 남은 71건만 strict 상태에서 다르다. 새로운 차이 경로는 없고 다른 수치/진단 필드를 제외하지 않았다. 원 strict 결과를 성공 수치로 고쳐 쓰지 않았다.

## 측정된 계산 비용

| evaluator call 비용 | baseline | candidate |
|---|---:|---:|
| 중앙값 | 67.772ms | 67.827ms |
| 평균 | 72.167ms | 71.624ms |
| p95 | 107.490ms | 100.857ms |
| 최대 | 161.133ms | 151.367ms |
| 100ms 초과 | 39/508 | 27/508 |
| 첫 계산 | 51.523ms | 52.205ms |
| 생성자 | 156.653ms | 156.568ms |
| seq717 | 78.631ms | 78.254ms |
| seq1304 | 161.133ms | 151.367ms |
| seq1311 | 133.081ms | 124.313ms |

실제 evaluator 호출만 타이머로 감쌌으며 view 계측용 wrapper는 설치하지 않았다. 입력 불변 검사와 전체 상태 JSON/gzip 직렬화는 호출 사이, 타이머 밖에서 수행했다. 전체 harness wall time은 각각63.317/63.120초이고 이를 controller 처리율로 쓰지 않는다.

baseline 후 candidate 순서의 한 쌍이므로 scheduling·cache·온도에 따른 순서 효과가 남는다. 두 실행의 thread 환경변수는 동일하게 미지정, governor는 모두 `schedutil`이었다. CPU 주파수는 동적이며 고정하지 않았다. 읽을 수 있었던 thermal zone은 시작 약42.0~44.4°C, 종료 약43.7~45.9°C였고 zone2/3/4는 원 환경 도구의 읽기 실패를 그대로 남겼다. 별도 경쟁 시험·ROS·센서·구동은 실행하지 않았다.

따라서 **같은 저장 sequence에서 관측한 지연 감소**이며 보편적인 인과적 속도 향상, 전체 ROS 동시부하 마감, 센서 수신부터 출력까지의 10Hz, 실제 주행의 증거는 아니다. 같은 명령을 유지하면서 중복 계산을 제거한다는 채택 근거는 충족했고, 남은 지연은 별도 주 담당 우선순위로 다룬다. 반복 성능시험은 하지 않았다.

## 종료·회수·자원 반환

launcher PID6758, baseline6770, candidate6841, compare6903을 기록했다. 세 자식 모두 exit0·forced=false·residual_group=false이고 source_changes=[]다. 실행기 종료 후 launcher 소멸·관련 프로세스0·센서 포트0을 확인한 뒤 Jetson 소유권을 주 담당에게 반환했다. 실행 전후 전원·패키지·설정 변경, native 재빌드, 센서/ROS/구동은 없다.

회수 `/home/q/it-arena-r501-a-results.zip`의 원격·로컬 SHA는 모두 `f47cad3bb9fa7ac8ebc6d011cf194f090567c85565a44e9f771be0ce0177bad0`이다. ZIP·원시 출력은 기존 대용량 제외 정책을 유지한다.

근거: [간추린 결과와 환경·PID·자원 반환](summary.json), [원격 실행 영수증 로그](execute.log), [원격 진입/native SHA](remote_entry.log), [종료 후 자원 반환](resource_return.log), [원 source/input/native 기대 SHA](source_manifest.json), [채택 검토·실행자료](REVIEW.md). 로컬 상세 원본은 `remote_result/execution_receipt.json`, `remote_result/results/comparison.json`, 양쪽 `outputs.jsonl.gz`와 `summary.json`에 보존했다.
