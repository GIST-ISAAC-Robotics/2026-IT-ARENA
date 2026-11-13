> 실행 후 결과: 22검사·세 입력 비교 완료, 결합 후보의 새 양수 회복1개/3개. [결과와 한계](RESULT.md). 아래 준비/범위는 시험 당시 기준이다.

# r396 거리 한정 장착 마스크의 부분 성분 후보

상태: **준비만 완료, 미실행**. 공통 소스·과거 artifact·기본 설정·장치는 변경하지 않았다. r386 미채택 결론과 기존 실패는 유지한다.

## 한 가지 변경 원인

기존 `bounded_adjacent_structure`는 실제 masked ray 집합이 nominal sector 전체와 같아야 한다. 거리 한정 프로필에서는 sector 안의 먼 유효 반환을 보존하므로, 그 반환이 masked 성분을 나누면 이 조건을 만족하지 못한다. 후보는 **원 whole-sector 경로를 그대로 두고**, 그 membership 조건에서 실패한 부분 성분만 추가 근거로 검사한다.

- [staged/range_mask_component.py](staged/range_mask_component.py): 원 r386 `geometry/`의 함수 본문을 유지한 독립 후보. 공통 설치나 전역 교체 없음.
- [staged/profile_evidence.py](staged/profile_evidence.py): 명시 `mask_profile`, `motion_epoch`, `timing_verified`를 받아 원 MeasurementScan/현재 LocalScene의 각도·출처·순서·시각·epoch·status·반환 끝점을 대조한다. 거리 band 없는 기존 sectors만으로는 부분 허가를 만들지 않는다.
- 부분 masked 집합은 한 nominal sector의 진부분집합이어야 하고 다른 sector와 겹치지 않아야 한다. 연결 성분 바깥의 실제 return 중 적어도 하나가 그 nominal sector 내부이면서 자기 반환 거리 상한보다 멀어야 한다. 부분 경계가 실제 반환 때문에 생겼다는 근거다.
- source `unknown`과 `masked` 배열을 수정하지 않는다. 임의 subset·NaN unknown·근접 반환·열린 gap·다른 sector를 면제하지 않는다. 기존 10 mm/6°/50 mm/0.15 m/s, 이웃 반환 최소0.6 m, 시간/점유 조건을 유지한다. r386의 상대 이웃 거리보다 이웃0.6 m는 더 엄격한 기존 adjacent helper 조건이다.

전체-sector 동작에서는 새 profile 검증을 호출하지 않아 기존 통과/실패 결과를 그대로 유지한다. 따라서 입력 시각 검증의 기존 호출자 책임까지 전체-sector 경로에 새로 추가하는 후보가 아니다. 새 부분 경로에만 명시 증거가 필요하다. API 인자는 기존 입력 근거 전달이며 새 운용 설정값이 아니다.

## 세 입력과 분리된 비교

외부 원 r386 revision의 `bundle_sources.zip`을 읽어 새 WSL 폴더에 추출한 뒤 **geometry arm**만 사용한다. 현재 공통 트리나 다른 정책 시기의795개 기대값을 가져오지 않는다. final YAML에서 길이/폭/축거/조향 한계가 원 입력과 같음을 확인하고 track/radius/wheel width만 원 final 방식대로 명시한다.

[compare_three.py](compare_three.py)는 원 r377 current **123/125/222**, trace seq **606/610/804**만 계산한다.

| 비교 | 계산/역할 |
|---|---|
| baseline | 원 동결 r375 전체 현재 정책 결과를 원 r386 final 저장 before와 exact 대조 |
| helper-only | 기존 `.10` relative retry와 같은 core·union 질의·속도에 새 adjacent helper만 대입. region이 baseline 저장 진단과 exact여야 함 |
| distance+component | 원 r386의 단일 거리 제안과 최종 검사에 이 helper만 연결. 원 r386 파일은 불변, 원 r386 standalone 결과는 저장 after를 별도 표시 |

helper-only가 폭10 mm 단계에서 거절되면 거리나 목표를 새로 탐색하지 않는다. 결합 단계도 원 r386의 제안 한 번뿐이다. 123은 기존 r386 eligible 밖이므로 결합 단계가 계산을 추가하지 않을 수 있다. 222의 별도 순수 unknown 성분 등 다른 원 거절은 그대로 남을 수 있다. 원하는 양수 결과까지 규칙을 추가하지 않는다.

결합 연결은 해당 후보 객체의 함수/`ps` 참조만 try/finally로 잠시 바꾼다. 공통 모듈 전역 monkeypatch나 캐시를 설치하지 않는다. operating integration으로 사용할 형태가 아니다.

`queries_123/125/222.json`은 실제 `.10` union과 결합 최종 union의 모든 q좌표, current/previous 조향 소유자, 실제 분류, bracketing ray index/각도 및 원 광선의 값·시각·변환 좌표를 저장한다. 각 조향 원호를 별도로 한 번 구성한 union이 실제 final q와 **정확히 같아야** 기록한다. 좌표 반올림·허용오차로 소유자를 붙이지 않는다. 두 조향의 몸체/바퀴 합성 질의를 보존하며, 이 진단 수집 시간을 성능 개선 수치로 사용하지 않는다.

## 필요한 검사만

[test_component.py](test_component.py)의 합성 장면은 원 생성자/실제 helper를 사용한다. 예상 수는22개이며 아직 실행 결과가 아니다.

- 기존 whole-sector 수락·근접 반환·NaN·속도·몸체 범위 결과의 exact 동등성.
- 원거리 실제 반환이 끊은 부분 성분만 추가 수락, 원 입력 불변.
- profile/거리 band 없음, raw와 scene 불일치, unknown NaN/최소거리 미만, 근접 반환, 바깥 반환 없음, 다른/겹친 sector, epoch/시각 미검증/노후/누락, 속도·10 mm 위반 거절.

helper는 path TTL을 다루지 않는다. 세 입력 비교의 기존 core가 원 source/경로/현재 속도·조향 검사를 유지하며, 만료나 source 시각 갱신 경로를 새로 만들지 않는다. 합성 호출은 실물 마스크·주행·제어주기 검증이 아니다.

## 실행 인계와 중단 조건

명령은 [HANDOFF.md](HANDOFF.md)에 고정한다. WSL/컴파일/pytest/세 행 비교는 지정 담당자만 수행한다. 이번 준비에서는 Python AST·복사 원문·JSON/해시만 확인한다.

native build120 s → 독립 검사120 s → 세 입력 비교180 s 순서이며, 실패/시간초과/소스 변경 시 바로 중단한다. child PID·로그·종료코드는 새 receipt에 남기고 자기 프로세스 그룹만 종료한다. ROS/시뮬/장치/운영 토픽/새 기본값은 열지 않는다. 기존 r386 archive 및 결과 경로에는 쓰지 않는다.

검사가 통과해도 두 후보 모두 신규 양수 회복0이면 `discard_if_no_recovery=true`로 종료하고 다음 분기를 만들지 않는다. 결합만 개선되면 helper 단독 효과와 분리하며 r386 전체 채택으로 해석하지 않는다. 양수 회복은 저장 계산 결과일 뿐 코너 탈출/접촉/실차 속도 지속 증명이 아니다.

## 고정 입력 근거

- 원 bundle SHA-256: `bf53444bbed329082c904d046921656e0035f9c1715c18e6e11812fb48cbfd52`
- 원 manifest SHA-256: `cf3e0806871de1345a113aaa3f2c1552e6d0ad82f0fb2b1744f777d2a35f8a3f`
- 원 final rows SHA-256: `32da245a37792319de7d9033568e5f9ee906c91c1691d36aa31686a54a6d8c26`; 여기서 세 before/after만 [expected_final_three.json](expected_final_three.json)으로 복사했다.
- r377 trace SHA-256: `cb07e07a2548a31fcfbc0896afe086abb576b18de5d9f39e905172cf624a7277`.
- r386 candidate/budget와 원 `_gaps.py`는 [manifest.json](manifest.json)의 참조 해시로 고정한다.
