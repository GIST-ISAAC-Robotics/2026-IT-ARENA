# 공통 선택 정책 계측 준비와 기존 기록 도구의 호환 실패

11/12 r182 원439판단의 입력/출력·reset·기타 기록은 바이트 그대로 보존하고 헤더의 `guard_observation_policy=selected_r181`만 명시한 파생 입력을 만들었다. 원본은 덮어쓰지 않았다. 원/파생/본문 해시는 `derived_input.json`에 있다.

116파일 묶음·정적 의존 폐포112파일과 archive/개별 해시를 로컬에서 확인했다. 그러나 PC의 기존 `profile_follow_trace.py`는 `current_path_recheck_policy`/`delivery_delay_profile` 이벤트를 모르기 때문에 **`unknown trace event`로 실패**했다. 계산 결과 불일치나 속도 측정 실패로 바꾸어 해석하지 않는다. 이 실패 전에 Jetson에 묶음을 전송하거나 실행하지 않았다.

원 `pc/report.json`, `pc.log`, `bundle_manifest.json`, `bundle_audit.json`, Git 제외 archive와 파생 입력을 보존한다. 후속은 알려진 설정 메타데이터를 기록 손실 없이 읽도록 재생기를 보완하고, 출력 동등성/기록 무결성을 확인한 뒤 새 묶음으로 계측하는 것이다. 기록된 지연 주입을 실제 계산 시간으로 재현하거나 계측에 더하지 않는다.
