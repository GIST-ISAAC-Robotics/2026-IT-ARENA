# selected_v2 source-only 생성 완료

2026-11-14, GPT. 선택 label: **`r538-r540-r541-r542-common-native110`**.

- [ZIP](selected_v2/runtime_handover_sources.zip): 소스734파일, ZIP735멤버(내부 manifest 포함). SHA256 **`92aa7039b188cf16ee31c3ecabe941793837ca9a157b9de9533198807feff9aa`**.
- [manifest](selected_v2/manifest.json) SHA256: **`167abb7d664cdebf2207aad4fecb9a7da3706a4b69fa6eca317bf6acd6fda6fa`**.
- 필수 `--steering-selection` 포함: 영점2811, 운영 count2600–2980, 중앙 상한 `0.28475436568260193 rad`, vehicle 내륜 상한 `0.30666234131856146 rad`. 두 공통 파일·선택 JSON·설치 안내를 함께 담았다. wheel각은 CAD 환산이다.
- 지정 guard SHA `0bc97fe6...`, steering `e3077...`, vehicle `dcb8...` 일치. 생성 중 수집 변경은 검출되지 않았다. CRC·멤버/manifest SHA·선택 동봉 검증 통과; 전체 시험 재실행은 하지 않았다.
- 실제 board/PI/포트는 입력하지 않았다. 기존 전체 설치 입력은 null이고 private board도 생성하지 않았다. **install_pending=true, 설치·Jetson 접속·flash·실물 출력0**.
- 기존 selected_v1 ZIP SHA `1358c38798a8cb286f93e2ba5f3aa28fc79ab24571d8dc0864d5b36401878e71` 및 r500 원본 ZIP/manifest 보존 확인. 측정 원자료·다른 세션 파일은 수정하지 않았다.

전체 핵심 SHA와 선택값은 [인수 영수증](selected_v2/acceptance_receipt.json), r500 대비 경로별 변경은 [delta](selected_v2/delta_from_r500.json), 생성 명령 출력은 [로그](selected_v2_prepare.log)에 기록했다. 이 묶음의 생성은 MCU/host 설치 또는 전체 실물 준비 완료가 아니다.
