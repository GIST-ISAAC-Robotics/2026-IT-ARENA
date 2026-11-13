# 생성 프로필의 기준값 분리

첫 검사는25통과/5실패였다. `build()`가 반환하는 프로필/모터 JSON과 registration의 기대값이 같은 dict를 참조해, 호출자가 출력 프로필을 바꾸면 검증 기준도 함께 바뀌었다. 파일로 저장한 뒤 따로 읽는 CLI와 별개로 메모리 API의 계약 위반이다.

원 소스와 manifest는 `verify_v1/source_at_failure`에 보존했다. registration 소유 기대값을 deepcopy하여 분리하는 한 줄을 수정했고 새 manifest는 `source_manifest_r2.json`이다. 허용값/실물 가정/검사 기준은 바꾸지 않았다. 동일30검사 후 생성/check를 한 번 수행한다.
