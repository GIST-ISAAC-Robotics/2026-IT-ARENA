# v2026.09.15 문서 릴리스 대조 근거

[적용 요약](../../../../docs/track/OFFICIAL_V2026_09_15_DOC_UPDATE.md) · [공식 ZIP 출처](../../../../assets/track/official/v2026.09.15/SOURCE.md)

- `manifest.json`: 고정 태그/커밋, ZIP 크기·SHA-256, 변경 파일·통과 여부.
- `zip_files.json`: ZIP의 23파일 각각 이전/새 SHA-256. README 이외 22파일 동일.
- `runtime_sha256.json`: 이번 문서 적용 중 보존할 공식·실험 실행 폴더 60파일의 해시.
- `upstream/`: 새 릴리스·커밋·전체 트리 비교·변경 원문 두 문서·#12와 #6의 조회 스냅샷. 당시 자료이며 이후 수정은 자동 반영되지 않음.

공식 저장소에서는 `MANUAL.md`와 `track/README.md`만 변경됐습니다. ZIP에는 매뉴얼이 없으므로 `README.md` 한 파일만 바뀝니다. 두 원문은 Git blob ID와, ZIP은 GitHub 자산 digest 및 전 버전의 실제 바이트와 대조했습니다. ZIP은 추출/수정하지 않고 내부 파일을 읽어 비교했습니다.

## 이번 회귀 검사

- 첫 호출은 ROS 기본 환경만 불러와 `arena_vehicle_interface` 모듈을 찾지 못하고 수집 단계에서 실패했습니다. 프로젝트 `install/setup.bash`를 추가 적용한 재실행에서 **212개 통과**(14.31초). 코드 수정으로 해결한 문제가 아니라 실행 환경 누락입니다.
- `build_official_track.py --check`: 공식 9/14 ZIP·입력/29출력 해시 통과.
- `build_experimental_track.py --check`: 초기 ZIP·보존 23파일 및 본선/분기/그리드 정적 검사 통과.
- ROS 프로젝트 5개 빌드(12.6초), C++ `single_motor_equations` 1개, 공식 월드 엄격 SDF 검사 통과.
- 문서 적용 뒤 실행 폴더 60파일의 해시가 대조 시작 시점과 모두 같습니다. 기존 운영 검출기의 기본 매개변수 사용도 확인했습니다.
- 게시 준비에서 총 624개 변경 파일(약 31.85 MB)을 확인했습니다. 로컬 참조 249개 모두 존재하고, 50 MB 초과 파일·일반적인 토큰/개인 키 패턴의 검출 결과는 없었습니다. 이 검사는 모든 형태의 비밀정보 부재를 보증하지 않습니다.
- Git 인덱스와 실제 파일을 대조해 623개는 바이트 동일했습니다. 회신 검토 MD 한 개의 CRLF→LF 정규화만 달랐으며, 실제 게시 본문/증거 파일은 줄바꿈까지 보존합니다. 공백 검사도 통과했습니다.
- 새 주행·렌더링·실물 시험은 하지 않았습니다. 이전 종료 실패 자료는 그대로 보존합니다.

재현 명령(WSL, 저장소 루트):

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest tests src/arena_vehicle_interface/test -q
python3 scripts/build_official_track.py --check
python3 scripts/build_experimental_track.py --check
```

조회/다운로드 도구는 `scripts/audit_document_release.py`입니다. 기존 결과·공식 버전 폴더를 덮어쓰지 않으므로 이 보존 경로로 재실행하지 않습니다. 후속 릴리스에 사용할 때는 태그·기대 변경 범위를 확인하고 새 출력 경로를 지정합니다.
