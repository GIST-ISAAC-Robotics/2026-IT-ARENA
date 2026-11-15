# 11/14 제작 기준 STEP 비교 산출물

[기술 보고서](../../docs/hardware/PACKAGING_CAD_COMPARE_2026_11_14.md)에 판정과 적용 범위를 정리했다. 최종 STEP은 [공개 전체 설계 릴리스](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA/releases/tag/final-design-2026.11.14)에서 받는다. 비교 코드·JSON·그림은 Git에 보존하고 B-rep/mesh 캐시는 다시 생성한다.

- 입력은 [수신 사본의 정보](source.json)와 [직전 시뮬 기준 사본의 정보](../cad_final_2026_11_13/source.json)로 식별한다. 각각 Downloads의 수신본 및 11/13 보존 원본과 해시를 대조했다.
- [상위 부품 대조](part_diff.json)는 같은 이름을 가까운 배치 경계끼리 매칭한다. 이름이 바뀌거나 합쳐진 형상은 별도 [솔리드 경계](solid_breakdown.json)와 단면으로 확인한다. `same_metrics`는 경계/체적/표면적/면수/솔리드 수가 일치한다는 뜻이다.
- [B-rep 단면](section_comparison.json), [평면 광선 수치](ray_comparison.json), [측면 그림](c1_scan_height_comparison.png), [공칭면 그림](c1_scan_section_comparison.png).
- 재현 순서: 이 폴더에서 `extract_step.py`로 사본을 읽고 `compare_step.py`, `solids.py`, `scan_compare.py`, `render_parts.py`, `plot_review.py`를 실행한다. 추출기는 이전 결과를 덮어쓰지 않도록 새 출력 디렉터리에서만 실행한다.

## 11/15 복구와 재생성

정리 과정에서 이 폴더의 전체 `*` 제외 규칙 때문에 보조 자료까지 삭제된 오류를 발견했다. 당시 생성 입력에서 Python8파일과 원 README·source.json을 복구했으며 삭제 전 바이트 길이와 일치했다. 삭제 전 SHA-256은 없으므로 원바이트 일치가 독립 검증됐다고 주장하지 않는다. 두 STEP은 기존 SHA-256과 일치하는 보존본을 사용했고 JSON·그림은 같은 코드로 재생성했다. 공칭134.6mm에서 신규/소실 가림 방향0·공통 교차거리 변화0을 다시 확인했다. 새 실차 측정이 아니다. [복구 검증](restoration_report.json).

재생성 입력은 현재 공개 릴리스의11/14 STEP과 [비공개 원본 보관소](https://github.com/GIST-ISAAC-Robotics/2026-IT-ARENA-evidence)의 `private-project-deliverables.zip`에 있는11/13 STEP이다. 원래 두 폴더의 `source.step`으로 각각 풀고,11/13 `extract_step.py`·`analyze_geometry.py`로 이전 B-rep을 만든 뒤 위 순서로 새 비교를 실행한다. `scan_compare.py`는 저장소의 `src/arena_autonomy`를 필요로 한다. 검증 환경은 Python3.14, cadquery-ocp8.0.1.1.0, NumPy2.5.4, Matplotlib3.11.2다.
