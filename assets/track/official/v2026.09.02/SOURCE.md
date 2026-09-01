# 공식 트랙 릴리스 `v2026.09.02`

- 출처: <https://github.com/MOSW626/istech-it-arena/releases/tag/v2026.09.02>
- 태그 커밋: `cb8fc14b1027c956b04cc297fa1454a65c956bfb`
- 배포 자산: `it_arena_track_v2026.09.02.zip`
- 크기: 883,631 bytes
- SHA-256: `6f74322703554e2dbe87598ce00a85332e1c6227353e82b1255180d2b12e12cb`
- 확인일: 2026-09-02

이 ZIP은 주최 측 이슈 #13의 급커브 노면과 갈림길 잔디 공백 수정을
반영한 공식 배포본입니다. 이전 `v2026.09.01`·`v2026.08.31` ZIP과 초기
전달본은 비교·감사 이력으로 별도 보존합니다. 이 폴더의 ZIP은 수정하거나
다시 압축하지 않습니다.

압축 항목 27개 중 파일은 24개이며, 경로 이탈·중복 경로 항목은
없었습니다. 주요 파일 해시는 다음과 같습니다.

| 파일 | SHA-256 |
|---|---|
| `track_gen.py` | `666153b9c56e5c73a9bed61930d187f93ef94fbab138115c43f2b61dc95cbe75` |
| `design_final.json` | `bd185e2c47263f759cea90d55668b653746f33d72da19c43e562e9f087b66cd8` |
| `README.md` | `7e22e5a9ae576b055bf7de26aebc697bda990250082139c65e4c5de352ece3aa` |
| `output_final/world.sdf` | `c21ce4384e8205a5438d62b88adf9682bcbe2bd547f7dfc001a692bd9f7f964f` |
| `output_final/scene.json` | `bf767c76778202ae49fba9db223cc34dd8d21c28a4d888e11971c354751092d3` |
| `output_final/venue_layout.dxf` | `c7e55a566ecb7838c1b985595dfdbe47dd315fb4fc652d5a9340002e1db6b342` |

릴리스 ZIP에는 이전판과 마찬가지로
`__pycache__/track_gen.cpython-314.pyc`가 함께 들어 있지만 실행 입력으로
사용하지 않습니다. 재생성·실행은 소스 `track_gen.py`와 명시된
JSON/SDF/출력물을 기준으로 합니다.
