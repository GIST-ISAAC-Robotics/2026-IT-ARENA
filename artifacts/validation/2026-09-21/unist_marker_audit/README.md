# UNIST Pinocchio 등록 마커의 표준 검출 시험

검사일: 2026-09-21. 대상은 [공식 이슈 #1 등록 댓글](https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-5755605009)의 1254×1254 PNG이다.

## 결론

**표준 마커로 안정적으로 인식되는 규격은 확인하지 못했다. 단, AprilTag `tag16h5` 검출기는 일부 전처리 조건에서 2비트를 보정해 ID 8로 반환했다.** 따라서 ‘어떤 검출기에서도 절대로 인식되지 않는다’는 표현도 부정확하다. 원본 그림을 정식 ID 8 마커라고 단정할 근거는 없으며, 부분적인 오인식 가능성으로 다뤄야 한다. 제작자의 의도와 자체 인식기는 미확인이다.

## 대상과 재현

- 원본: `unist_original.png`
- 원본 주소: https://github.com/user-attachments/assets/257f21bd-fbba-4143-be94-7338c9eefde8
- SHA-256: `4385f53c9b0ad097299a6d2747ef4d7aafbf4bdaf268baa02390d195de9a9c0b`
- 실행: Python 3.12, opencv-contrib-python-headless 5.0.0.93, numpy 2.5.3, pupil-apriltags 1.0.4.post11, zxing-cpp 3.1.1.
- 해당 패키지를 별도 환경에 설치하고 `python audit.py`, `python followup.py`로 재현한다. 운영 ROS/OpenCV 환경은 변경하지 않았다.
- `report.json`은 최초 전체 시험, `followup.json`은 예외 검출과 대조군의 추가 확인이다. 최초 결과를 덮어쓰지 않았다.

## 실제 시험 결과

| 범위 | 결과 |
|---|---|
| OpenCV의 고유 사전 22개 | 2,464조건 모두 미검출. 정상 ID 0 대조군 88/88 통과 |
| 별도 AprilTag C 검출기 8계열 | tag16h5만 일부 조건에서 ID 8·Hamming 2 반환. 나머지 7계열 미검출 |
| ZXing-C++ 기본 전체 바코드 검사 | 28조건 모두 미검출. QR·Data Matrix·Aztec 정상 대조군 3/3 통과 |

OpenCV 사전은 ArUco 4×4/5×5/6×6/7×7 각각 50/100/250/1000, ARUCO_ORIGINAL, ARUCO_MIP_36h12와 AprilTag 16h5/25h9/36h10/36h11이다. 대소문자 별칭은 중복 집계하지 않았다.

입력 28종은 원본·50%·25% 크기의 회색조/이진화 각각 4방향 회전(24종), 원본 크기 명암 반전 4방향(4종)이다. 각 사전마다 기본·검은 테두리 2셀·AprilTag 사각형 검출·후보 거리 0.02와 임계값 창 확대의 4설정을 적용했다. 마지막 설정은 오류 보정 허용치를 늘린 것이 아니다. 기본 OpenCV에서도 약 961,870.5 px²의 외곽 사각형 후보는 검출되어 거부 목록에 남았다. 외곽 사각형 존재와 유효 ID 해독을 구분한다.

별도 AprilTag 계열은 tag16h5, tag25h9, tag36h11, tagCircle21h7, tagCircle49h12, tagCustom48h12, tagStandard41h12, tagStandard52h13이다. `quad_decimate=1.0`이며 래퍼의 기본 오류 보정은 최대 2비트다.

최초 C 생성기 대조군은 5/8만 통과했고 일반 사각형 3계열은 실패했다. 생성 경로 문제의 원인은 확정하지 않았다. 이 세 계열은 OpenCV로 규격에 맞게 생성한 독립 ID 0 대조군으로 다시 검사해 **모두 ID 0·Hamming 0**을 확인했다. 따라서 최종적으로 8계열 각각 정상 대조군이 확보됐다. 이 보완은 `followup.json`에 있으며 최초 5/8 결과도 보존한다.

## ID 8 예외를 분리한 추가 시험

tag16h5에 대해 오류 보정 한도 0/1/2비트 × 크기 100/75/50/37.5/25% × 회색조/이진화로 30조건을 확인했다.

- 0비트, 1비트 허용: 각각 10조건 전부 미검출.
- 2비트 허용: 50% 이진화, 37.5% 회색조, 37.5% 이진화에서만 ID 8·Hamming 2. 10조건 중 3조건이다.
- 원본 크기 회색조/이진화는 모든 보정 한도에서 미검출.
- 규격대로 생성한 진짜 tag16h5 ID 8은 보정 한도 0/1/2 모두 **ID 8·Hamming 0**.
- 최초 검사의 50% 이진화는 0/90/180/270° 모두 동일 ID 8·Hamming 2였다.

`genuine_tag16h5_id8.png`는 정상 비교용 이미지이며 등록 원본을 보정하거나 대체한 파일이 아니다. Hamming 2는 검출기가 읽은 16비트에서 코드와 2비트 차이가 났다는 뜻이며, 원본 그림 전체가 정상 마커와 2칸만 다르다는 뜻은 아니다.

## 해석과 한계

검사한 표준 사전에 안정적인 일치가 없다는 결과다. 전 세계의 모든 마커 체계·사용자 정의 사전·그림 템플릿 인식기를 배제하지 않는다. ARToolKit 사용자 정의 패턴 등 별도 등록이 필요한 방식은 실행하지 않았다. 다른 그림처럼 따로 학습/등록하면 인식 가능한가와 기존 표준 ID로 읽히는가는 별개다. 실물 인쇄·카메라 거리·조명·동작 중 신뢰도를 시험하지 않았다.

규정 위반 여부는 판정하지 않았다. 등록 이슈는 이미지 또는 ArUco ID 공유를 허용하며 제작자의 규격·의도는 댓글에 없다. 외부 댓글 게시·팀 마커 변경·운영 검출기 변경을 수행하지 않았다.

## 참고한 1차 자료

- [OpenCV 사전과 검출기](https://docs.opencv.org/4.x/de/d67/group__objdetect__aruco.html)
- [AprilTag 공식 구현](https://github.com/AprilRobotics/apriltag)
- [ZXing-C++ 지원 형식](https://github.com/zxing-cpp/zxing-cpp)
