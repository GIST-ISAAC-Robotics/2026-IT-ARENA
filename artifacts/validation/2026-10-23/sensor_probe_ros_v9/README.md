# 선택형 PC SHM 16 MiB — 5입력 기록 제한 통과

관련 138회귀·1.76 s, ROS/감사 exit 0·소스 불변. 0.05 s 회차 spin 대기·0.25 s drain은 v7과 같고 전송은 **명시 SHM 단독 16 MiB**다. Jazzy에서 LOCALHOST가 XML에 transport를 추가하는 동작을 피하려고 이 시험 프로세스에서만 SYSTEM_DEFAULT discovery를 설정했다. 격리는 환경변수 이름이 아니라 두 프로세스의 builtin false·SHM만/16777216 적용 검사로 확인했다.

- Image 20/20(1280×720 RGB/BGR·행 padding 교대), CameraInfo 20/20, IMU 20/20, scan 20/20, 단일 모터축 20/20.
- Image/IMU/scan/모터축 80표본은 송신 bytes 해시와 그대로 같았다. CameraInfo 20표본은 송신 prefix 전체 동일+끝 정렬용 0 세 바이트 추가로 구분했다. 예상하지 못한 내용·누락·중복 0.
- callback에서 받은 CDR와 bag/receipt 대응은 모든 bytes에 대해 원래 엄격한 무결성 검사를 통과했다. 기록기를 바꾸거나 padding을 삭제해 저장하지 않았다.
- 헤더 0·역전은 보존했고 제어/명령 구독·매개변수 요청·운영 재생/구동 허가는 없다. 마지막 SIGINT 정상 종료, child 종료를 확인했다.

기록기 원문은 v7/v8과 같고, 추가 계측 wrapper는 이 폴더에만 있다. 순수 전달 진단의 inspector와 기존 replay XML/검사 함수를 재사용했지만 Humble 재생 설정/검사를 수정하지 않았다. v7 Image 17/20·v8 10/20 실패와 sender/receiver 시각/bytes 경계는 그대로 남는다.

한 번의 짧은 PC 시험이다. 60 fps·장시간·Jetson/Humble·실물 카메라·동시 주행 부하·네트워크 원격 운용 보장이 아니다. SHM은 같은 호스트 전용이며 모든 센서가 해당 설정에 참여해야 한다. 일반 실행/전역 설정이나 최소 기록 묶음에 자동 적용하지 않았다. 이후 실물 첫 기록에서 생산자 설정/원천 순번과 함께 재확인한다.
