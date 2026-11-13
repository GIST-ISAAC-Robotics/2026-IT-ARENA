"""현재 원천 센서의 좌표계 이름. 이름 자체가 장착/축 교정 증거는 아니다."""
REGISTERED_SOURCE_FRAMES = ('laser_frame', 'c1_unmounted_preview')


def registered_source_frame(value):
    # 실제 운동 변환에서는 MotionConfig의 선택값과도 일치해야 한다.
    # 하류 관측기는 이미 base_link로 투영한 장면의 원 출처를 보존한다.
    return type(value) is str and value in REGISTERED_SOURCE_FRAMES
