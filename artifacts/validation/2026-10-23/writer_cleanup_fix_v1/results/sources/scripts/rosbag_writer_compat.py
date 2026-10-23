"""Humble/Jazzy metadata와 명시적 writer 종료 오류 보존. ROS 초기화 없음."""


def close_writer(writer, errors):
    """호출자가 소유한 writer를 한 번 닫고 기존 오류에 종료 오류를 덧붙인다.

    재시도/멎음 해제/영구 저장 보장은 아니다. 호출자는 finally 한 곳에서만
    사용하고 반환값이 False이면 기록을 불완전 상태로 종료해야 한다.
    """
    if writer is None:
        return True
    try:
        writer.close()
    except Exception as error:
        errors.append('writer close failed: '+repr(error))
        return False
    return True


def check_writer_api(rosbag):
    missing = [name for name in ('open', 'write', 'create_topic', 'close')
               if not callable(getattr(rosbag.SequentialWriter, name, None))]
    if missing:
        raise RuntimeError('rosbag2_py SequentialWriter lacks required explicit API: '+','.join(missing))


def topic_metadata(rosbag, index, name, type_name):
    if type(index) is not int or not 0 <= index <= 65535:
        raise ValueError('bounded topic id required')
    fields = dict(name=name, type=type_name, serialization_format='cdr')
    # Jazzy exposes the id property; Humble's 4-string constructor has no id.
    # Do not catch all TypeError and retry: an unrelated API/type failure must remain visible.
    if hasattr(rosbag.TopicMetadata, 'id'):
        fields['id'] = index
    return rosbag.TopicMetadata(**fields)
