"""Humble/Jazzy 토픽 metadata 차이만 분리. 장치 접근/ROS 초기화 없음."""


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
