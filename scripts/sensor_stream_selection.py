"""수신 전용 스트림의 명시 domain/topic. 픽셀·광선·취득시각은 변경하지 않는다."""
import re


def add_stream_arguments(parser, default_topic):
    parser.add_argument('--ros-domain', type=int, default=216,
                        help='ROS_DOMAIN_ID와 일치해야 하는 명시 격리 domain')
    parser.add_argument('--topic', default=default_topic,
                        help='관측 전용 /hardware_preview/ 또는 /hardware_runtime/ 토픽')


def validate_stream(domain, topic, environment):
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('explicit ROS domain in 0..232 required')
    if environment.get('ROS_LOCALHOST_ONLY') != '1' or environment.get('ROS_DOMAIN_ID') != str(domain):
        raise ValueError('localhost ROS environment must match declared domain')
    if (type(topic) is not str or not re.fullmatch(r'/[A-Za-z_]\w*(?:/[A-Za-z_]\w*)*', topic)
            or not topic.startswith(('/hardware_preview/', '/hardware_runtime/'))):
        raise ValueError('explicit isolated sensor topic required')
    return dict(ros_domain=domain, topic=topic, localhost_only=True,
                frame_relabelled=False, source_timestamp_rewritten=False, actuation_output=False)
