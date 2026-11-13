import argparse
import pytest
from sensor_stream_selection import add_stream_arguments, validate_stream


def test_legacy_stream_default_and_runtime_explicit():
    p=argparse.ArgumentParser();add_stream_arguments(p,'/hardware_preview/c1/native')
    old=p.parse_args([])
    assert (old.ros_domain,old.topic)==(216,'/hardware_preview/c1/native')
    current=p.parse_args(['--ros-domain','220','--topic','/hardware_runtime/c1/native'])
    result=validate_stream(current.ros_domain,current.topic,{'ROS_DOMAIN_ID':'220','ROS_LOCALHOST_ONLY':'1'})
    assert result['topic']==current.topic and not result['frame_relabelled'] and not result['source_timestamp_rewritten']


@pytest.mark.parametrize('domain,topic,environment',[
    (220,'/hardware_runtime/c1/native',{'ROS_DOMAIN_ID':'216','ROS_LOCALHOST_ONLY':'1'}),
    (220,'/hardware_runtime/c1/native',{'ROS_DOMAIN_ID':'220','ROS_LOCALHOST_ONLY':'0'}),
    (220,'/drive/safe',{'ROS_DOMAIN_ID':'220','ROS_LOCALHOST_ONLY':'1'}),
    (220,'/hardware_runtime/c1 native',{'ROS_DOMAIN_ID':'220','ROS_LOCALHOST_ONLY':'1'}),
    (True,'/hardware_runtime/c1/native',{'ROS_DOMAIN_ID':'True','ROS_LOCALHOST_ONLY':'1'}),
    (233,'/hardware_runtime/c1/native',{'ROS_DOMAIN_ID':'233','ROS_LOCALHOST_ONLY':'1'})])
def test_stream_mismatch_rejected_before_ros_device(domain,topic,environment):
    with pytest.raises(ValueError):validate_stream(domain,topic,environment)
