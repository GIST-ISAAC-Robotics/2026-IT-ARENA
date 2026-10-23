"""ROS 고장 시험 자식에서만 사용하는 수신기 거절 진단. 운영 launch 미연결.

원래 _ingest를 정확히 한 번 호출하고 반환/예외를 유지한다. 새 invalid_encoder
고장 전환 뒤에만 실제 인수와 전환 직전 상태를 기록한다. 지연 원인이 OS, DDS,
콜백, 발행기 중 어디인지는 이 기록만으로 확정할 수 없다.
"""
import json
import time


def install_test_encoder_probe(receiver_type, emit=None):
    """별도 시험 프로세스용. 복구 함수 반환; 중복 설치 거절."""
    original = receiver_type._ingest
    if getattr(original, '_pipeline_rejection_probe', False):
        raise RuntimeError('test encoder rejection probe already installed')
    if emit is None:
        def emit(row):
            print('MCU_ENCODER_REJECTION=' + json.dumps(row, allow_nan=False), flush=True)

    def traced(receiver, now_us, sequence, capture_us, counts):
        before = (receiver.state.value, receiver.reason, receiver.generation,
                  receiver.feedback, receiver.last_control_us, receiver.armed)
        accepted = original(receiver, now_us, sequence, capture_us, counts)
        if not accepted and before[5] and receiver.reason == 'invalid_encoder':
            # 판정이 끝난 뒤의 진단이며 값을 고치거나 신선도를 갱신하지 않는다.
            try:
                emit(dict(type='test_encoder_rejection', scope='validator_child_only',
                    received_wall_s=time.monotonic(), now_us=now_us, sequence=sequence,
                    capture_us=capture_us, counts=list(counts),
                    age_us=now_us-capture_us,
                    feedback_timeout_us=receiver.config.feedback_timeout_us,
                    state_before=before[0], reason_before=before[1], generation_before=before[2],
                    last_accepted_feedback_before=before[3], last_control_us_before=before[4],
                    state_after=receiver.state.value, reason_after=receiver.reason,
                    generation_after=receiver.generation,
                    interpretation='actual rejected call; transport and scheduling root cause not measured'))
            except Exception:
                # 기록 실패가 원래 제어 판정/예외를 바꾸지 않도록 한다.
                pass
        return accepted

    traced._pipeline_rejection_probe = True
    receiver_type._ingest = traced

    def restore():
        if receiver_type._ingest is not traced:
            raise RuntimeError('receiver implementation changed after probe installation')
        receiver_type._ingest = original
    return restore
