"""선택형 offer의 원 모터축 관측. 구동 허가/운동 이력 입력을 만들지 않는다."""


def decode_motor_feedback(offer, status):
    """동일 offer를 변환한 MCU 상태에 원 취득 시각을 연결한다.

    host 시계는 상태 시각과 같은 추정이다. invalid/무관측을 유효 0으로 바꾸지 않는다.
    """
    raw = offer.get('motor_feedback_v1')
    if type(raw) is not dict or set(raw) != {'valid', 'sequence', 'capture_us', 'counts'}:
        raise ValueError('motor_feedback_schema')
    if type(raw['valid']) is not bool:
        raise ValueError('motor_feedback_valid_type')
    if any(status.get(k) != offer.get(k) for k in ('boot', 'generation')):
        raise ValueError('motor_feedback_status_identity')
    now = offer.get('status_now_us')
    if type(now) is not int or now < 0 or status.get('mcu_now_us') != now:
        raise ValueError('motor_feedback_status_time')
    sequence, capture, count = (raw[k] for k in ('sequence', 'capture_us', 'counts'))
    absent = sequence is None and capture is None and count is None
    if absent:
        if raw['valid']:
            raise ValueError('motor_feedback_missing_valid_sample')
        estimated = None
    else:
        if (type(sequence) is not int or not 0 <= sequence <= (1 << 63)-1
                or type(capture) is not int or not 0 <= capture <= now
                or type(count) is not int or not -(1 << 63) <= count <= (1 << 63)-1):
            raise ValueError('motor_feedback_value')
        estimated = status['now_us']*1000-(now-capture)*1000
        if estimated < 0 or estimated > status['host_receipt_ns']:
            raise ValueError('motor_feedback_mapped_time')
    return dict(schema='motor_feedback_observation_v1', boot=status['boot'],
                generation=status['generation'], clock_epoch=status['clock_epoch'],
                source_valid=raw['valid'], sequence=sequence, motor_counts=count,
                mcu_capture_us=capture, estimated_capture_ns=estimated,
                host_receipt_ns=status['host_receipt_ns'], time_basis=status['time_basis'],
                time_contract_verified=False, scope='observation_only_no_drive')
