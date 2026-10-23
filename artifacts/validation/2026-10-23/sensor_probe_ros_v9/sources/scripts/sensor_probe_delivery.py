"""시험용 송신 목록과 수신 CDR 비교. 저장 bytes를 수정하지 않는다.

별도 프로세스 ROS 전달에서 관측한 최대 3 B의 끝 정렬 0 padding만 명시적으로
구분한다. 원래 payload prefix는 SHA-256/크기가 정확히 같아야 한다.
"""
from collections import Counter
import hashlib


class DeliveryInventory:
    def __init__(self, transmissions):
        self.expected = Counter()
        self.received = Counter()
        self.exact = Counter()
        self.aligned = Counter()
        self.unexpected = Counter()
        if not 1 <= len(transmissions) <= 10000:
            raise ValueError('bounded transmission list required')
        for row in transmissions:
            topic, size, digest = row['topic'], row['bytes'], row['sha256']
            if (type(topic) is not str or not topic or len(topic) > 256
                    or type(size) is not int or not 4 <= size <= 4*1024*1024
                    or type(digest) is not str or len(digest) != 64
                    or any(c not in '0123456789abcdef' for c in digest)):
                raise ValueError('invalid transmission identity')
            self.expected[topic, size, digest] += 1

    def observe(self, topic, payload):
        if not isinstance(payload, bytes) or not 4 <= len(payload) <= 4*1024*1024+3:
            raise ValueError('bounded received bytes required')
        key = topic, len(payload), hashlib.sha256(payload).hexdigest()
        if key in self.expected:
            self.received[key] += 1
            self.exact[topic] += 1
            return
        candidates = []
        if len(payload) % 4 == 0:
            for trim in (1, 2, 3):
                size = len(payload)-trim
                if payload[-trim:] == b'\0'*trim:
                    k = topic, size, hashlib.sha256(payload[:size]).hexdigest()
                    if k in self.expected:
                        candidates.append(k)
        if len(candidates) == 1:
            self.received[candidates[0]] += 1
            self.aligned[topic] += 1
        else:
            # No guess when more than one transmitted prefix could match.
            self.unexpected[key] += 1

    def result(self):
        def export(counter):
            return [dict(topic=t, bytes=n, sha256=d, count=c)
                    for (t, n, d), c in sorted(counter.items())]
        missing = self.expected-self.received
        excess = self.received-self.expected
        return dict(sent=sum(self.expected.values()), matched=sum(self.received.values()),
                    exact_by_topic=dict(self.exact), trailing_zero_alignment_by_topic=dict(self.aligned),
                    missing=export(missing), excess=export(excess), unexpected=export(self.unexpected),
                    every_received_payload_matched=not self.unexpected,
                    all_sent_observed_exactly_once=not (missing or excess or self.unexpected),
                    full_wire_bytes_equal=not (self.aligned or self.unexpected),
                    acquisition_time_verified=False, physical_sensor_verified=False)
