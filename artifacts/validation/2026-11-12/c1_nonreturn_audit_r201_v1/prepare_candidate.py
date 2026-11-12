"""실행 중인 주행 소스와 분리해 변환 후보를 준비한다."""
import hashlib
import json
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
source=ROOT/'src/arena_vehicle_interface/arena_vehicle_interface/c1_native_preview.py'
text=source.read_text('utf8')
old='''            # 품질0·거리0은 자유 공간으로 만들지 않는다. 원 정수는 별도 raw에 보존한다.
            ranges.append(distance/4000. if quality and distance else None)'''
new='''            # SDK 거리0은 제조사 ROS와 같은 무반환(+Inf). 자유 공간은 아니다.
            # 양수 거리/품질0은 무효(NaN)로 구분한다. 원 정수는 raw에 보존한다.
            ranges.append('+inf' if distance == 0 else distance/4000. if quality else None)'''
assert text.count(old)==1
text=text.replace(old,new)
anchor="            mounting_calibrated=False, drive_output=False)"
assert text.count(anchor)==1
text=text.replace(anchor,"""            mounting_calibrated=False, drive_output=False,
            range_encoding='sdk_zero_distance_nonreturn_v1',
            zero_distance_points=sum(value == '+inf' for value in ranges),
            invalid_quality_positive_distance_points=sum(value is None for value in ranges),
            nonreturn_implies_free_space=False)""")
candidate=A/'candidate_c1_native_preview.py'
with candidate.open('x',encoding='utf8') as stream:stream.write(text)
with (A/'candidate_sources.json').open('x') as stream:
    json.dump(dict(source=source.relative_to(ROOT).as_posix(),
        before_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
        candidate_sha256=hashlib.sha256(candidate.read_bytes()).hexdigest()),stream,indent=2)
