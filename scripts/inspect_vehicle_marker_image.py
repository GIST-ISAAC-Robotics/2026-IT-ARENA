"""저장 RGB 이미지에서 검은 코드 폭·근사 pose를 읽는다. 카메라/ROS/장치에 접근하지 않는다."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from arena_autonomy.core import marker_observations
from arena_autonomy.vehicle_marker_geometry import estimate_regions,validate_profile


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--profile',type=Path,required=True)
    p.add_argument('--image',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();profile=validate_profile(json.loads(a.profile.read_text(encoding='utf-8')))
    raw=cv2.imdecode(np.frombuffer(a.image.read_bytes(),np.uint8),cv2.IMREAD_COLOR)
    if raw is None:raise ValueError('image decode failed')
    rgb=cv2.cvtColor(raw,cv2.COLOR_BGR2RGB);observations=marker_observations(rgb,.02)
    report=estimate_regions(rgb,observations,profile,source_stamp_s=0.,
        frame_id=profile['camera']['optical_frame_id'],epoch=0)
    report.update(time_basis='offline_image_unverified_capture',runtime_usable=False,
        image_path=str(a.image),image_width_px=rgb.shape[1],image_height_px=rgb.shape[0])
    report['black_code_pixels']=[dict(dictionary='DICT_4X4_50',id=i,corners_px=np.asarray(q).tolist(),
        mean_horizontal_edge_px=float((np.linalg.norm(q[1]-q[0])+np.linalg.norm(q[2]-q[3]))/2))
        for i,q in observations]
    with a.output.open('x',encoding='utf-8') as stream:
        json.dump(report,stream,ensure_ascii=False,indent=2);stream.write('\n')


if __name__=='__main__':main()
