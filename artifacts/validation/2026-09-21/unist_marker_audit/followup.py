"""AprilTag ID 8 반환의 오류 보정 의존성과 정상 대조군을 추가 확인한다."""
import ctypes, json
from pathlib import Path
import cv2
import numpy as np
from pupil_apriltags import Detector
from pupil_apriltags.bindings import _ApriltagFamily, _ApriltagDetector, _ImageU8
ROOT=Path(__file__).resolve().parent
gray=cv2.imdecode(np.frombuffer((ROOT/'unist_original.png').read_bytes(),np.uint8),0)
out={'controls':[], 'correction_sweep':[]}
for fam, const in [('tag16h5',cv2.aruco.DICT_APRILTAG_16h5),('tag25h9',cv2.aruco.DICT_APRILTAG_25h9),('tag36h11',cv2.aruco.DICT_APRILTAG_36h11)]:
    d=Detector(families=fam,quad_decimate=1.0)
    image=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(const),0,360)
    image=cv2.copyMakeBorder(image,60,60,60,60,cv2.BORDER_CONSTANT,value=255)
    tags=d.detect(image)
    out['controls'].append({'family':fam,'source':'OpenCV canonical ID 0','ids':[t.tag_id for t in tags],'hamming':[t.hamming for t in tags]})
    cv2.imencode('.png',image)[1].tofile(str(ROOT/(fam+'_control.png')))

for bits in (0,1,2):
    d=Detector(families='tag16h5',quad_decimate=1.0)
    lib=d.libc
    lib.apriltag_detector_remove_family.argtypes=[ctypes.POINTER(_ApriltagDetector),ctypes.POINTER(_ApriltagFamily)]
    lib.apriltag_detector_remove_family.restype=None
    lib.apriltag_detector_remove_family(d.tag_detector_ptr,d.tag_families['tag16h5'])
    lib.apriltag_detector_add_family_bits.argtypes=[ctypes.POINTER(_ApriltagDetector),ctypes.POINTER(_ApriltagFamily),ctypes.c_int]
    lib.apriltag_detector_add_family_bits(d.tag_detector_ptr,d.tag_families['tag16h5'],bits)
    for scale in (1.0,0.75,0.5,0.375,0.25):
        im=cv2.resize(gray,None,fx=scale,fy=scale,interpolation=cv2.INTER_AREA)
        for mode,img in [('gray',im),('binary',cv2.threshold(im,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)[1])]:
            tags=d.detect(img)
            out['correction_sweep'].append({'bits_allowed':bits,'scale':scale,'mode':mode,'tags':[{'id':t.tag_id,'hamming':t.hamming} for t in tags]})
    # genuine ID 8 must be detected even with zero correction
    genuine=cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5),8,360)
    genuine=cv2.copyMakeBorder(genuine,60,60,60,60,cv2.BORDER_CONSTANT,value=255)
    tags=d.detect(genuine)
    out.setdefault('id8_controls',[]).append({'bits_allowed':bits,'ids':[t.tag_id for t in tags],'hamming':[t.hamming for t in tags]})
    if bits==0: cv2.imencode('.png',genuine)[1].tofile(str(ROOT/'genuine_tag16h5_id8.png'))
(ROOT/'followup.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
print(json.dumps(out,indent=2))
