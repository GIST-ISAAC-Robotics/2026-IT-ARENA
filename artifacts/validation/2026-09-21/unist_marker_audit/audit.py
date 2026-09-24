"""UNIST 등록 원본의 표준 마커 검출 재현. 실행 환경은 report.json에 기록."""
import ctypes
import hashlib
import importlib.metadata
import json
from pathlib import Path
import cv2
import numpy as np
import zxingcpp
from pupil_apriltags import Detector
from pupil_apriltags.bindings import _ApriltagFamily, _ImageU8

ROOT = Path(__file__).resolve().parent
raw = (ROOT / 'unist_original.png').read_bytes()
gray = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_GRAYSCALE)
assert gray is not None
variants = {}
for scale in (1.0, 0.5, 0.25):
    resized = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    for mode, img in [('gray', resized), ('binary', cv2.threshold(resized, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1])]:
        for rotation in range(4):
            variants[f'{scale}_{mode}_rot{rotation*90}'] = np.ascontiguousarray(np.rot90(img, rotation))
for rotation in range(4):
    variants[f'inverted_rot{rotation*90}'] = np.ascontiguousarray(np.rot90(255-gray, rotation))

report = {'source': 'https://github.com/MOSW626/istech-it-arena/issues/1#issuecomment-5755605009',
          'sha256': hashlib.sha256(raw).hexdigest(), 'shape': list(gray.shape),
          'versions': {p: importlib.metadata.version(p) for p in ['opencv-contrib-python-headless','numpy','pupil-apriltags','zxing-cpp']},
          'variants': list(variants), 'opencv': [], 'apriltag_native': [], 'barcode': []}

def save():
    (ROOT/'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')

# 동일 상수를 가리키는 대소문자 별칭은 한 번만 검사한다.
dicts = {}
for name in sorted(dir(cv2.aruco)):
    if name.startswith('DICT_'):
        dicts.setdefault(getattr(cv2.aruco, name), name)
for value, name in sorted(dicts.items()):
    dictionary = cv2.aruco.getPredefinedDictionary(value)
    for profile in ('default', 'border2', 'apriltag_quad', 'geometry_relaxed'):
        p = cv2.aruco.DetectorParameters()
        if profile == 'border2': p.markerBorderBits = 2
        if profile == 'apriltag_quad': p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_APRILTAG
        if profile == 'geometry_relaxed':
            p.minMarkerDistanceRate = 0.02
            p.adaptiveThreshWinSizeMax = 101
        detector = cv2.aruco.ArucoDetector(dictionary, p)
        control = cv2.aruco.generateImageMarker(dictionary, 0, 360, borderBits=p.markerBorderBits)
        control = cv2.copyMakeBorder(control, 60, 60, 60, 60, cv2.BORDER_CONSTANT, value=255)
        _, ids, _ = detector.detectMarkers(control)
        record = {'dictionary': name, 'profile': profile, 'control_id0_ok': ids is not None and 0 in ids,
                  'control_ids': [] if ids is None else ids.flatten().tolist(), 'detections': [], 'original_rejected': 0}
        for key, img in variants.items():
            corners, ids, rejected = detector.detectMarkers(img)
            if key == '1.0_gray_rot0':
                record['original_rejected'] = len(rejected)
                record['original_largest_rejected_area'] = max((float(cv2.contourArea(c)) for c in rejected), default=0)
            if ids is not None:
                record['detections'].append({'variant': key, 'ids': ids.flatten().tolist(), 'corners': [c.tolist() for c in corners]})
        report['opencv'].append(record)
    print(name, 'done', flush=True)
    save()

for family in ('tag16h5','tag25h9','tag36h11','tagCircle21h7','tagCircle49h12','tagCustom48h12','tagStandard41h12','tagStandard52h13'):
    detector = Detector(families=family, nthreads=2, quad_decimate=1.0)
    record = {'family': family, 'detections': []}
    # 원본 AprilTag C 라이브러리의 생성기로 각 계열 ID 0 대조군을 만든다.
    lib = detector.libc
    lib.apriltag_to_image.argtypes = [ctypes.POINTER(_ApriltagFamily), ctypes.c_int]
    lib.apriltag_to_image.restype = ctypes.POINTER(_ImageU8)
    ptr = lib.apriltag_to_image(detector.tag_families[family], 0)
    im = ptr.contents
    control = np.ctypeslib.as_array(im.buf, shape=(im.height*im.stride,)).reshape(im.height, im.stride)[:, :im.width].copy()
    lib.image_u8_destroy.argtypes = [ctypes.POINTER(_ImageU8)]
    lib.image_u8_destroy(ptr)
    control = cv2.resize(control, None, fx=24, fy=24, interpolation=cv2.INTER_NEAREST)
    control = cv2.copyMakeBorder(control, 60,60,60,60,cv2.BORDER_CONSTANT,value=255)
    controls = detector.detect(control)
    record['control_id0_ok'] = any(t.tag_id == 0 for t in controls)
    record['control_ids'] = [int(t.tag_id) for t in controls]
    for key,img in variants.items():
        tags = detector.detect(img)
        if tags:
            record['detections'].append({'variant':key, 'tags':[{'id':int(t.tag_id),'hamming':int(t.hamming),'margin':float(t.decision_margin), 'corners': t.corners.tolist()} for t in tags]})
    report['apriltag_native'].append(record)
    print(family, 'done', flush=True)
    save()

for key,img in variants.items():
    codes = zxingcpp.read_barcodes(img)
    if codes:
        report['barcode'].append({'variant': key,'results':[{'format':str(c.format),'text':c.text,'valid':c.valid} for c in codes]})
report['barcode_controls'] = []
for fmt in (zxingcpp.BarcodeFormat.QRCode, zxingcpp.BarcodeFormat.DataMatrix, zxingcpp.BarcodeFormat.Aztec):
    barcode = zxingcpp.create_barcode('marker-audit-control', fmt)
    control = np.asarray(zxingcpp.write_barcode_to_image(barcode, scale=8))
    results = zxingcpp.read_barcodes(control)
    report['barcode_controls'].append({'format':str(fmt),'ok':any(r.text == 'marker-audit-control' for r in results)})
report['summary'] = {
    'opencv_dictionaries':len(dicts), 'opencv_calls':len(report['opencv'])*len(variants),
    'opencv_positive_controls':sum(r['control_id0_ok'] for r in report['opencv']),
    'opencv_detection_conditions':sum(len(r['detections']) for r in report['opencv']),
    'native_families':len(report['apriltag_native']),
    'native_positive_controls':sum(r['control_id0_ok'] for r in report['apriltag_native']),
    'native_detection_conditions':sum(len(r['detections']) for r in report['apriltag_native']),
    'barcode_detection_conditions':len(report['barcode'])}
save()
print(json.dumps(report['summary']), flush=True)
