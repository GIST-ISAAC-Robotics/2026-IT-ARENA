"""WSL 임시 PPM 프레임을 JPEG(q95)로 옮기고 개수·시각 범위를 기록한 뒤 PPM을 지운다.

사용: python3 convert_frames.py <scratch_clip_dir> <jpeg_out_dir>
"""
import json
import shutil
import sys
from multiprocessing import Pool
from pathlib import Path

import cv2


def convert(job):
    src, dst = job
    image = cv2.imread(src)
    if image is None:
        return src, False
    return src, bool(cv2.imwrite(dst, image, [cv2.IMWRITE_JPEG_QUALITY, 95]))


def main():
    scratch, out = Path(sys.argv[1]), Path(sys.argv[2])
    summary = {}
    for cam in sorted(p for p in scratch.iterdir() if p.is_dir() and p.name.startswith("frames_")):
        dst = out / cam.name
        dst.mkdir(parents=True, exist_ok=True)
        files = sorted(cam.glob("f_*.ppm"))
        jobs = [(str(f), str(dst / (f.stem + ".jpg"))) for f in files]
        with Pool(12) as pool:
            results = pool.map(convert, jobs, chunksize=8)
        bad = [s for s, ok in results if not ok]
        if bad:
            raise SystemExit(f"conversion failed for {len(bad)} frames, e.g. {bad[:3]}")
        stamps = [int(f.stem[2:]) for f in files]
        summary[cam.name] = dict(frames=len(files), first_ns=stamps[0] if stamps else None,
                                 last_ns=stamps[-1] if stamps else None)
        shutil.rmtree(cam)
    (out / "convert_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print("converted " + " ".join(f"{k}={v['frames']}" for k, v in summary.items()))


if __name__ == "__main__":
    main()
