#!/usr/bin/env python3
"""Inspect the shared start-light STL: header, bbox, connected parts, slices.

Read-only analysis of the downloaded file. Writes stl_analysis.json next to
this script. Units in the STL are assumed mm (bbox 625x465x57 matches the
official issue comment); the assumption is reported, not hidden.
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
STL = HERE / "download" / "STadium_starting_light.stl"


def load(path):
    data = path.read_bytes()
    header = data[:80]
    n = struct.unpack("<I", data[80:84])[0]
    assert len(data) == 84 + 50 * n, (len(data), n)
    rec = np.frombuffer(data[84:], dtype=np.dtype([
        ("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=n)
    return header, rec["n"].astype(float), rec["v"].astype(float), rec["a"]


def components(tris):
    verts = tris.reshape(-1, 3)
    key = np.round(verts, 4)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1, 3)
    parent = list(range(inv.max() + 1))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b, c in inv:
        ra, rb, rc = find(a), find(b), find(c)
        parent[rb] = ra
        parent[find(rc)] = ra
    roots = np.array([find(t[0]) for t in inv])
    out = []
    for r in np.unique(roots):
        sel = tris[roots == r].reshape(-1, 3)
        out.append({
            "triangles": int((roots == r).sum()),
            "min": sel.min(0).round(3).tolist(),
            "max": sel.max(0).round(3).tolist(),
            "size": (sel.max(0) - sel.min(0)).round(3).tolist(),
        })
    out.sort(key=lambda c: (c["min"][0], c["min"][1], c["min"][2]))
    return out


def slice_xy(tris, axis, level):
    """Return segments where plane coord[axis]==level cuts triangles."""
    segs = []
    for t in tris:
        d = t[:, axis] - level
        if d.min() > 0 or d.max() < 0:
            continue
        pts = []
        for i in range(3):
            a, b = t[i], t[(i + 1) % 3]
            da, db = d[i], d[(i + 1) % 3]
            if da == 0:
                pts.append(a)
            if da * db < 0:
                s = da / (da - db)
                pts.append(a + s * (b - a))
        if len(pts) >= 2:
            segs.append(np.array(pts[:2]))
    return segs


def main():
    header, normals, tris, attr = load(STL)
    allv = tris.reshape(-1, 3)
    lo, hi = allv.min(0), allv.max(0)
    comps = components(tris)
    # pick vertical axis: the axis with ~465 extent
    ext = hi - lo
    report = {
        "file": str(STL.relative_to(HERE)),
        "header_ascii": header.decode("latin-1").rstrip("\x00 "),
        "triangles": int(len(tris)),
        "bbox_min": lo.round(3).tolist(),
        "bbox_max": hi.round(3).tolist(),
        "extent": ext.round(3).tolist(),
        "units_assumption": "mm (extent 625x465x57 matches issue #3 comment 6084019922)",
        "components": comps,
    }
    # horizontal slices along the tall axis to show post cross sections
    tall = int(np.argmax(np.abs(ext - 465.0) < 1.0)) if np.any(np.abs(ext - 465.0) < 1.0) else int(np.argsort(ext)[1])
    report["tall_axis"] = tall
    others = [i for i in range(3) if i != tall]
    slices = {}
    for frac in (0.01, 0.05, 0.1, 0.2, 0.4, 0.6, 0.75, 0.85, 0.9, 0.95, 0.99):
        level = lo[tall] + frac * ext[tall]
        segs = slice_xy(tris, tall, level)
        if not segs:
            slices[f"{frac:.2f}"] = {"level": round(float(level), 3), "segments": 0}
            continue
        pts = np.concatenate(segs)[:, others]
        # cluster along the long horizontal axis to split left/right posts
        wide = 0 if ext[others[0]] >= ext[others[1]] else 1
        xs = np.sort(np.unique(np.round(pts[:, wide], 2)))
        gaps = np.where(np.diff(xs) > 20.0)[0]
        groups = np.split(xs, gaps + 1)
        clusters = []
        for g in groups:
            r = np.round(pts[:, wide], 2)
            m = (r >= g.min()) & (r <= g.max())
            p = pts[m]
            clusters.append({
                "min": p.min(0).round(2).tolist(),
                "max": p.max(0).round(2).tolist(),
                "size": (p.max(0) - p.min(0)).round(2).tolist(),
            })
        slices[f"{frac:.2f}"] = {
            "level": round(float(level), 3),
            "segments": len(segs),
            "axes": others,
            "clusters": clusters,
        }
    report["slices_along_tall_axis"] = slices
    out = HERE / "stl_analysis.json"
    out.write_text(json.dumps(report, indent=1, ensure_ascii=False), encoding="utf-8", newline="\n")
    print(json.dumps({k: report[k] for k in ("header_ascii", "triangles", "bbox_min", "bbox_max", "extent", "tall_axis")}, ensure_ascii=False))
    print("components:", len(comps))
    for c in comps:
        print(" ", c)
    for k, v in slices.items():
        print(k, v.get("level"), [ (c["min"], c["size"]) for c in v.get("clusters", [])])


if __name__ == "__main__":
    sys.exit(main())
