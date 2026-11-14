"""7개 기록 영상의 색인(recordings_index.json)과 렌더 기록 사본·도구 해시 목록을 만든다.

- build/competition_recordings_20261115/logs/<clip>/ 의 작은 기록만 검증 폴더로 복사한다(poses.txt·render_world.sdf는 해시만).
사용: python make_index.py
"""
import hashlib
import json
import shutil
from pathlib import Path

TOOLS = Path(__file__).resolve().parent
VIDEO_DIR = TOOLS.parent
REPO = TOOLS.parents[3]
BASE = REPO / "build" / "competition_recordings_20261115"
VAL_OUT = REPO / "artifacts" / "validation" / "2026-11-15" / "video_recording_v1"
COPY = ("meta.json", "processes.txt", "render_result.txt", "build_scene.log", "done.txt")


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    spec = json.load(open(TOOLS / "clips.json", encoding="utf-8"))
    items = []
    for clip in spec["clips"]:
        edit = json.load(open(VAL_OUT / "edit" / f"{clip['file']}.edit.json", encoding="utf-8"))
        qa = json.load(open(VAL_OUT / "qa" / f"{clip['file']}.qa.json", encoding="utf-8"))
        logs = BASE / "logs" / clip["key"]
        dst = VAL_OUT / "render_logs" / clip["key"]
        dst.mkdir(parents=True, exist_ok=True)
        for name in COPY:
            if (logs / name).exists():
                shutil.copy2(logs / name, dst / name)
        conv = BASE / "frames" / clip["key"] / "convert_summary.json"
        if conv.exists():
            shutil.copy2(conv, dst / "convert_summary.json")
        hashes = {name: sha256(logs / name) for name in ("poses.txt", "render_world.sdf") if (logs / name).exists()}
        (dst / "large_inputs_sha256.json").write_text(json.dumps(hashes, indent=2) + chr(10), encoding="utf-8", newline=chr(10))
        items.append(dict(
            key=clip["key"], video=edit["output"], poster=edit["poster"], title=clip["title"], subtitle=clip["subtitle"],
            category=clip["category"], label="Gazebo 시뮬레이션 · 저장 자세 재렌더(새 주행 아님)",
            source_run=edit["source_run"], report=clip["report"], facts=clip["facts"], conditions=clip["conditions"],
            sha256=edit["sha256"], bytes=edit["bytes"], frames=edit["video"]["frames"],
            duration_s=edit["video"]["duration_s"], size=[edit["video"]["width"], edit["video"]["height"]],
            fps=edit["video"]["fps"], sim_window_s=[edit["sim_window"]["first_frame_target_s"], edit["sim_window"]["last_frame_target_s"]],
            playback_rate=1.0, poster_sha256=edit["poster_sha256"],
            contact_sheet=qa["contact_sheet"], qa_frames_match=qa["frames_match"], qa_dark_frames=qa["dark_frames"],
            render_result=edit["render"]["result"].split(" done=")[0]))
    tools = {str(p.relative_to(TOOLS)).replace(chr(92), "/"): sha256(p) for p in sorted(TOOLS.rglob("*"))
             if p.is_file() and "__pycache__" not in p.parts}
    index = dict(schema="competition_recordings_20261115_index_v1", generated_by="tools/make_index.py",
                 note="모든 영상은 기존 Gazebo 실행의 저장 자세 재렌더다. 실차 영상·새 주행·최종 ESP32 묶음 실행이 아니다.",
                 final_esp32_relation="성공 영상의 원 실행 소스는 모두 최종 ESP32 묶음(r533 selected_v2)과 다르다. 비교: "
                                      "artifacts/validation/2026-11-15/video_recording_v1/provenance/source_comparison.json",
                 videos=items, total_bytes=sum(i["bytes"] for i in items),
                 plugin_sha256=sha256(BASE / "plugin" / "librecording-pose-replay.so"), tools_sha256=tools)
    (VIDEO_DIR / "recordings_index.json").write_text(json.dumps(index, ensure_ascii=False, indent=2) + chr(10),
                                                    encoding="utf-8", newline=chr(10))
    print(json.dumps(dict(videos=len(items), total_bytes=index["total_bytes"]), ensure_ascii=False))


if __name__ == "__main__":
    main()
