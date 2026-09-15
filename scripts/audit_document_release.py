#!/usr/bin/env python3
"""문서 전용 공식 릴리스를 보존하고 ZIP/저장소 변경 범위를 대조합니다."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path, PurePosixPath
import re
import subprocess
import urllib.parse
import zipfile

ROOT = Path(__file__).resolve().parents[1]
REMOTE = "MOSW626/istech-it-arena"


def api(path, raw=False):
    command = ["gh", "api", f"repos/{REMOTE}/{path}"]
    if raw:
        command += ["-H", "Accept: application/vnd.github.raw+json"]
    result = subprocess.check_output(command, timeout=60)
    return result if raw else json.loads(result)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def zip_files(path):
    with zipfile.ZipFile(path) as archive:
        files = {}
        for item in archive.infolist():
            name = PurePosixPath(item.filename)
            if name.is_absolute() or ".." in name.parts or "\\" in item.filename:
                raise ValueError(f"unsafe ZIP path: {item.filename}")
            if item.is_dir():
                continue
            if item.filename in files:
                raise ValueError(f"duplicate ZIP entry: {item.filename}")
            files[item.filename] = archive.read(item)
        return files


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--previous", required=True)
    parser.add_argument("--release", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    for tag in (args.previous, args.release):
        if not re.fullmatch(r"v\d{4}\.\d{2}\.\d{2}", tag):
            parser.error("release tags must use vYYYY.MM.DD")
    out = (ROOT / args.output).resolve()
    if not out.is_relative_to(ROOT / "artifacts/validation"):
        parser.error("output must be under artifacts/validation")
    out.mkdir(parents=True, exist_ok=False)

    def save(name, value):
        target = out / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    release = api(f"releases/tags/{args.release}")
    commit = api(f"commits/{args.release}")
    previous = api(f"commits/{args.previous}")
    comparison = api(f"compare/{previous['sha']}...{commit['sha']}")
    save("upstream/release.json", release)
    save("upstream/commit.json", commit)
    save("upstream/compare.json", comparison)
    old_tree = api(f"git/trees/{previous['sha']}?recursive=1")
    new_tree = api(f"git/trees/{commit['sha']}?recursive=1")
    assert not old_tree.get("truncated") and not new_tree.get("truncated")
    save("upstream/previous_tree.json", old_tree)
    save("upstream/tree.json", new_tree)
    before = {e["path"]: e["sha"] for e in old_tree["tree"] if e["type"] == "blob"}
    after = {e["path"]: e["sha"] for e in new_tree["tree"] if e["type"] == "blob"}
    changed = sorted(p for p in before.keys() | after.keys() if before.get(p) != after.get(p))
    assert changed == ["MANUAL.md", "track/README.md"], changed
    for path in changed:
        target = out / "upstream" / path
        target.parent.mkdir(parents=True, exist_ok=True)
        data = api(f"contents/{urllib.parse.quote(path)}?ref={commit['sha']}", raw=True)
        blob = b"blob " + str(len(data)).encode() + b"\0" + data
        assert hashlib.sha1(blob).hexdigest() == after[path]
        target.write_bytes(data)

    asset_name = f"it_arena_track_{args.release}.zip"
    asset = next(a for a in release["assets"] if a["name"] == asset_name)
    destination = ROOT / "assets/track/official" / args.release
    destination.mkdir(parents=True, exist_ok=False)
    subprocess.run(["gh", "release", "download", args.release, "--repo", REMOTE,
                    "--pattern", asset_name, "--dir", str(destination)], check=True, timeout=60)
    downloaded = destination / asset_name
    data = downloaded.read_bytes()
    assert len(data) == asset["size"]
    assert asset["digest"] == "sha256:" + sha(data)
    old_path = ROOT / "assets/track/official" / args.previous / f"it_arena_track_{args.previous}.zip"
    old_zip, new_zip = zip_files(old_path), zip_files(downloaded)
    assert old_zip.keys() == new_zip.keys(), "ZIP member list changed"
    zip_changes = sorted(p for p in old_zip if old_zip[p] != new_zip[p])
    assert len(zip_changes) == 1 and zip_changes[0].endswith("README.md"), zip_changes
    assert new_zip[zip_changes[0]] == (out / "upstream/track/README.md").read_bytes()
    save("zip_files.json", [{"path": p, "bytes": len(new_zip[p]),
                             "previous_sha256": sha(old_zip[p]), "sha256": sha(new_zip[p]),
                             "unchanged": old_zip[p] == new_zip[p]} for p in sorted(new_zip)])
    runtime = {}
    for folder in ("src/arena_gazebo/worlds/it_arena_official", "src/arena_gazebo/worlds/it_arena_experimental"):
        for path in sorted((ROOT / folder).rglob("*")):
            if path.is_file():
                runtime[path.relative_to(ROOT).as_posix()] = sha(path.read_bytes())
    save("runtime_sha256.json", runtime)
    for number in (6, 12):
        save(f"upstream/issue_{number}.json", api(f"issues/{number}"))
        save(f"upstream/issue_{number}_comments.json", api(f"issues/{number}/comments?per_page=100"))
    manifest = {"observed_at_utc": datetime.now(timezone.utc).isoformat(),
                "release": args.release, "commit": commit["sha"],
                "previous_release": args.previous, "previous_commit": previous["sha"],
                "asset": asset_name, "bytes": len(data), "sha256": sha(data),
                "previous_sha256": sha(old_path.read_bytes()),
                "repository_changed_files": changed, "zip_changed_files": zip_changes,
                "zip_file_count": len(new_zip), "zip_identical_file_count": len(new_zip) - len(zip_changes),
                "runtime_snapshot_file_count": len(runtime),
                "source_sha256": sha(Path(__file__).read_bytes()), "passed": True}
    save("manifest.json", manifest)
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
