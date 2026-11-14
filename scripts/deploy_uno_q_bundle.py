#!/usr/bin/env python3
"""UNO Q 이관 묶음 검증/별도 폴더 설치. 장치·서비스·flash에는 접근하지 않는다."""
import argparse
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import shutil
import stat
import tempfile
import zipfile

SCHEMA = "uno_q_deployment_bundle_v1"
MAX_BYTES = 512 * 1024 * 1024


def digest(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else hashlib.sha256(stream.read()).hexdigest()


def safe_name(name):
    path = PurePosixPath(name)
    if (not name or "\\" in name or ":" in name or path.is_absolute()
            or any(p in ("", ".", "..") for p in name.split("/"))):
        raise ValueError("unsafe bundle path: " + repr(name))
    return path


def verify_directory(root):
    root = Path(root).resolve(strict=True)
    manifest_path = root / "bundle_manifest.json"
    if manifest_path.is_symlink():
        raise ValueError("manifest symlink is not allowed")
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    if manifest.get("schema") != SCHEMA or not isinstance(manifest.get("files"), dict):
        raise ValueError("unsupported bundle manifest")
    files = manifest["files"]
    if not files or "bundle_manifest.json" in files:
        raise ValueError("invalid manifest inventory")
    total = 0
    for name, record in files.items():
        rel = safe_name(name)
        target = root.joinpath(*rel.parts)
        for candidate in (target, *target.parents):
            if candidate == root:
                break
            if candidate.is_symlink():
                raise ValueError("symlink in bundle: " + name)
        if not target.is_file() or not target.resolve().is_relative_to(root):
            raise ValueError("missing or escaped bundle file: " + name)
        if not isinstance(record, dict) or record.get("size") != target.stat().st_size:
            raise ValueError("size mismatch: " + name)
        total += target.stat().st_size
        if total > MAX_BYTES or digest(target) != record.get("sha256"):
            raise ValueError("hash mismatch or oversized bundle: " + name)
    actual = set()
    for item in root.rglob("*"):
        if item.is_symlink():
            raise ValueError("symlink in bundle: " + str(item))
        if item.is_file() and item != manifest_path:
            actual.add(item.relative_to(root).as_posix())
    if actual != set(files):
        raise ValueError("unexpected files in bundle")
    return manifest


def extract_archive(archive, destination):
    """fresh temporary directory only; reject links, traversal and duplicates."""
    seen, total = set(), 0
    with zipfile.ZipFile(archive) as bundle:
        members = bundle.infolist()
        for member in members:
            name = member.filename.rstrip("/") if member.is_dir() else member.filename
            safe_name(name)
            mode = member.external_attr >> 16
            if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
                raise ValueError("non-regular archive entry")
            if name in seen:
                raise ValueError("duplicate archive entry")
            seen.add(name)
            total += member.file_size
            if total > MAX_BYTES:
                raise ValueError("oversized archive")
        for member in members:
            target = Path(destination).joinpath(*PurePosixPath(member.filename).parts)
            if member.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            with bundle.open(member) as source, target.open("xb") as output:
                shutil.copyfileobj(source, output)


def install_directory(source, destination):
    manifest = verify_directory(source)
    destination = Path(destination).expanduser().absolute()
    if destination.exists() or destination.is_symlink():
        raise ValueError("destination must not already exist")
    destination.parent.mkdir(parents=True, exist_ok=True)
    # Build beside destination, verify the complete copy, then publish atomically.
    with tempfile.TemporaryDirectory(prefix=".uno-q-stage-", dir=destination.parent) as stage:
        copied = Path(stage) / "release"
        shutil.copytree(source, copied)
        verify_directory(copied)
        if destination.exists() or destination.is_symlink():
            raise ValueError("destination appeared during installation")
        os.rename(copied, destination)
    return manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bundle", type=Path, help="ZIP 또는 압축을 푼 폴더")
    parser.add_argument("--install", action="store_true", help="검증 뒤 새 폴더로 복사")
    parser.add_argument("--destination", type=Path)
    args = parser.parse_args(argv)
    if args.install != (args.destination is not None):
        parser.error("--install and --destination must be supplied together")
    with tempfile.TemporaryDirectory(prefix="uno-q-verify-") as temporary:
        if args.bundle.is_dir():
            root = args.bundle
        else:
            root = Path(temporary)
            extract_archive(args.bundle, root)
        manifest = (install_directory(root, args.destination) if args.install else verify_directory(root))
    print(json.dumps({"verified": True, "installed": args.install,
                      "destination": str(args.destination) if args.install else None,
                      "file_count": len(manifest["files"]),
                      "hardware_access": False, "flash_executed": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, zipfile.BadZipFile) as error:
        raise SystemExit("UNO Q bundle rejected: " + str(error))
