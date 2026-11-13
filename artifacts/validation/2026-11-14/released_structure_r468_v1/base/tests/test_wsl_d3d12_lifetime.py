"""실제 로더로 core 수명 유지와 다른 라이브러리의 정상 해제를 확인합니다."""
import os
from pathlib import Path
import shutil
import subprocess
import sys

import pytest


@pytest.mark.skipif(sys.platform != "linux" or not shutil.which("g++"), reason="Linux ELF loader required")
def test_only_d3d12_core_remains_mapped_after_dlclose(tmp_path):
    directory = tmp_path / "library path with spaces"
    directory.mkdir()
    repo = Path(__file__).resolve().parents[1]
    source = repo / "src/arena_gazebo/src/wsl_d3d12_lifetime.cpp"
    subprocess.run(["g++", "-shared", "-fPIC", str(source), "-ldl", "-o", str(directory / "libguard.so")], check=True)
    dummy = directory / "dummy.cpp"
    dummy.write_text('extern "C" int marker() { return 42; }\n')
    for name in ("libd3d12core.so", "libordinary.so"):
        subprocess.run(["g++", "-shared", "-fPIC", str(dummy), "-o", str(directory / name)], check=True)
    env = {**os.environ, "LD_PRELOAD": "libguard.so", "LD_LIBRARY_PATH": str(directory)}
    probe = r'''
import ctypes, os, pathlib, sys
base = pathlib.Path(sys.argv[1])
loader = ctypes.CDLL(None)
loader.dlclose.argtypes = [ctypes.c_void_p]
loader.dlclose.restype = ctypes.c_int
loader.dlopen.argtypes = [ctypes.c_char_p, ctypes.c_int]
loader.dlopen.restype = ctypes.c_void_p
for name, retained in (("libd3d12core.so", True), ("libordinary.so", False)):
    path = base / name
    module = ctypes.CDLL(str(path))
    assert module.marker() == 42
    assert loader.dlclose(module._handle) == 0
    handle = loader.dlopen(os.fsencode(path), os.RTLD_NOW | os.RTLD_NOLOAD)
    assert bool(handle) == retained, (name, handle)
    if handle:
        assert loader.dlclose(handle) == 0
assert loader.dlopen(None, os.RTLD_NOW)
'''
    subprocess.run([sys.executable, "-c", probe, str(directory)], env=env, check=True, timeout=20)
