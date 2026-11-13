"""현재 공통 코드의 준비/허가 분리. 가짜 입력이며 HAL/실차 검증이 아니다."""
from pathlib import Path
import shutil
import subprocess
import pytest

A=Path(__file__).resolve().parent
ROOT=A.parents[3]


@pytest.mark.parametrize('flags', [
    ['-Os'], ['-O1','-fsanitize=address,undefined','-fno-omit-frame-pointer'],
])
def test_runtime_preparation_does_not_extend_authority(tmp_path,flags):
    compiler=shutil.which('g++')
    assert compiler,'g++ is required; missing compiler is not a test pass'
    includes=[ROOT/'src/arena_vehicle_interface'/p for p in
              ('portable','firmware_smoke/main','firmware_runtime_preview/main')]
    binary=tmp_path/'preparation'
    result=subprocess.run([compiler,'-std=c++17','-Wall','-Wextra','-Werror','-pedantic',
        '-ffp-contract=off',*flags,*(f'-I{p}' for p in includes),str(A/'cases.cpp'),
        '-o',str(binary)],capture_output=True,text=True,timeout=90)
    assert result.returncode==0,(result.stdout,result.stderr)
    result=subprocess.run([str(binary)],capture_output=True,text=True,timeout=15)
    assert result.returncode==0 and not result.stderr,(result.stdout,result.stderr)
    assert result.stdout.strip().endswith(':PASS')
