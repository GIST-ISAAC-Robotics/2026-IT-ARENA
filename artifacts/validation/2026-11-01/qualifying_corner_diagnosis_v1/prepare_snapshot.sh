#!/bin/bash
# 현재 src 사본 위에 실행 당시 스냅숏을 덮어 재생용 트리를 만든다(저장소 src는 읽기만).
# 스냅숏에 없는 trial 모듈은 현재 사본을 쓰므로 baseline 재생 일치로 동등성을 확인한다.
set -eu
R="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local"
S="$HOME/arena_scratch/qc_diag/snap_combined"
rm -rf "$S"; mkdir -p "$S/src"
cp -r "$R/src/arena_autonomy" "$R/src/arena_vehicle_interface" "$S/src/"
cd "$S"
python3 -I -c "
import zipfile,sys
z=zipfile.ZipFile(sys.argv[1]);names=[n for n in z.namelist() if n.startswith('src/')]
z.extractall('.',names);print('overlaid',len(names))
" "$R/artifacts/validation/2026-11-01/qualifying_profile_v1/run1/source_snapshot.zip"
