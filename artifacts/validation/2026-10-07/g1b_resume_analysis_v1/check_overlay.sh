#!/usr/bin/env bash
# 이미 사용한 source_before(작업 트리 복사+ZIP)와 HEAD+ZIP 재생성본이 같은지 확인(분석 전용).
set -e
cd '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
A=artifacts/validation/2026-10-07/g1b_resume_analysis_v1
T=$(mktemp -d)
git archive HEAD src/arena_autonomy src/arena_vehicle_interface | tar -x -C "$T"
python3 -c "import zipfile;zipfile.ZipFile('artifacts/validation/2026-10-07/g1b_resume_start_v1/source_before.zip').extractall('$T')"
(cd "$A/source_before" && find src -type f -name '*.py' | sort | xargs sha256sum) > /tmp/a.txt
(cd "$T" && find src -type f -name '*.py' | sort | xargs sha256sum) > /tmp/b.txt
diff /tmp/a.txt /tmp/b.txt && echo "overlay identical: $(wc -l < /tmp/a.txt) python files"
rm -rf "$T"
