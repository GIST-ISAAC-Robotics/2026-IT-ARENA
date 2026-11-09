#!/usr/bin/env bash
# 여섯 교통 장면을 순서대로 1회씩 재렌더하고 PNG로 옮긴다. 결과 요약: ~/arena_scratch/feature_showcase/render_all.summary
set -o pipefail
TOOLS="/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/videos/2026-11-09/feature_showcase/tools"
W="$HOME/arena_scratch/feature_showcase"
PNG_ROOT="/mnt/c/Users/Jinhyeong/arena_media_work/clips"
SUMMARY="$W/render_all.summary"
: > "$SUMMARY"
if pgrep -af '^gz sim' > /dev/null; then echo "preexisting gz sim process; abort" | tee -a "$SUMMARY"; exit 3; fi
CLIPS=(
  "slow_030|artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1|7.6|19.6"
  "medium_050_recessed|artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1|8.0|18.0"
  "yield_back|artifacts/validation/2026-11-07/return_guide_front_v1/straight_moving_lead/run2|8.5|18.0"
  "static_bypass|artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1|8.2|20.5"
  "stop_restart|artifacts/validation/2026-11-08/rear_face_restart_v1/moving_stop_restart/run1|9.8|24.5"
  "overtaken|artifacts/validation/2026-11-08/rear_overtake_measured_v1/left_rear_overtakes/run1|8.2|15.0"
)
STATUS=0
for spec in "${CLIPS[@]}"; do
  IFS='|' read -r NAME RUN T0 T1 <<< "$spec"
  if [ -n "${ONLY:-}" ] && [ "$ONLY" != "$NAME" ]; then continue; fi
  bash "$TOOLS/render_scene.sh" "$NAME" "$RUN" "$T0" "$T1" >> "$SUMMARY" 2>&1 || STATUS=1
  rm -rf "$PNG_ROOT/$NAME"; mkdir -p "$PNG_ROOT/$NAME"
  python3 -I - "$W/clips/$NAME/frames" "$PNG_ROOT/$NAME" >> "$SUMMARY" 2>&1 <<'PY' || STATUS=1
import sys, pathlib, cv2
src, dst = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
n = 0
for f in sorted(src.glob("f_*.ppm")):
    im = cv2.imread(str(f))
    if im is None: raise SystemExit(f"unreadable {f}")
    cv2.imwrite(str(dst / (f.stem + ".png")), im, [cv2.IMWRITE_PNG_COMPRESSION, 1]); n += 1
print(f"png {dst.name} {n}")
PY
  cp -f "$W/clips/$NAME/meta.json" "$PNG_ROOT/$NAME/meta.json"
  cp -f "$W/clips/$NAME/render_result.txt" "$PNG_ROOT/$NAME/render_result.txt"
done
LEFT=$(pgrep -af '^gz sim' | wc -l)
echo "residual_gz_processes=$LEFT status=$STATUS" >> "$SUMMARY"
cp -f "$SUMMARY" "$PNG_ROOT/render_all.summary"
echo "$STATUS" > "$PNG_ROOT/render_all.exit"
exit $STATUS
