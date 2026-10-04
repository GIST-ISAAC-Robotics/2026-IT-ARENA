#!/usr/bin/env bash
# G0 central_v2 startup timeout 진단: gz 측 실제 토픽 이름/발행/set_pose 반영만 확인한다.
# ROS 구독자·구동 출력 없음. 소유 프로세스 그룹만 정리한다.
repo='/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local'
out="$repo/artifacts/validation/2026-10-05/multi_vehicle_g0_topic_probe_v1"
cd "$repo"
source /opt/ros/jazzy/setup.bash
source install/local_setup.bash
set -u
export GZ_PARTITION="arena_g0_topic_probe_$$"
mkdir -p "$out/gz_logs"
cp "$repo/artifacts/validation/2026-10-05/multi_vehicle_g0_gazebo_central_v2/central_stop/world.sdf" "$out/world.sdf"
lib="$(ros2 pkg prefix arena_gazebo)/lib"
echo "partition=$GZ_PARTITION lib=$lib" > "$out/probe.log"
grep -h GL_RENDERER "$HOME/.gz/rendering/ogre2.log" 2>/dev/null | tail -1 > "$out/renderer_before.txt"
GALLIUM_DRIVER=d3d12 LIBGL_ALWAYS_SOFTWARE=false MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA \
  LD_LIBRARY_PATH="$lib:${LD_LIBRARY_PATH:-}" LD_PRELOAD=libarena-wsl-d3d12-lifetime.so \
  GZ_LOG_PATH="$out/gz_logs" setsid gz sim -r -s -v 3 "$out/world.sdf" > "$out/server.log" 2>&1 &
server=$!
echo "server_pid=$server" >> "$out/probe.log"
found=0
for i in $(seq 1 60); do
  if gz topic -i -t /g0/raw 2>/dev/null | grep -q 'tcp://'; then found=1; break; fi
  sleep 1
done
echo "raw_publisher_found=$found after_s=$i" >> "$out/probe.log"
sleep 3
gz topic -l > "$out/topics.txt" 2>&1
gz topic -i -t /g0/raw > "$out/raw_info.txt" 2>&1
gz topic -i -t /g0/actor_pose > "$out/g0_actor_pose_info.txt" 2>&1
grep -E 'pose' "$out/topics.txt" > "$out/pose_topics.txt"
timeout 10 gz topic -e -t /g0/raw -n 1 > "$out/raw_one_before.txt" 2>&1
echo "raw_echo_rc=$?" >> "$out/probe.log"
n=0
while read -r t; do
  n=$((n+1))
  gz topic -i -t "$t" > "$out/pose_info_$n.txt" 2>&1
  timeout 5 gz topic -e -t "$t" -n 1 > "$out/pose_echo_$n.txt" 2>&1
  echo "pose_topic_$n=$t echo_rc=$?" >> "$out/probe.log"
done < "$out/pose_topics.txt"
# static model set_pose: 서비스 응답과 렌더링 광선 반영을 따로 본다.
gz service -s /world/g0_lab/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'name: "g0_actor", position: {x: 1.5, y: 0.0, z: 0.06}, orientation: {w: 1.0}' > "$out/set_pose_reply.txt" 2>&1
echo "set_pose_rc=$?" >> "$out/probe.log"
sleep 2
timeout 10 gz topic -e -t /g0/raw -n 1 > "$out/raw_one_after.txt" 2>&1
echo "raw_echo_after_rc=$?" >> "$out/probe.log"
n=0
while read -r t; do
  n=$((n+1))
  timeout 5 gz topic -e -t "$t" -n 1 > "$out/pose_echo_after_$n.txt" 2>&1
done < "$out/pose_topics.txt"
gz service -s /server_control --reqtype gz.msgs.ServerControl --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'stop: true' > "$out/server_control_reply.txt" 2>&1
echo "server_control_rc=$?" >> "$out/probe.log"
for i in $(seq 1 20); do
  kill -0 "$server" 2>/dev/null || break
  sleep 1
done
if kill -0 "$server" 2>/dev/null; then
  echo "server_still_running_after_control=1; killing owned group" >> "$out/probe.log"
  kill -TERM -- "-$server" 2>/dev/null; sleep 2; kill -KILL -- "-$server" 2>/dev/null
fi
wait "$server"; echo "server_rc=$?" >> "$out/probe.log"
echo "remaining_group=$(pgrep -g "$server" | tr '\n' ' ')" >> "$out/probe.log"
grep -h GL_RENDERER "$HOME/.gz/rendering/ogre2.log" "$out"/gz_logs/*.log 2>/dev/null | tail -2 > "$out/renderer_after.txt"
python3 - "$out" <<'EOF' >> "$out/probe.log"
import re, sys, pathlib
out = pathlib.Path(sys.argv[1])
for name in ('raw_one_before.txt', 'raw_one_after.txt'):
    text = (out/name).read_text(errors='replace')
    ranges = [float(v) for v in re.findall(r'^ranges:\s*([-+0-9.einfINF]+)', text, re.M)]
    stamp = re.search(r'sec:\s*(\d+)\s*\n\s*nsec:\s*(\d+)', text)
    print(name, 'count', len(ranges), 'forward_idx250', ranges[250] if len(ranges) > 250 else None,
          'stamp', stamp.groups() if stamp else None)
EOF
echo done >> "$out/probe.log"
