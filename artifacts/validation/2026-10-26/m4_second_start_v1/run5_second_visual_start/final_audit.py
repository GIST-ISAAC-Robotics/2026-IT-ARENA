import hashlib,json,os
from pathlib import Path
out=Path(__file__).resolve().parent; repo=out.parents[4]; pre=json.loads((out/"pre_run_hashes.json").read_text()); names=json.loads((out/"frozen_files.json").read_text())
post={n:hashlib.sha256((repo/n).read_bytes()).hexdigest() for n in names}
(out/"final_source_hashes.json").write_text(json.dumps(post,sort_keys=True,indent=2)+"\n")
diff={n:{"before":pre.get(n),"after":post.get(n)} for n in sorted(set(pre)|set(post)) if pre.get(n)!=post.get(n)}
(out/"final_source_hash_diff.json").write_text(json.dumps(diff,sort_keys=True,indent=2)+"\n")
tokens=("gz sim","gzserver","gzclient","ros2 launch","ros_gz_bridge","validate_g1_traffic.py","arena_bringup","arena_vehicle_interface","local_pursuit")
found=[]
for p in Path("/proc").iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid(): continue
 try:
  cmd=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace"); comm=(p/"comm").read_text().strip()
 except OSError: continue
 if any(t in cmd for t in tokens) or comm in ("gzserver","gzclient"):
  found.append({"pid":int(p.name),"comm":comm,"cmdline":cmd})
(out/"final_processes.json").write_text(json.dumps(found,indent=2)+"\n",encoding="utf-8")
print(json.dumps({"hash_diff":diff,"residual_processes":found},sort_keys=True))
