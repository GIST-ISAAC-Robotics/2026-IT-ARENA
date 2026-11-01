import hashlib, json, pathlib, subprocess
base=pathlib.Path('artifacts/validation/2026-11-01/race_continuity_v1')
source=json.loads((base/'solo_2p0_corrected_pre_manifest.json').read_text(encoding='utf-8'))
out=base/'solo_1p5_race_path_pre_manifest.json'
if out.exists(): raise SystemExit(f'refusing to overwrite {out}')
run=base/'solo_1p5_race_path'
if run.exists(): raise SystemExit(f'refusing to reuse {run}')
args=list(source['command_arguments'])
args[args.index('g1_open_2p0')]='g1_open_1p5'
args[-1]=str(pathlib.Path('C:/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local')/run)
hashes={}
missing=[]
for rel in source['source_sha256']:
 p=pathlib.Path(rel)
 if not p.exists(): missing.append(rel)
 else: hashes[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
if missing: raise SystemExit(f'missing source inputs: {missing[:5]} ({len(missing)})')
head=subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True,check=True).stdout.strip()
result={'created_before_run':True,'git_head':head,'scope':'same command/input/source/world/race-path flags as solo_2p0_corrected; only speed profile changed to g1_open_1p5 and output directory changed to solo_1p5_race_path','command_arguments':args,'cwd_wsl':str(pathlib.Path.cwd()),'environment':{'ros_setup':'/opt/ros/jazzy/setup.bash','workspace_setup':'install/local_setup.bash','PYTHONPATH_prefix':'/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests'},'drive_gear_ratio':source['drive_gear_ratio'],'source_sha256':hashes,'source_hashes_match_solo_2p0_corrected_manifest':hashes==source['source_sha256'],'race_path_flags_expected_true':source['race_path_flags_expected_true'],'compared_with':'artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected_pre_manifest.json','output_absent_before_run':True}
out.write_text(json.dumps(result,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
print(json.dumps({'manifest':str(out),'hash_count':len(hashes),'source_hashes_match':result['source_hashes_match_solo_2p0_corrected_manifest'],'git_head':head,'args':args},ensure_ascii=False))