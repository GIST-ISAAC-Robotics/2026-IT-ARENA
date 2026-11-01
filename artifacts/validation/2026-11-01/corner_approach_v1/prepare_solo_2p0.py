import hashlib,json,pathlib,subprocess
base=pathlib.Path('artifacts/validation/2026-11-01/corner_approach_v1')
ref=pathlib.Path('artifacts/validation/2026-11-01/race_continuity_v1/solo_2p0_corrected_pre_manifest.json')
refm=json.loads(ref.read_text(encoding='utf-8'))
manifest=base/'solo_2p0_pre_manifest.json'; out=base/'solo_2p0'
if manifest.exists() or out.exists(): raise SystemExit(f'refusing to overwrite manifest/run: {manifest.exists()}, {out.exists()}')
args=list(refm['command_arguments']); args[args.index('g1_open_2p0')]='g1_open_2p0_approach'; args[-1]=str(pathlib.Path('C:/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local')/out)
hashes={}; changed=[]
for rel,old in refm['source_sha256'].items():
 p=pathlib.Path(rel)
 if not p.is_file(): raise SystemExit(f'missing input source: {rel}')
 digest=hashlib.sha256(p.read_bytes()).hexdigest(); hashes[rel]=digest
 if digest!=old: changed.append({'path':rel,'corrected_2p0_sha256':old,'current_sha256':digest})
head=subprocess.run(['git','rev-parse','HEAD'],capture_output=True,text=True,check=True).stdout.strip()
result={'created_before_run':True,'git_head':head,'scope':'same run command, one-lap conditions, world, and race-path option set as solo_2p0_corrected; profile g1_open_2p0_approach and output corner_approach_v1/solo_2p0','reference_manifest':str(ref),'command_arguments':args,'cwd_wsl':str(pathlib.Path.cwd()),'environment':{'ros_setup':'/opt/ros/jazzy/setup.bash','workspace_setup':'install/local_setup.bash','PYTHONPATH_prefix':'/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests'},'drive_gear_ratio':refm['drive_gear_ratio'],'source_sha256':hashes,'reference_source_hash_differences':changed,'race_path_flags_expected_true':refm['race_path_flags_expected_true'],'output_absent_before_run':True}
manifest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'manifest':str(manifest),'source_count':len(hashes),'source_hash_differences':changed,'git_head':head,'profile':args[args.index('--speed-profile')+1],'output':args[-1]},ensure_ascii=False))