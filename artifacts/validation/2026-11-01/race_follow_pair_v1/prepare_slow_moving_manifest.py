import hashlib,json,pathlib,subprocess,sys
root=pathlib.Path.cwd(); base=root/'artifacts/validation/2026-11-01/race_follow_pair_v1'; run=base/'slow_moving'
manifest=base/'slow_moving_pre_manifest.json'
if manifest.exists() or run.exists(): raise SystemExit(f'refusing to overwrite existing manifest/run: {manifest.exists()}, {run.exists()}')
config=base/'slow_lead.json'; cfg=json.loads(config.read_text(encoding='utf-8')); parent=root/cfg['parent']['path']; parent_hash=hashlib.sha256(parent.read_bytes()).hexdigest()
if parent_hash!=cfg['parent']['sha256']: raise SystemExit(f'config parent hash mismatch: {parent_hash}')
sys.path.insert(0,str(root/'scripts'));sys.path.insert(0,str(root/'src/arena_autonomy'));sys.path.insert(0,str(root/'src/arena_vehicle_interface'))
import validate_g1_traffic as validator
files=set(validator.SOURCE_FILES)|{str(config.relative_to(root)),str((base/'slow_summary.py').relative_to(root))}
probe=pathlib.Path('artifacts/validation/2026-10-07/g1b_contact_probe_v1/report.json')
if probe.exists(): files.add(str(probe))
hashes={}
for rel in sorted(files):
 p=root/rel
 if not p.is_file(): raise SystemExit(f'missing input: {rel}')
 hashes[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
args=['python3','-B','scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','moving','--config',str(config.relative_to(root)),'--case','slow_lead','--seed','0','--record-follow-decisions','--output',str(run.relative_to(root))]
ratio_line=next(line.strip() for line in (root/'src/arena_description/config/vehicle.yaml').read_text().splitlines() if 'gear_ratio:' in line and 'gear_ratio_status' not in line)
ratio=float(ratio_line.split(':',1)[1].strip())
build=base/'colcon_build_arena_bringup.log'; build_text=build.read_text(encoding='utf-8'); build_exit=next(int(x.split('=',1)[1]) for x in reversed(build_text.splitlines()) if x.startswith('ACTUAL_EXIT_CODE='))
result={'created_before_run':True,'git_head':subprocess.run(['git','rev-parse','HEAD'],cwd=root,capture_output=True,text=True,check=True).stdout.strip(),'scope':'single 12 s constant-speed 0.35 m/s slow_lead case, g1g2 seed 0, moving reserve','command_arguments':args,'cwd_wsl':str(root),'config_path':str(config.relative_to(root)),'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),'config_parent_path':str(parent.relative_to(root)),'config_parent_sha256':parent_hash,'summary_script':str((base/'slow_summary.py').relative_to(root)),'summary_script_sha256':hashlib.sha256((base/'slow_summary.py').read_bytes()).hexdigest(),'source_sha256':hashes,'source_count':len(hashes),'gear_ratio':ratio,'gear_ratio_source':'src/arena_description/config/vehicle.yaml','build':{'command':'colcon build --packages-select arena_bringup --symlink-install','exit_code':build_exit,'log':str(build.relative_to(root))},'output_absent_before_run':True,'environment':{'ros_setup':'/opt/ros/jazzy/setup.bash','workspace_setup':'install/local_setup.bash','PYTHONPATH_prefix':'/opt/ros/jazzy/lib/python3.12/site-packages:$PWD/src/arena_autonomy:$PWD/src/arena_vehicle_interface:$PWD/scripts:$PWD/tests'}}
manifest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'manifest':str(manifest.relative_to(root)),'source_count':len(hashes),'gear_ratio':ratio,'build_exit':build_exit,'command':args},ensure_ascii=False,indent=2))