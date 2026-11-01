import hashlib,json,pathlib,sys
root=pathlib.Path.cwd(); base=root/'artifacts/validation/2026-11-01/race_follow_pair_v1'
manifest=base/'paired_run_pre_manifest.json'
if manifest.exists(): raise SystemExit(f'refusing to overwrite {manifest}')
for name in ('fixed','moving'):
 if (base/name).exists(): raise SystemExit(f'refusing to reuse existing output directory {base/name}')
# The validator declares the exact source/config/world inputs it fingerprints in its report.
sys.path.insert(0,str(root/'scripts'))
sys.path.insert(0,str(root/'src/arena_autonomy'))
sys.path.insert(0,str(root/'src/arena_vehicle_interface'))
import validate_g1_traffic as validator
config=base/'stop_restart.json'; cfg=json.loads(config.read_text(encoding='utf-8'))
parent=root/cfg['parent']['path']; parent_hash=hashlib.sha256(parent.read_bytes()).hexdigest()
if parent_hash!=cfg['parent']['sha256']: raise SystemExit(f'parent config hash mismatch: expected {cfg["parent"]["sha256"]}, actual {parent_hash}')
files=set(validator.SOURCE_FILES)|{str(config.relative_to(root))}
probe=pathlib.Path('artifacts/validation/2026-10-07/g1b_contact_probe_v1/report.json')
if probe.exists(): files.add(str(probe))
hashes={}
for rel in sorted(files):
 p=root/rel
 if not p.is_file(): raise SystemExit(f'missing referenced input file: {rel}')
 hashes[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
vehicle=root/'src/arena_description/config/vehicle.yaml'
vehicle_text=vehicle.read_text(encoding='utf-8')
ratio_line=next((line.strip() for line in vehicle_text.splitlines() if 'gear_ratio:' in line and 'gear_ratio_status' not in line),None)
if ratio_line is None: raise SystemExit('gear_ratio not found in vehicle.yaml')
ratio=float(ratio_line.split(':',1)[1].strip())
base_args=['python3','-B','scripts/validate_g1_traffic.py','--config','artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json','--mode','g1g2','--case','stop_restart','--seed','0','--record-follow-decisions']
commands={reserve:base_args+['--handoff-reserve',reserve,'--output',f'artifacts/validation/2026-11-01/race_follow_pair_v1/{reserve}'] for reserve in ('fixed','moving')}
existing={}
for name in ('before.zip','README.md','compare.py'):
 p=base/name
 if p.exists(): existing[str(p.relative_to(root))]=hashlib.sha256(p.read_bytes()).hexdigest()
build_log=base/'colcon_build_arena_bringup.log'
build_text=build_log.read_text(encoding='utf-8')
build_exit=next((int(line.split('=',1)[1]) for line in reversed(build_text.splitlines()) if line.startswith('ACTUAL_EXIT_CODE=')),None)
result={'created_before_run':True,'git_head':__import__('subprocess').run(['git','rev-parse','HEAD'],cwd=root,capture_output=True,text=True,check=True).stdout.strip(),'scope':'paired 15 s stop_restart, g1g2, seed 0; only handoff reserve fixed/moving differs','config_path':str(config.relative_to(root)),'config_sha256':hashlib.sha256(config.read_bytes()).hexdigest(),'config_parent_path':str(parent.relative_to(root)),'config_parent_sha256':parent_hash,'commands':commands,'source_sha256':hashes,'source_file_count':len(hashes),'validator_source_files_count':len(validator.SOURCE_FILES),'drive_gear_ratio':ratio,'gear_ratio_source':str(vehicle.relative_to(root)),'handoff_reserve_values':['fixed','moving'],'expected_case_duration_s':15,'build':{'command':'colcon build --packages-select arena_bringup --symlink-install','exit_code':build_exit,'log':str(build_log.relative_to(root))},'preserved_existing_file_sha256':existing,'output_directories_absent_before_run':True,'environment':{'ros_setup':'/opt/ros/jazzy/setup.bash','workspace_setup':'install/local_setup.bash','PYTHONPATH':'existing sourced environment; validator requires scripts and local packages'}}
manifest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'manifest':str(manifest.relative_to(root)),'source_files':len(hashes),'ratio':ratio,'build_exit':build_exit,'commands':commands},ensure_ascii=False,indent=2))