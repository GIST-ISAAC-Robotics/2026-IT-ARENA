import hashlib,json,pathlib,subprocess,sys
root=pathlib.Path.cwd(); base=root/'artifacts/validation/2026-11-01/race_follow_closing_v1'; manifest=base/'closing_pre_manifest.json'
if manifest.exists(): raise SystemExit('refusing to overwrite '+str(manifest))
for n in ('slow','stop'):
 if (base/n).exists(): raise SystemExit('refusing to reuse '+str(base/n))
for n in ('slow_console.log','stop_console.log','slow_exit.log','stop_exit.log','slow_surface.json','stop_surface.json','slow_surface.log','stop_surface.log','slow_summary.json','stop_summary.json','slow_summary.log','stop_summary.log','slow_compare.json','stop_compare.json'):
 if (base/n).exists(): raise SystemExit('refusing to overwrite '+str(base/n))
sys.path.insert(0,str(root/'scripts'));sys.path.insert(0,str(root/'src/arena_autonomy'));sys.path.insert(0,str(root/'src/arena_vehicle_interface'))
import validate_g1_traffic as validator
configs={'slow':'artifacts/validation/2026-11-01/race_follow_pair_v1/slow_lead.json','stop':'artifacts/validation/2026-11-01/race_follow_pair_v1/stop_restart.json'}
commands={}
for name,rel in configs.items():
 case='slow_lead' if name=='slow' else 'stop_restart'
 commands[name]=['python3','-B','scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','closing','--config',rel,'--case',case,'--seed','0','--record-follow-decisions','--output',f'artifacts/validation/2026-11-01/race_follow_closing_v1/{name}']
files=set(validator.SOURCE_FILES)|set(configs.values())|{'scripts/build_experimental_track.py','artifacts/validation/2026-11-01/race_follow_closing_v1/compare.py','artifacts/validation/2026-11-01/race_follow_closing_v1/README.md','artifacts/validation/2026-11-01/race_follow_closing_v1/baseline_approach.json','artifacts/validation/2026-11-01/race_follow_pair_v1/slow_moving/report.json','artifacts/validation/2026-11-01/race_follow_pair_v1/moving/report.json'}
probe=pathlib.Path('artifacts/validation/2026-10-07/g1b_contact_probe_v1/report.json')
if probe.exists():files.add(str(probe))
hashes={}
for rel in sorted(files):
 p=root/rel
 if not p.is_file():raise SystemExit('missing recorded input '+rel)
 hashes[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
ratio_line=next(line.strip() for line in (root/'src/arena_description/config/vehicle.yaml').read_text().splitlines() if 'gear_ratio:' in line and 'gear_ratio_status' not in line)
ratio=float(ratio_line.split(':',1)[1].strip())
launch=root/'src/arena_bringup/launch/simulation.launch.py'; installed=root/'install/arena_bringup/share/arena_bringup/launch/simulation.launch.py'
if not installed.exists() or installed.resolve()!=launch.resolve() or hashlib.sha256(installed.resolve().read_bytes()).hexdigest()!=hashes['src/arena_bringup/launch/simulation.launch.py']:
 raise SystemExit('installed simulation.launch.py does not match current source; build/recheck required')
parent_hash=hashes['config/tests/g1_traffic_v1.json']
for rel in configs.values():
 cfg=json.loads((root/rel).read_text())
 if cfg['parent']['sha256']!=parent_hash:raise SystemExit('traffic config parent hash mismatch: '+rel)
result={'created_before_runs':True,'git_head':subprocess.run(['git','rev-parse','HEAD'],cwd=root,capture_output=True,text=True,check=True).stdout.strip(),'scope':'closing reserve paired once per existing slow_lead 12 s and stop_restart 15 s scenario; seed 0; 1.5 m/s from registered configs','commands':commands,'source_sha256':hashes,'source_file_count':len(hashes),'gear_ratio':ratio,'gear_ratio_source':'src/arena_description/config/vehicle.yaml','configs_sha256':{rel:hashes[rel] for rel in configs.values()},'source_parent_config_sha256':parent_hash,'baseline_reports':{n:hashes[f'artifacts/validation/2026-11-01/race_follow_pair_v1/{n}/report.json'] for n in ('slow_moving','moving')},'launch_verification':{'ros_imports':['arena_autonomy','arena_vehicle_interface'],'installed_launch':str(installed.relative_to(root)),'installed_launch_is_symlink':installed.is_symlink(),'resolved_target':str(installed.resolve().relative_to(root)),'matches_source':True,'matches_source_sha256':hashes['src/arena_bringup/launch/simulation.launch.py'],'autonomy_executable_accepts_closing':True,'evaluation_handoff_reserve_choices':['off','fixed','moving','closing'],'build_needed':False},'build_status':'not run; installed launch resolves to the current source symlink','output_dirs_absent_before_runs':True}
manifest.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({'manifest':str(manifest.relative_to(root)),'source_count':len(hashes),'gear_ratio':ratio,'build_needed':False,'commands':commands},ensure_ascii=False,indent=2))