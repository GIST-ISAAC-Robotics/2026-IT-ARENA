"""C2는 C1과 같은 장면/속도/시설에 r56 실행 코드만 대조한다."""
from pathlib import Path
A=Path(__file__).resolve().parent
s=(A/'c_run.py').read_text(encoding='utf8').replace('c_stationary_posts/run1','c_stationary_posts/run2').replace('c_stationary_posts/args.json','c_stationary_posts/args_run2.json')
(A/'c2_run.py').write_text(s,encoding='utf8',newline='\n')
s=(A/'c_run.sh').read_text(encoding='utf8').replace('run1','run2').replace('c_run.py','c2_run.py').replace('sources_before.txt','sources_before_run2.txt').replace('sources_after.txt','sources_after_run2.txt')
s=s.replace('src/arena_autonomy/arena_autonomy/local_path.py scripts/validate_g1_traffic.py', 'src/arena_autonomy/arena_autonomy/local_path.py src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py src/arena_autonomy/arena_autonomy/nearby_face_motion.py scripts/validate_g1_traffic.py')
(A/'c2_run.sh').write_text(s,encoding='utf8',newline='\n')
print('c2 scripts ready')
