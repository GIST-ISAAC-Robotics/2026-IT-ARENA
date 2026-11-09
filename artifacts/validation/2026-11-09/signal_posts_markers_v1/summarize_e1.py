"""E1 저장 결과 집계. 새로운 실행이나 원 판정 수정 없음."""
import json
from pathlib import Path
from summarize_solo import summarize
A=Path(__file__).resolve().parent
out=summarize(A/'e_all_posts_blank')
out['marker_post_evidence']=json.loads((A/'e_all_posts_blank/marker_post_evidence.json').read_text())
(A/'e1_summary.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({k:out[k] for k in ('passed','progress_m','elapsed_to_last_sample_s','peak_ground_speed_mps',
    'sampled_10kmh_time_s','longest_low_interval_s','p21_window','p22_window','stop_stable','shutdown_clean')},indent=1))
