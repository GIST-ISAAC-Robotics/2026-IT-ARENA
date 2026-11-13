"""실행 추가 없이 확보한 실패/좌표/회수 결과만 Windows에서 인계한다."""
import hashlib,json,math,shutil
from pathlib import Path
root=Path(__file__).resolve().parent
for name,dest in [('result.json','result_windows_v11.json'),('RESULT.md','RESULT_windows_v11.md')]:
    if not (root/dest).exists():shutil.copyfile(root/name,root/dest)
before=json.loads((root/'result_windows_v11.json').read_bytes())
run=json.loads((root/'execution_v1/run1/result.json').read_bytes())
cleanup=json.loads((root/'execution_v1/pause_pid_audit.json').read_bytes())
metrics=dict(image_width_px=1280,image_height_px=720,opencv='4.6.0',same_id=1,
    outer_area_px2=6241,inner_area_px2=4625,intersection_over_union=4625/6241,
    mean_edge_ratio=79/math.sqrt(4625),center_difference_px=0.,max_corner_difference_px=math.sqrt(61),
    outer_black_code_edge_px=79.,inner_edge_px=math.sqrt(4625),
    source='execution_v1/duplicate_diagnostic.json',
    interpretation='same centered synthetic code yielded outer/inner detections; original 2 px equivalence rejected it',
    physical_accuracy_verified=False)
(root/'execution_v1/duplicate_metrics.json').write_text(json.dumps(metrics,indent=2)+'\n')
current=dict(before,status='paused_after_first_ROS_failure_duplicate_handling_unfixed',
    ros_runtime_executed=True,wsl_used=True,ros_test_exit_code=1,
    ros_import_tests_passed=2,ros_representative_passed=False,
    ros_failure='front cap absent: same marker rejected as ambiguous_duplicate',
    ros_result='execution_v1/run1/result.json',ros_first_result_preserved=True,
    duplicate_fix_implemented=False,additional_ROS_retry_started=False,
    owned_child_processes_remaining=cleanup['owned_matching_count'],
    owned_process_audit='execution_v1/pause_pid_audit.json',wsl_returned_to_root=True,
    unchanged_stage_archive_sha256=hashlib.sha256((root/'stage_source.zip').read_bytes()).hexdigest())
(root/'result.json').write_text(json.dumps(current,indent=2)+'\n')
print(json.dumps(dict(status=current['status'],archive=current['unchanged_stage_archive_sha256'],
    owned_processes=cleanup['owned_matching_count'],metrics=metrics),indent=2))
