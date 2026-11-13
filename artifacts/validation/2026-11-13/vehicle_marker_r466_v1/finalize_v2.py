"""Windows-only sealing of already completed r466 v2 evidence; no execution."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parent


def read(name):
    return json.loads((ROOT / name).read_text(encoding="utf-8"))


def write(name, value):
    (ROOT / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


prior = read("revision_v2/result.json")
backup = ROOT / "revision_v2/result_pre_ros.json"
if not backup.exists():
    backup.write_bytes((ROOT / "revision_v2/result.json").read_bytes())
run = read("execution_v2/run1/result.json")
completion = read("execution_v2/completion.json")
assert run["passed"] and completion["test_exit_code"] == 0
assert completion["source_snapshot_unchanged"]
assert completion["reported_child_pids_alive"] == []
preserved = {name: {"expected": digest, "actual": sha(ROOT / name)}
             for name, digest in read("revision_v2/preserved.json").items()}
assert all(x["expected"] == x["actual"] for x in preserved.values())
changes = read("revision_v2/changes.json")
stage_checks = [{"path": x["path"], "expected": x["stage_sha256"],
                 "actual": sha(ROOT / "stage" / x["path"])} for x in changes]
assert all(x["expected"] == x["actual"] for x in stage_checks)
archive_checks = {}
for name, key in [("revision_v2/stage_source.zip", "stage_archive_sha256"),
                  ("execution_v2/source.zip", "ros_source_archive_sha256"),
                  ("execution_v2/prepared.json", "ros_prepared_sha256")]:
    archive_checks[name] = {"expected": prior[key], "actual": sha(ROOT / name)}
assert all(x["expected"] == x["actual"] for x in archive_checks.values())
write("revision_v2/final_seal.json", {"scope": "Windows file hash checks only; no rerun",
      "original_v1_preserved": preserved, "stage_files": stage_checks,
      "archives": archive_checks, "all_match": True})

audit = {
    "status": "not_completed_entry_failure", "exit_code": 1,
    "error_code": "Wsl/Service/E_UNEXPECTED",
    "command": "wsl.exe -d Ubuntu-24.04 -- python3 -B '/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1/ros_validation/final_pid_audit_v2.py'",
    "receipt_basis": "observed tool return, normalized error; not reconstructed raw stdout",
    "audit_script_executed": False, "independent_final_global_pid_audit_completed": False,
    "same_batch_reported_child_pids_alive": [],
    "same_batch_vision_child_exitcode": 0, "same_batch_nodes_destroyed": 3,
    "main_runner_exit_code": 0,
    "retry_or_restart_performed": False, "wsl_ownership_returned_to_root": True,
}
write("execution_v2/final_pid_audit_failure.json", audit)

packets = [json.loads(line) for line in (ROOT / "execution_v2/run1/marker_packets.jsonl").read_text().splitlines()]
front = next(row["packet"] for row in packets if row["phase"] == "front")
region = front["regions"][0]
write("revision_v2/ros_duplicate_evidence.json", {
    "source": "execution_v2/run1/marker_packets.jsonl", "phase": "front",
    "source_stamp_s": front["source_stamp_s"], "source_frame_id": front["source_frame_id"],
    "rear_center_xy_m": region["rear_center_xy_m"],
    "duplicate_resolution": region["duplicate_resolution"],
    "timing_verified": front["timing_verified"],
    "metric_accuracy_verified": front["metric_accuracy_verified"],
})
prior.update({
    "status": "v2_windows_and_bounded_ROS_passed_common_not_integrated",
    "ros_v2_executed": True, "pending": "root integration review; no new execution scheduled",
    "ros_result": "execution_v2/run1/result.json", "ros_exit_code": 0,
    "ros_import_tests_passed": 2, "ros_cases": run["cases"],
    "ros_final_zero_observed": run["final_zero_observed"],
    "ros_input_files": 232, "ros_source_snapshot_unchanged": True,
    "cleanup": audit,
    "project_numeric_native_v1": {"selected": False,
        "basis": "frozen source and executed command scope, not backend runtime instrumentation",
        "opencv_detector_PnP": "4.6 native OpenCV used", "native_build": False, "ARM_measurement": False},
    "limits": ["synthetic local ROS without actuator", "no hardware accuracy or installed calibration claim",
        "marker-only overtaking initiation is not connected; existing candidate sweeps/current/guard are connected",
        "late independent global PID audit could not enter WSL; same-batch normal child cleanup verified"],
})
write("revision_v2/result.json", prior)
latest = read("latest_revision.json")
latest["status"] = prior["status"]
write("latest_revision.json", latest)
write("result.json", {"status": prior["status"], "latest_revision": "revision_v2/result.json",
    "original_v1_failure": "execution_v1/run1/result.json",
    "historical_paused_summary": "revision_v2/before_result.json",
    "no_common_edits": True, "device_accessed": False, "ros_v2_passed": True,
    "independent_final_pid_audit_completed": False, "wsl_returned": True})

summary = """# r466 v2 인계 결과

선택형 차량 마커의 중첩 quad 처리를 수정했고, **Windows 관련 32검사 및 실제 ROS import 2검사·5.2초 대표가 통과**했다. 공통 소스는 미통합이며 WSL 소유권은 root에 반환했다. 원 v1 실패·아카이브는 그대로 보존했다.

- 원 OpenCV 4.6의 79/68 px 중첩 검출은 크기비·중심·IoU로 묶은 뒤 원 RGB의 검은 코드 border·bit·경계 표본으로 대표를 선택한다. 큰 quad 자동 선택은 아니다. 실제 ROS packet에서도 두 후보 중 하나가 지지되고 원 79 px 코드 외곽이 선택됐다.
- 실제 `VisionProcess → MarkerChannel → LidarSafety.timer`에서 부재 `.75 m/s`, 정면 `.067812~.20 m/s`와 `vehicle_marker_braking`, 측방 `.75 m/s`, TTL 만료 뒤 `.75 m/s`를 확인했다. 마지막0도 관측했다. RGB20개/packet20개, wheel/gyro 각517개, 거절0이다.
- 같은 실행 묶음에서 exit0, child 강제 종료 없이 exit0/join, 3노드 destroy, 생존 reported child `[]`, 232개 입력 불변을 확인했다. **후속 별도 `/proc` 감사 호출은 `Wsl/Service/E_UNEXPECTED` exit1로 진입하지 못했다. 전역 잔류 PID0을 별도 재확인했다고 쓰지 않는다.** 재호출·재시작 없이 반환했다.
- 프로젝트 `numeric native_v1`은 실행 소스/명령상 미선택이다. OpenCV 4.6의 detector/PnP는 사용했다. native build·Jetson ARM 비용·10 Hz 통합 부하 검증이 아니다. 추가7×7/PnP/중복 처리는 `stages_ms.markers` 밖이지만 전체 `duration_ms`에 포함된다.
- 단일 정면 `.2~.5 m` 표본의 간이 초점거리 CLI 계약을 확인했다. 실측값을 채우거나 절대 거리 정확도를 주장하지 않았다. 기존 추월 후보의 점유 검사까지 연결하며 **마커만으로 새 추월 후보를 시작하는 기능은 범위 밖**이다.

최신 근거: [v2 상세](revision_v2/README.md), [기계 결과](revision_v2/result.json), [실제 ROS 결과](execution_v2/run1/result.json), [회수](execution_v2/completion.json), [추가 감사 실패](execution_v2/final_pid_audit_failure.json), [최종 SHA 확인](revision_v2/final_seal.json).

이관은 [22파일 변경 목록](revision_v2/changes.json)과 [v2 stage archive](revision_v2/stage_source.zip)를 기준으로 한다. 이번 v2 runtime 수정은 geometry 한 파일이며 [v1 대비 diff](revision_v2/delta_from_v1.diff)를 제공한다. 이전 문서·상태는 `revision_v2/before_*`와 `PAUSED.md`에 남겼다. 실제 차량·설치 교정·완주 검증의 증거로 확대하지 않는다.
"""
(ROOT / "RESULT.md").write_text(summary, encoding="utf-8")

readme = ROOT / "README.md"
lines = readme.read_text(encoding="utf-8").splitlines()
lines[2] = "**최신 상태:** [revision_v2](revision_v2/README.md)의 중첩 quad 수정 후 Windows32검사·실제 ROS import2와 제한 대표가 통과했다. [현재 결과/회수 한계](RESULT.md)를 먼저 읽는다. 공통 미통합, WSL 반환 완료이며 이관 목록·아카이브는 [latest_revision.json](latest_revision.json)을 따른다."
lines[4] = "2026-11-14 격리 stage. 원 v1 ROS 정면 cap 실패는 보존했고 v2는 별도 결과로 기록했다. 공통 소스·Jetson·센서·ESP·모터는 변경/실행하지 않았다. 주행 성공, 설치 교정, 실차 안전의 증거가 아니다."
readme.write_text("\n".join(lines) + "\n", encoding="utf-8")
paused = ROOT / "PAUSED.md"
lines = paused.read_text(encoding="utf-8").splitlines()
lines[2] = "**보존된 중지 당시 기록입니다.** 사용자 재개 뒤 [v2 수정·Windows/ROS 결과](revision_v2/README.md)를 완료했습니다. 아래의 ‘수정 미적용’은 중지 당시 사실이며 현재 결과는 [RESULT.md](RESULT.md)를 따릅니다."
paused.write_text("\n".join(lines) + "\n", encoding="utf-8")
detail = ROOT / "revision_v2/README.md"
text = detail.read_text(encoding="utf-8")
text = text.replace("상태: **stage 수정·Windows 32개 검사 통과, 공식 ROS 재실행 대기**. WSL/Jetson/실물 장치를 사용하지 않았다.", "상태: **stage 수정·Windows 32검사·ROS import2 및 제한 대표 통과**. WSL 합성 ROS만 사용했으며 Jetson/실물 장치는 사용하지 않았다.")
text = text.replace("## 공식 ROS 재실행 준비", "## 실제 ROS 대표 결과")
text = text.replace("root가 WSL 단독권을 다시 전달한 뒤에만 다음 명령을 한 번 실행한다.", "root의 WSL 단독권 전달 뒤 다음 고정 명령을 한 번 실행했고 exit0으로 종료했다. 아래는 실행 이력이며 추가 실행 지시가 아니다.")
text = text.replace("실제 ROS cap 회복 여부는 아직 미검증이다.", "실제 합성 ROS에서 정면 cap이 적용되고 부재/측방/TTL 뒤 원출력이 유지됐다. 이 결과는 실차 주행이나 전체 plan 재생이 아니다.")
text += """

| 실제 ROS 사례 | 출력 속도(m/s) | 판정 |
|---|---:|---|
| 부재 | .75 | 원출력 유지 |
| 정면 | .067812~.20 | vehicle_marker_braking |
| 측방 | .75 | 원출력 유지 |
| TTL 만료 | .75 | 보조 packet 소거·원출력 복귀 |

마지막0을 관측했다. RGB20/20 및 packet20, wheel/gyro 각517, 거절0이었다. 실제 ROS의 중복 metadata도 count2/support1, IoU.7411, 검은 border/bit100%, 지지 경계4변으로 남았다. [packet 발췌](ros_duplicate_evidence.json)는 원 `execution_v2/run1/marker_packets.jsonl`에서 추출했다.

같은 실행 묶음에서 exit0·3노드 destroy·Vision child forced=false/exit0·reported child 생존[]·232입력 불변을 확인했다. 후속 별도 `/proc` 감사는 **WSL 진입 E_UNEXPECTED exit1로 미수행**이다. [실패 영수증](../execution_v2/final_pid_audit_failure.json)을 보존하며 별도 전역 PID0 확인으로 바꾸지 않는다. 재시작/재호출 없이 WSL을 root에 반환했다.

프로젝트 `numeric native_v1`은 frozen source/명령 범위상 미선택이며 별도 backend 런타임 계측은 하지 않았다. OpenCV4.6 detector/PnP는 사용했다. 추가7×7/PnP/중복 표본 비용은 `stages_ms.markers` 밖이지만 전체 `duration_ms`에 포함된다. 처리20개·합계193.036918ms·최대26.989059ms는 이 합성 WSL 대표의 기록이며 Jetson/실제 주행 부하의 증명이 아니다.

소스 archive와 원 v1 네 근거, stage22파일의 SHA를 Windows에서 다시 대조했다([final_seal.json](final_seal.json)). 공통 미통합이며 후속은 root의 이관 검토다. 이관 후 실제 설치/간이 보정은 별도 작업으로 남긴다.
"""
detail.write_text(text, encoding="utf-8")
print(json.dumps({"sealed": True, "stage_files": len(stage_checks), "original_v1_preserved": True,
                  "ros_passed": True, "independent_final_pid_audit_completed": False,
                  "wsl_returned": True}, ensure_ascii=False))
