"""오프라인 후보 기록을 운영 schema5로 오인하거나 다른 후보로 재생하지 않는다."""
import json
from pathlib import Path
import sys
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from probe_closed_loop import CandidateTrace, replay_candidate, candidate_hash
from candidate import MovingReserveCandidate
import arena_autonomy.bypass_handoff as runtime
from arena_autonomy.follow_trace import replay_events
from arena_autonomy.lidar_motion import MotionConfig


def header(tmp_path):
    controller = MovingReserveCandidate(allow_simulation_trial=True)
    path = tmp_path/'candidate.jsonl'
    trace = CandidateTrace(path.resolve(), MotionConfig(), controller.config,
        mode='g1_g2_handoff_trial', execution_config=controller.execution)
    trace.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['schema'] == 905
    assert events[0]['data']['offline_candidate_sha256'] == candidate_hash()
    return events


def test_operating_replayer_rejects_offline_record(tmp_path):
    events = header(tmp_path)
    with pytest.raises(ValueError):
        replay_events(iter(events))


@pytest.mark.parametrize('field,value', [('schema',5),('scope','other'),('offline_candidate_sha256','0'*64)])
def test_mismatched_candidate_rejected_and_patch_restored(tmp_path,field,value):
    events = header(tmp_path)
    events[0]['data'][field] = value
    original = runtime.BypassHandoffFollow
    with pytest.raises(ValueError,match='unmatched offline candidate'):
        replay_candidate(iter(events))
    assert runtime.BypassHandoffFollow is original
