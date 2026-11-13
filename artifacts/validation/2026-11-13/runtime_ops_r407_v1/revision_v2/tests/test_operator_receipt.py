"""저장 실패의 stale-ready 재현과 새 STOP binding 연결만 검사한다. ROS/장치 없음."""
import copy
import json
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
HELPERS = HERE.parent if (HERE.parent / 'operator_receipt.py').is_file() else HERE.parent / 'staged/r407'
sys.path.insert(0, str(HELPERS))
from operator_receipt import accepted_reply, policy_accepted, stopped_binding


class OperatorReceiptTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture = json.loads((HERE / 'failure_fixture.json').read_bytes())

    def setUp(self):
        self.f = copy.deepcopy(self.fixture)
        self.reply = self.f['stop_reply']
        self.race_id = self.f['resume_http']['request']['view']['race_id']

    @staticmethod
    def display(status):
        # Same field projection as frozen OpsCore.view lines335-338. This is a
        # pure test input, not a claim that a complete HTTP GET was recorded.
        race = status['race']
        return dict(connection=dict(status_stale=False),
            binding={k: race[k] for k in ('session', 'race_id', 'race_epoch', 'intervention')},
            race={k: race[k] for k in ('latch', 'resume_start_ready', 'resume')})

    def test_original_ready_state_cannot_authorize_new_stop_binding(self):
        old = self.display(self.f['state_before_stop'])
        self.assertTrue(old['race']['resume_start_ready'])
        self.assertEqual(old['binding'], self.f['resume_http']['request']['view'])
        self.assertTrue(policy_accepted(self.f['stop_entry']))
        self.assertIsNone(stopped_binding(old, self.reply, self.race_id))

    def test_later_matching_explicit_stop_yields_captured_view(self):
        new = self.display(self.f['state_after_stop'])
        selected = stopped_binding(new, self.reply, self.race_id)
        self.assertEqual(selected, new['binding'])
        self.assertEqual(selected['intervention'], 2)
        new['binding']['intervention'] = 3
        self.assertEqual(selected['intervention'], 2)

    def test_exact_session_epoch_intervention_and_race_are_required(self):
        for key, value in dict(session='other', race_epoch=1, intervention=3, race_id='other').items():
            with self.subTest(key=key):
                view = self.display(self.f['state_after_stop'])
                view['binding'][key] = value
                self.assertIsNone(stopped_binding(view, self.reply, self.race_id))

    def test_ready_alone_or_stale_or_active_resume_is_insufficient(self):
        for key, value in (('latch', 'process_start'), ('resume_start_ready', False), ('resume', {'phase': 'inputs'})):
            with self.subTest(key=key):
                view = self.display(self.f['state_after_stop']);view['race'][key] = value
                self.assertIsNone(stopped_binding(view, self.reply, self.race_id))
        view = self.display(self.f['state_after_stop']);view['connection']['status_stale'] = True
        self.assertIsNone(stopped_binding(view, self.reply, self.race_id))

    def test_missing_reply_stays_pending(self):
        self.assertIsNone(accepted_reply([], 'r407-stop', 'stop'))
        self.assertIsNone(stopped_binding(self.display(self.f['state_after_stop']), None, self.race_id))
        self.assertIsNone(accepted_reply([self.reply], 'another-request', 'stop'))
        self.assertEqual(accepted_reply([self.reply], 'r407-stop', 'stop'), self.reply)

    def test_original_resume_rejection_is_immediate_and_explicit(self):
        with self.assertRaisesRegex(RuntimeError, 'r407-resume rejected: intervention_mismatch'):
            accepted_reply([self.f['resume_rejection']], 'r407-resume', 'resume_start')
        entry = dict(request='r407-resume', stage='policy_rejected',
                     policy={k: self.f['resume_rejection'][k] for k in ('accepted', 'reason')})
        with self.assertRaisesRegex(RuntimeError, 'intervention_mismatch'):
            policy_accepted(entry)

    def test_adapter_rejection_is_immediate_while_pending_is_not_rejection(self):
        self.assertFalse(policy_accepted({}))
        self.assertFalse(policy_accepted(dict(stage='queued', policy=None)))
        with self.assertRaisesRegex(RuntimeError, 'status_stale'):
            policy_accepted(dict(request='example', stage='rejected_by_adapter', adapter_reason='status_stale'))

    def test_wrong_operation_or_nonstop_receipt_cannot_be_rebound(self):
        with self.assertRaisesRegex(RuntimeError, 'operation mismatch'):
            accepted_reply([self.reply], 'r407-stop', 'resume_start')
        for key, value in (('accepted', False), ('op', 'resume_start'), ('latch', 'process_start')):
            with self.subTest(key=key):
                reply = dict(self.reply);reply[key] = value
                with self.assertRaisesRegex(RuntimeError, 'explicit STOP receipt required'):
                    stopped_binding(self.display(self.f['state_after_stop']), reply, self.race_id)


if __name__ == '__main__':
    unittest.main(verbosity=2)
