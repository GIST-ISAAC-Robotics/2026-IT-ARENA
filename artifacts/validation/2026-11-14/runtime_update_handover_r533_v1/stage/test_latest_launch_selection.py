"""최신 설명 SHA·옛 상한 복사·선택형 compact의 파일 전용 회귀."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import refresh_steering_selection_metadata as refresh
import steering_launch_selection as launch

HERE=Path(__file__).resolve().parent
ROOT=launch.ROOT
OLD=HERE.parent/'steering_selection.assembled_2026_11_14.json'
PREVIOUS=ROOT/'artifacts/validation/2026-11-14/measured_steering_apply_r541_v1/selected_v2/steering_zero.selected.json'


class LatestSelectionTests(unittest.TestCase):
    def test_metadata_refresh_keeps_original_and_rejects_count_changes(self):
        oldraw=OLD.read_bytes();priorraw=PREVIOUS.read_bytes()
        with tempfile.TemporaryDirectory() as temp:
            out=Path(temp)/'fresh.json'
            result=refresh.refresh(OLD,PREVIOUS,out)
            self.assertTrue(result['metadata_only'])
            selected=json.loads(out.read_bytes())
            raw=(ROOT/selected['steering_profile']['path']).read_bytes()
            self.assertEqual(selected['steering_profile']['sha256'],hashlib.sha256(raw).hexdigest())
            self.assertEqual(launch.selected_arguments(out)['source_command_limit_rad'],0.28475436568260193)
            self.assertEqual((OLD.read_bytes(),PREVIOUS.read_bytes()),(oldraw,priorraw))
            with self.assertRaises(ValueError):launch.selected_arguments(OLD)
            with self.assertRaises(FileExistsError):refresh.refresh(OLD,PREVIOUS,out)
            current=json.loads(raw);changed=copy.deepcopy(current);changed['safe_min_count']-=1
            self.assertNotEqual(refresh.operational_value(current),refresh.operational_value(changed))

    def test_source_only_checks_both_actual_yaml_caps_and_optional_r547_status(self):
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);selection=folder/'selected.json';refresh.refresh(OLD,PREVIOUS,selection)
            base=folder/'runtime.yaml'
            old={node:{'ros__parameters':{'max_steering_angle_rad':.4578436017036438}}
                 for node in launch.limits.NODES}
            base.write_text(launch.limits.yaml.safe_dump(old),encoding='utf-8')
            with self.assertRaisesRegex(ValueError,'different steering cap'):
                launch.selected_arguments(selection,runtime_params=base)
            for node in launch.limits.NODES:old[node]['ros__parameters']['max_steering_angle_rad']=.28475436568260193
            base.write_text(launch.limits.yaml.safe_dump(old),encoding='utf-8')
            full=launch.selected_arguments(selection,runtime_params=base)
            compact=launch.selected_arguments(selection,runtime_params=base,guard_detail='compact_v1',
                                              guard_result_path='/home/q/new_run_not_executed')
            self.assertFalse(full['installed_profile_link_verified'])
            self.assertTrue(full['runtime_params_selected'])
            self.assertIn('safety_status_detail:=full',full['argument_fragments']['guard_with_status'])
            self.assertEqual(full['argument_fragments']['planner'],compact['argument_fragments']['planner'])
            self.assertEqual(full['argument_fragments']['host'],compact['argument_fragments']['host'])
            self.assertEqual(compact['argument_fragments']['race_ops'][-1],'manual_steering_limit_rad:=0.2')
            self.assertIn('safety_status_evidence_path:=/home/q/new_run_not_executed/guard_status_contexts.jsonl',
                          compact['argument_fragments']['guard_with_status'])
            for bad in (None,'relative','/','/home/q/../old'):
                with self.assertRaises(ValueError):launch.guard_arguments('compact_v1',bad)
            self.assertFalse(compact['device_io'])

    def test_latest_profile_flows_through_existing_generator_without_old_cap(self):
        sys.path.insert(0,str(ROOT/'tests'))
        from test_runtime_installed_limits import synthetic_bundle,BASE
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp);selection=folder/'selection.json';refresh.refresh(OLD,PREVIOUS,selection)
            chosen=json.loads(selection.read_bytes())
            synthetic=synthetic_bundle(folder);board=launch.installed.read_json(synthetic/'runtime_board.json')
            board['assembly_id']=chosen['assembly_id'];board['servo']['command_limit']=chosen['command_limit_rad']
            bp=folder/'board.json';bp.write_text(json.dumps(board),encoding='utf-8')
            ip=folder/'selected_installed'
            launch.installed.prepare(bp,ROOT/chosen['steering_profile']['path'],chosen['assembly_id'],ip)
            base=folder/'base.yaml';base.write_text(launch.limits.yaml.safe_dump(BASE),encoding='utf-8')
            runtime=folder/'runtime';launch.limits.prepare(ip,base,runtime)
            args=launch.selected_arguments(selection,installed_profile=ip,runtime_selection=runtime,
                                           runtime_params=runtime/'runtime_profile.yaml')
            self.assertTrue(args['installed_profile_link_verified'])
            self.assertEqual(args['argument_fragments']['host'],['--max-steering-rad','0.28475436568260193'])
            self.assertEqual(args['argument_fragments']['planner'],args['argument_fragments']['guard'])


if __name__=='__main__':unittest.main()
