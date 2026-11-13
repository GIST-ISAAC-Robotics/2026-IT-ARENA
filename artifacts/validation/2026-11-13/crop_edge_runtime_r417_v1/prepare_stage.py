"""Windows stdlib 전용 1회 준비. 공통 파일은 읽고 r417/staged만 생성한다."""
from pathlib import Path
import hashlib
import difflib

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
PREFIX = 'src/arena_autonomy/arena_autonomy/'


def stage(relative, replacements):
    source = ROOT / relative
    original = source.read_text(encoding='utf8')
    changed = original
    for before, after in replacements:
        assert changed.count(before) == 1, (relative, before, changed.count(before))
        changed = changed.replace(before, after)
    target = A / 'staged' / relative
    target.parent.mkdir(parents=True, exist_ok=True)
    if target.exists():
        assert target.read_bytes() == changed.encode('utf8'), target
    else:
        target.write_bytes(changed.encode('utf8'))


stage(PREFIX+'local_follow.py', [
    ("    confirmations: int = 2", "    confirmations: int = 2\n    crop_edge_component_trial: bool = False  # 명시 verified native 시뮬 경계 성분 후보."),
    ("        from arena_autonomy.known_structure import validate as validate_structure", "        from arena_autonomy.crop_edge_selection import validate_policy_selection\n        validate_policy_selection(self.crop_edge_component_trial, self.partial_observation_policy,\n                                  self.traffic_recovery_trial)\n        from arena_autonomy.known_structure import validate as validate_structure"),
    ("if k not in ('expiry_route_reference_trial'", "if k not in ('crop_edge_component_trial', 'expiry_route_reference_trial'"),
    ("oblique_metric_enabled=self.config.oblique_gap_trial and self.config.solo_route_only)", "oblique_metric_enabled=self.config.oblique_gap_trial and self.config.solo_route_only,\n                crop_edge_component_trial=self.config.crop_edge_component_trial)"),
])
stage(PREFIX+'local_pursuit.py', [
    ("        traffic_recovery = bool(self.declare_parameter('g1_traffic_recovery_trial', False).value)", "        traffic_recovery = bool(self.declare_parameter('g1_traffic_recovery_trial', False).value)\n        crop_edge = self.declare_parameter('g1_crop_edge_component_trial', False).value\n        from arena_autonomy.crop_edge_selection import validate_policy_selection, validate_node_selection\n        validate_policy_selection(crop_edge, partial_observation, traffic_recovery)\n        if crop_edge and not self.g1_enabled:\n            raise ValueError('crop edge component requires explicit G1 activation')\n        validate_node_selection(self, crop_edge)"),
    ("            partial_observation_policy=partial_observation,", "            partial_observation_policy=partial_observation,\n            crop_edge_component_trial=crop_edge,"),
])
stage(PREFIX+'lidar_safety.py', [
    ("        if policy != 'strict':\n", "        crop_edge = self.declare_parameter('g1_crop_edge_component_trial', False).value\n        from arena_autonomy.crop_edge_selection import validate_policy_selection, validate_node_selection\n        validate_policy_selection(crop_edge, policy, traffic_recovery)\n        validate_node_selection(self, crop_edge)\n        if policy != 'strict':\n"),
    ("                                                          oblique_metric_enabled=oblique)", "                                                          oblique_metric_enabled=oblique,\n                                                          crop_edge_component_trial=crop_edge)"),
])
stage(PREFIX+'native_lidar_input.py', [
    ("def subscribe_scan(node,callback,qos):", "def input_representation(node):\n    \"\"\"먼저 선택을 검사한 노드에서도 선언된 원값/기존 기본 표현을 유지한다.\"\"\"\n    name = 'lidar_input_representation'\n    parameter = node.get_parameter(name) if node.has_parameter(name) else node.declare_parameter(name, 'laser_scan')\n    return str(parameter.value)\n\n\n+def subscribe_scan(node,callback,qos):".replace('\n+def', '\ndef')),
    ("    mode=str(node.declare_parameter('lidar_input_representation','laser_scan').value)", "    mode=input_representation(node)"),
])
launch='src/arena_bringup/launch/simulation.launch.py'
stage(launch, [
    ("    if native_sampling:\n", "    crop_edge = _as_bool(LaunchConfiguration('g1_crop_edge_component_trial').perform(context))\n    if crop_edge and (not local_mode or not native_sampling):\n        raise ValueError('crop edge component requires explicit native local_pursuit simulation')\n    if native_sampling:\n"),
    ('            "lidar_input_representation": \'native_simulation_v1\' if native_sampling else \'laser_scan\',', '            "g1_crop_edge_component_trial": crop_edge,\n            "lidar_input_representation": \'native_simulation_v1\' if native_sampling else \'laser_scan\','),
    ('            "g1_runtime_policy": LaunchConfiguration("g1_runtime_policy").perform(context),', '            "g1_crop_edge_component_trial": crop_edge,\n            "g1_runtime_policy": LaunchConfiguration("g1_runtime_policy").perform(context),'),
    ("            DeclareLaunchArgument('g1_current_corner_budget', default_value='off',", "            DeclareLaunchArgument('g1_crop_edge_component_trial', default_value='false',\n                                  choices=['false', 'true'],\n                                  description='명시 verified native 시뮬 경계 성분 후보; 계획/독립 보호에 같은 선택'),\n            DeclareLaunchArgument('g1_current_corner_budget', default_value='off',"),
])
source=A.parent/'crop_edge_component_r414_v1/staged/arena_autonomy/partial_observation.py'
assert hashlib.sha256(source.read_bytes()).hexdigest() == '576966260591adf169c426f33c921df0586ca75f31e9e973c69be94b2c268425'
target=A/'staged'/PREFIX/'partial_observation.py'
if target.exists():
    assert target.read_bytes() == source.read_bytes()
else:
    target.write_bytes(source.read_bytes())

stage('src/arena_bringup/launch/g1_follow.launch.py', [
    ("        DeclareLaunchArgument('g1_oblique_gap_trial'", "        DeclareLaunchArgument('g1_crop_edge_component_trial', default_value='false', choices=['true', 'false'],\n                              description='명시 verified native 시뮬 경계 성분; 계획/독립 보호 각각 계산'),\n        SetParameter(name='g1_crop_edge_component_trial',\n                     value=ParameterValue(LaunchConfiguration('g1_crop_edge_component_trial'), value_type=bool)),\n        DeclareLaunchArgument('g1_oblique_gap_trial'"),
])
