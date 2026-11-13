"""허용한 stage 사본만 연결한다. 문자열 일치 실패는 멈추며 공통은 쓰지 않는다."""
import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parent;repo=root.parents[3]
base=root/'stage/src/arena_autonomy/arena_autonomy'
manifest=json.loads((root/'original_sha256.json').read_text(encoding='utf-8'))
for name in ('local_follow.py','m3_execution_trial.py'):
    rel='src/arena_autonomy/arena_autonomy/'+name
    source=repo/rel;original=root/'original'/rel
    if original.exists():raise RuntimeError('already captured '+rel)
    shutil.copyfile(source,original);shutil.copyfile(source,base/name)
    manifest[rel]=hashlib.sha256(source.read_bytes()).hexdigest()
(root/'original_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')

def edit(name,old,new,count=1):
    p=base/name;s=p.read_text(encoding='utf-8')
    if s.count(old)!=count:raise RuntimeError(f'{name}: expected {count}, found {s.count(old)}')
    p.write_text(s.replace(old,new),encoding='utf-8',newline='\n')

edit('local_pursuit.py','        self.vision_pipeline = None\n',
    "        from arena_autonomy.vehicle_marker_channel import MarkerChannel\n"
    "        self.vehicle_marker_channel = MarkerChannel(self, producer=True)\n"
    "        if self.vehicle_marker_channel.profile is not None and self.vision_execution != 'process':\n"
    "            raise ValueError('metric vehicle markers require process vision')\n"
    '        self.vision_pipeline = None\n')
edit('local_pursuit.py','                vehicle_marker_ids=vehicle_ids)',
    '                vehicle_marker_ids=vehicle_ids, vehicle_region_profile=self.vehicle_marker_channel.profile)')
edit('local_pursuit.py',"            elif result is not None:\n                self.signal = self.vision_pipeline.signal",
    "            elif result is not None:\n                self.vehicle_marker_channel.publish(self.vision_pipeline)\n                self.signal = self.vision_pipeline.signal")

edit('follow_trace.py','verified, rear_half_angle, *, timing_policy=None):',
    'verified, rear_half_angle, *, timing_policy=None, vehicle_marker_regions=None):')
edit('follow_trace.py',"        'scan': raw,", "        'scan': raw,\n        **({'vehicle_marker_regions':vehicle_marker_regions} if vehicle_marker_regions is not None else {}),")

edit('runtime_node_backend.py','    return selected\n',
    "    channel = getattr(node, 'vehicle_marker_channel', None)\n"
    "    if channel is not None and channel.profile is not None:\n"
    "        selected.update(vehicle_marker_profile=channel.profile, vehicle_marker_clock_domain=channel.clock_domain)\n"
    '    return selected\n')
edit('runtime_node_backend.py','        snapshot = snapshot_call(',
    '        from arena_autonomy.vehicle_marker_channel import node_snapshot\n        snapshot = snapshot_call(')
edit('runtime_node_backend.py','            timing_policy=node.motion.timing_policy)\n        delivery.offer(snapshot)',
    '            timing_policy=node.motion.timing_policy, vehicle_marker_regions=node_snapshot(node, reference))\n        delivery.offer(snapshot)')
edit('g1_g2_node.py',"        if self.complete_plan_trace and not self.runtime_dual_selected:",
    "        if self.vehicle_marker_channel.profile is not None and not self.runtime_dual_selected:\n"
    "            raise ValueError('metric vehicle markers currently require registered dual plan/current runtime')\n"
    "        if self.complete_plan_trace and not self.runtime_dual_selected:")
edit('g1_g2_node.py','from pathlib import Path\n',
    'from pathlib import Path\nfrom arena_autonomy.vehicle_marker_channel import node_snapshot\n')
edit('g1_g2_node.py',"                    speed, self.side, self.last_steering, self.decision_output_dt_s,\n                    self.motion.verified, self.rear_blind_half_angle,\n                    timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'))",
    "                    speed, self.side, self.last_steering, self.decision_output_dt_s,\n                    self.motion.verified, self.rear_blind_half_angle,\n                    timing_policy=getattr(self.motion, 'timing_policy', 'verified_only'),\n                    vehicle_marker_regions=node_snapshot(self, now))",2)
edit('g1_g2_node.py',"    return freeze(result)\n", "    if 'vehicle_marker_occupancy' in detail:\n        result['vehicle_marker_occupancy'] = detail['vehicle_marker_occupancy']\n    return freeze(result)\n")

edit('race_runtime_policy/evaluator.py',"        self.stage = stage\n",
    "        self.stage = stage\n"
    "        self.vehicle_marker_profile = header.get('vehicle_marker_profile')\n"
    "        self.vehicle_marker_clock_domain = header.get('vehicle_marker_clock_domain')\n"
    "        if self.vehicle_marker_profile is not None:\n"
    "            from arena_autonomy.vehicle_marker_geometry import validate_profile\n"
    "            validate_profile(self.vehicle_marker_profile)\n"
    "            if self.vehicle_marker_clock_domain not in ('ros_sim','ros_system'):\n"
    "                raise ValueError('explicit vehicle marker clock domain required')\n")
edit('race_runtime_policy/evaluator.py',"        if self.stage == 'plan':\n            return freeze(replay_call(self.follow, data, self.motion))",
    "        if self.stage == 'plan':\n"
    "            if self.vehicle_marker_profile is None:\n"
    "                return freeze(replay_call(self.follow, data, self.motion))\n"
    "            from arena_autonomy.vehicle_marker_occupancy import marker_scope, apply_plan\n"
    "            _, _, history = restore_call(data, self.motion)\n"
    "            with marker_scope(data, history, self.follow.config, self.vehicle_marker_profile,\n"
    "                              self.vehicle_marker_clock_domain):\n"
    "                output = replay_call(self.follow, data, self.motion)\n"
    "                return freeze(apply_plan(output, data, history, self.follow.config,\n"
    "                    self.vehicle_marker_profile, self.vehicle_marker_clock_domain))")
edit('race_runtime_policy/evaluator.py','        return freeze(result)\n',
    "        if self.vehicle_marker_profile is not None:\n"
    "            from arena_autonomy.vehicle_marker_occupancy import apply_current\n"
    "            result = apply_current(result, info, history, self.follow.config,\n"
    "                self.vehicle_marker_profile, self.vehicle_marker_clock_domain)\n"
    '        return freeze(result)\n')

edit('local_follow.py','            available = max(0., clearance-cfg.standstill_gap_m)',
    '            from arena_autonomy.vehicle_marker_occupancy import additional_clearance\n'
    '            clearance = additional_clearance(clearance, steering, previous_steering, cfg,\n'
    '                horizon=max(2.5,horizon) if open_route else 2.5)\n'
    '            available = max(0., clearance-cfg.standstill_gap_m)')
edit('m3_execution_trial.py','        available = max(0., clearance-cfg.standstill_gap_m)',
    '        from arena_autonomy.vehicle_marker_occupancy import additional_clearance\n'
    '        clearance = additional_clearance(clearance, steering, previous_steering, cfg)\n'
    '        available = max(0., clearance-cfg.standstill_gap_m)')

edit('lidar_safety.py','        self.scan = self.command = None\n        self.scan_wall',
    "        from arena_autonomy.vehicle_marker_channel import MarkerChannel\n"
    "        self.vehicle_marker_channel = MarkerChannel(self, producer=False)\n"
    "        if self.vehicle_marker_channel.profile is not None:\n"
    "            from arena_autonomy.local_follow import FollowConfig\n"
    "            self.vehicle_marker_config = FollowConfig(wheelbase_m=self.wheelbase,\n"
    "                length_m=self.length, width_m=self.width, max_steering_rad=self.limit)\n"
    '        self.scan = self.command = None\n        self.scan_wall',1)
edit('lidar_safety.py','        self.input_floor_s = boundary\n',
    '        self.input_floor_s = boundary\n        self.vehicle_marker_channel.reset(boundary)\n')
edit('lidar_safety.py',"            self.input_floor_s = -math.inf\n",
    "            self.input_floor_s = -math.inf\n            self.vehicle_marker_channel.reset()\n")
edit('lidar_safety.py','        command_stale = None\n',
    '        command_stale = None\n        marker_evidence = None\n')
edit('lidar_safety.py','            reason = "clear" if cap >= speed else "lidar_braking"\n',
    '            reason = "clear" if cap >= speed else "lidar_braking"\n'
    '            packet = self.vehicle_marker_channel.snapshot(now)\n'
    '            if packet is not None:\n'
    '                from arena_autonomy.vehicle_marker_occupancy import constrain_command\n'
    '                limited = constrain_command((output.drive.speed, output.drive.steering_angle, {"reason":reason}),\n'
    '                    packet, self.motion.history, now, packet["vision_epoch"],\n'
    '                    self.previous_safe_steering, self.vehicle_marker_config)\n'
    '                output.drive.speed = limited[0]\n'
    '                marker_evidence = limited[2].get("vehicle_marker_occupancy")\n'
    '                if marker_evidence is not None: reason = "vehicle_marker_braking"\n')
edit('lidar_safety.py','                                                    "front_observation": vars(observation) if observation else None,',
    '                                                    **({"vehicle_marker_occupancy":marker_evidence} if marker_evidence is not None else {}),\n'
    '                                                    "front_observation": vars(observation) if observation else None,')
