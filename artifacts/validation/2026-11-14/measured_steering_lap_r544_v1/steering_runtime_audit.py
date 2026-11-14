"""실제 launch가 만든 Node 파라미터와 SDF를 기록한다. 값을 바꾸지 않는다."""
import hashlib,json,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
VEHICLE=ROOT/'src/arena_description/config/vehicle.measured_steering_2026_11_14.yaml'
SHA='dcb8a7130f3eea4a474aff4602b86547bb7f700e6fecaff3049bb8dbd32a5021'
LIMIT=.28475436568260193;JOINT=.30666234131856146

def install(module):
    original=module._launch_setup
    def audited(context):
        from arena_autonomy.vehicle_geometry import equivalent_steering_limit
        import yaml
        selected=Path(module.LaunchConfiguration('vehicle_config').perform(context)).resolve()
        assert selected==VEHICLE.resolve()
        assert hashlib.sha256(selected.read_bytes()).hexdigest()==SHA
        drive=yaml.safe_load(selected.read_text())['vehicle']['drivetrain'];physical=equivalent_steering_limit(drive)
        assert drive['max_steering_angle_rad']==JOINT
        captured=[];node=module.Node
        def record(*args,**kwargs):
            parameters={}
            for group in kwargs.get('parameters',[]):
                if isinstance(group,dict):parameters.update(group)
            if 'max_steering_angle_rad' in parameters:
                captured.append(dict(executable=kwargs['executable'],package=kwargs['package'],max_steering_angle_rad=parameters['max_steering_angle_rad']))
            return node(*args,**kwargs)
        module.Node=record
        try:actions=original(context)
        finally:module.Node=node
        found={r['executable']:r['max_steering_angle_rad'] for r in captured}
        expected={'local_pursuit':LIMIT,'lidar_safety':LIMIT,'virtual_mcu':physical,'guarded_sim_actuator':physical}
        checks={name:found.get(name)==value for name,value in expected.items()}
        sdf=A/'run1/runtime_vehicle.sdf';root=ET.parse(sdf).getroot()
        plugin=root.find("model/plugin[@name='arena::SingleMotorDrive']")
        checks['sdf_plugin']=float(plugin.findtext('steering_limit'))==JOINT
        joints={}
        for name in ('front_left_steering_joint','front_right_steering_joint'):
            elem=root.find("model/joint[@name='%s']/axis/limit"%name)
            bounds=[float(elem.findtext('lower')),float(elem.findtext('upper'))];joints[name]=bounds;checks[name]=bounds==[-JOINT,JOINT]
        result=dict(passed=all(checks.values()),checks=checks,vehicle_path=str(selected),vehicle_sha256=SHA,
            command_limit_rad=LIMIT,physical_limit_rad=physical,joint_limit_rad=JOINT,actual_node_parameters=captured,
            runtime_sdf_sha256=hashlib.sha256(sdf.read_bytes()).hexdigest(),joint_limits=joints,
            scope='Actual launch construction and generated SDF, before actions execute; no parameter mutation or wheel-angle measurement')
        (A/'steering_runtime_receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
        assert result['passed'],result
        return actions
    module._launch_setup=audited
