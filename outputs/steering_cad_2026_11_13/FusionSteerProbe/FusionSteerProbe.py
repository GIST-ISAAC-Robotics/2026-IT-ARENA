import adsk.core, adsk.fusion, json, math, os, traceback

OUT = os.path.join(os.path.dirname(__file__), 'sweep2.json')

def wrap(x):
    return (x + 180.0) % 360.0 - 180.0

def run(context):
    app = adsk.core.Application.get()
    if os.path.exists(OUT):
        app.userInterface.messageBox('Existing sweep preserved; change output path before running.')
        return
    data = {'document': app.activeDocument.name, 'units': 'degrees; transforms row-major, length cm', 'step_deg': 0.5, 'branches': {}, 'error': None, 'restoration': None}
    servo = None
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)
        matches = [x for x in design.rootComponent.allOccurrences if x.name.startswith('조향_애커만')]
        if len(matches) != 1:
            raise RuntimeError('Unexpected steering assembly count')
        assembly = matches[0]
        joints = [j.createForAssemblyContext(assembly) for j in assembly.component.asBuiltJoints]
        byname = {j.name: j for j in joints}
        servo = byname['서보_입력'].jointMotion
        left = byname['킹핀_4x8x3_좌'].jointMotion
        right = byname['킹핀_4x8x3_우'].jointMotion
        children = {c.name: c for c in assembly.childOccurrences}
        wl = children['바퀴회전_좌:1']
        wr = children['바퀴회전_우:1']
        if max(abs(wrap(math.degrees(x.rotationValue))) for x in (servo, left, right)) > 0.05:
            raise RuntimeError('Not in neutral; refusing to move')
        def record(target):
            ml = list(wl.transform2.asArray())
            mr = list(wr.transform2.asArray())
            heading_l = math.degrees(math.atan2(-ml[8], ml[0]))
            heading_r = math.degrees(math.atan2(-mr[8], mr[0]))
            return {'target_deg': target, 'servo_deg': math.degrees(servo.rotationValue), 'left_joint_raw_deg': math.degrees(left.rotationValue), 'right_joint_raw_deg': math.degrees(right.rotationValue), 'left_joint_deg': wrap(math.degrees(left.rotationValue)), 'right_joint_deg': wrap(math.degrees(right.rotationValue)), 'left_heading_deg': heading_l, 'right_heading_deg': heading_r, 'left_transform': ml, 'right_transform': mr}
        data['neutral'] = record(0)
        for direction, endpoint in [('positive', 23.4), ('negative', -23.6)]:
            rows = []
            data['branches'][direction] = rows
            targets = [math.copysign(i * 0.5, endpoint) for i in range(1, int(abs(endpoint) / 0.5) + 1)] + [endpoint]
            last = data['neutral']
            for target in targets:
                servo.rotationValue = math.radians(target)
                adsk.doEvents()
                row = record(target)
                rows.append(row)
                if abs(row['servo_deg'] - target) > 0.02 or max(abs(row['left_joint_deg']), abs(row['right_joint_deg'])) > 55 or max(abs(row['left_joint_deg']-last['left_joint_deg']), abs(row['right_joint_deg']-last['right_joint_deg'])) > 5:
                    raise RuntimeError('Motion discontinuity or input mismatch at ' + str(target))
                last = row
            for row in reversed(rows[:-1]):
                servo.rotationValue = math.radians(row['target_deg'])
                adsk.doEvents()
            servo.rotationValue = 0.0
            adsk.doEvents()
            if max(abs(wrap(math.degrees(x.rotationValue))) for x in (servo, left, right)) > 0.05:
                raise RuntimeError('Neutral did not restore between branches')
    except:
        data['error'] = traceback.format_exc()
    finally:
        if servo:
            try:
                start = math.degrees(servo.rotationValue)
                steps = max(1, int(abs(start) / 0.25) + 1)
                for i in range(steps - 1, -1, -1):
                    servo.rotationValue = math.radians(start * i / steps)
                    adsk.doEvents()
                data['restoration'] = {'servo_deg': math.degrees(servo.rotationValue), 'left_joint_deg': math.degrees(left.rotationValue), 'right_joint_deg': math.degrees(right.rotationValue)}
            except:
                data['restoration'] = {'error': traceback.format_exc()}
        with open(OUT, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        app.userInterface.messageBox('Steering sweep saved.\n' + (data['error'] or 'OK'))
