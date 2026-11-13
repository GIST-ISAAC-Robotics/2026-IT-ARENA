"""동결 소스는 읽기만 하고 r366 사본·패치·AST 결과를 산출물 폴더에 만든다."""
import ast
import difflib
import hashlib
import json
from pathlib import Path

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
STAGED = OUT / 'staged'
PRODUCT = 'src/arena_autonomy/arena_autonomy/'
changes = {}


def read(name):
    path = PRODUCT + name
    return path, (ROOT / path).read_text(encoding='utf-8')


def replace(text, old, new, count=1):
    assert text.count(old) == count, (old[:90], text.count(old), count)
    return text.replace(old, new)


path, before = read('race_session.py')
after = replace(before, 'COMMAND_KEYS = {\n', "COMMAND_KEYS = {\n    'manual_visual_start': {'v', 'request', 'op', 'session', 'race_epoch', 'intervention',\n                            'signal_confirmed'},\n")
after = replace(after, '    def handle_command(self, payload, now_s=None, wall_s=None):', '''    def _manual_visual_start(self, request, payload, now_s, check):
        """시각 출발 사실만 운영자 확인으로 대체한다. 기존 구동/센서/보호 관문은 유지한다."""
        reason = None
        if payload['signal_confirmed'] is not True:
            reason = 'signal_confirmation_required'
        elif type(payload['intervention']) is not int or payload['intervention'] != self.permission.intervention:
            reason = 'intervention_mismatch'
        elif self.permission.started:
            reason = 'race_already_started'
        elif self.permission.latch is not None or self.resume is not None:
            reason = 'wait_green_required'
        elif self.vision.epoch != self.bound_vision_epoch:
            reason = 'vision_epoch_mismatch'
        elif not _finite(now_s) or now_s <= 0:
            reason = 'clock_unavailable'
        elif not callable(check) or check() is not True:
            # HTTP 표시나 payload의 주장이 아니라 현재 노드의 기존 WAIT_GREEN 상태를 대조한다.
            reason = 'wait_green_required'
        if reason is not None:
            return self._result(request, 'manual_visual_start', False, reason)
        self.permission.started = True
        self.start_source = 'operator_confirmed'
        # signal.started/last_visual_stamp를 조작하지 않는다. 신호 계산만 끝내고 마커는 유지한다.
        self.vision.signal_detection_enabled = False
        self._event('manual_visual_start', request=request, start_source=self.start_source)
        return self._result(request, 'manual_visual_start', True, 'operator_confirmed_start_fact',
                            start_source=self.start_source)

    def handle_command(self, payload, now_s=None, wall_s=None, *, manual_start_check=None):''')
after = replace(after, "        if op in ('recovery_reset', 'resume_start', 'restore_race'):\n", "        if op == 'manual_visual_start':\n            return self._manual_visual_start(request, payload, now_s, manual_start_check)\n        if op in ('recovery_reset', 'resume_start', 'restore_race'):\n")
changes[path] = (before, after)

path, before = read('local_pursuit.py')
after = replace(before, '        result = self.race.handle_command(payload, self.now_s(), time.monotonic())', '''        manual = isinstance(payload, dict) and payload.get('op') == 'manual_visual_start'
        check = self.manual_visual_start_waiting if manual else None
        result = self.race.handle_command(payload, self.now_s(), time.monotonic(), manual_start_check=check)''')
after = replace(after, '    def on_operator_link(self, message):', '''    def manual_visual_start_waiting(self):
        """수신 시 기존 WAIT_GREEN 조건을 다시 본다. 운동 투영/계획/MCU ARM을 실행하지 않는다."""
        now, wall = self.now_s(), time.monotonic()
        if not self.enabled or self.last_status != 'WAIT_GREEN' or now < self.last_control_time:
            return False
        # WallFollow.control의 WAIT_GREEN 앞 센서 관문과 같은 한도를 재사용한다.
        scan_fresh = (self.scan is not None and
                      0 <= now-stamp_seconds(self.scan.header.stamp) < self.settings['scan_timeout_s'])
        image_fresh = self.rgb_stamp is not None and 0 <= now-self.rgb_stamp < self.settings['image_timeout_s']
        timeout = self.settings.get('sensor_wall_timeout_s', 3.)
        wall_fresh = wall-self.scan_wall_time < timeout and wall-self.image_wall_time < timeout
        return bool(scan_fresh and image_fresh and wall_fresh)

    def on_operator_link(self, message):''')
changes[path] = (before, after)

path, before = read('race_ops_core.py')
after = replace(before, "'prepare_start', 'new_race', 'restore_race', 'set_speed_limit')", "'prepare_start', 'manual_visual_start', 'new_race', 'restore_race', 'set_speed_limit')")
after = replace(after, '        result.update(\n', "        result.update(\n            manual_visual_start=(bool((self.status or {}).get('state') == 'WAIT_GREEN' and not started\n                                      and not latch and not race.get('resume')), 'wait_green_required'),\n")
after = replace(after, "        if op == 'new_race' and confirm != 'NEW_RACE':", "        if op == 'manual_visual_start' and confirm != 'SIGNAL_CONFIRMED':\n            return 'signal_confirmation_required'\n        if op == 'new_race' and confirm != 'NEW_RACE':")
after = replace(after, "            if 'speed_limit_mps' in payload:", "            if 'start_source' in payload:\n                entry['policy']['start_source'] = payload['start_source']\n            if 'speed_limit_mps' in payload:")
after = replace(after, "started=race.get('started'), speed_limit=", "started=race.get('started'), start_source=race.get('start_source'), speed_limit=")
after = replace(after, "latch='래치', started='출발 사실', speed_limit=", "latch='래치', started='출발 사실', start_source='출발 확인 출처', speed_limit=")
changes[path] = (before, after)

path, before = read('race_ops_node.py')
after = replace(before, "        elif op == 'set_speed_limit':\n", "        elif op == 'manual_visual_start':\n            self.publish({**self.bound(entry, op), 'signal_confirmed': True})\n        elif op == 'set_speed_limit':\n")
changes[path] = (before, after)

path, before = read('race_ops_page.py')
after = replace(before, '<div id="restorebox" hidden>', '<div class="row" style="margin-top:4px"><button id="manualstart" disabled>신호 확인 후 수동 출발</button></div>\n<p class="muted" style="margin:4px 0">출발 신호를 직접 확인했을 때만 누릅니다. 카메라 검출 대신 운영자 수동 확인으로 기록됩니다. 버튼 수락은 즉시 구동을 보장하지 않으며 기존 센서·MCU·보호 조건은 유지됩니다.</p>\n<div id="restorebox" hidden>')
after = replace(after, "prepare_start:'출발 준비',", "prepare_start:'출발 준비',manual_visual_start:'신호 확인 후 수동 출발',")
after = replace(after, " $('reset').disabled=off('recovery_reset');", " $('manualstart').disabled=off('manual_visual_start');\n $('reset').disabled=off('recovery_reset');")
after = replace(after, "['reset','start','prepare','newrace','restore','setspeedlimit']", "['reset','start','prepare','manualstart','newrace','restore','setspeedlimit']")
after = replace(after, "$('setspeedlimit').onclick=", "$('manualstart').onclick=()=>{if(!busy&&!$('manualstart').disabled){$('manualstart').disabled=true;send('manual_visual_start','SIGNAL_CONFIRMED')}};\n$('setspeedlimit').onclick=")
changes[path] = (before, after)

path, before = read('vision_process.py')
after = replace(before, "        start.update(rgb, request['stamp'], observations)", "        if request.get('signal_detection_enabled', True):\n            start.update(rgb, request['stamp'], observations)")
after = replace(after, "        if signal_mode in ('official_4red_candidate', 'official_4red_local_candidate'):\n            start.update(rgb, request['stamp'])\n        else:\n            start.update(rgb)", "        if request.get('signal_detection_enabled', True):\n            if signal_mode in ('official_4red_candidate', 'official_4red_local_candidate'):\n                start.update(rgb, request['stamp'])\n            else:\n                start.update(rgb)")
after = replace(after, '        self.signal = make_start_signal(signal_mode)\n', '        self.signal = make_start_signal(signal_mode)\n        self.signal_detection_enabled = True\n')
after = replace(after, '        self.signal = make_start_signal(self.signal_mode)\n', '        self.signal = make_start_signal(self.signal_mode)\n        self.signal_detection_enabled = True\n')
after = replace(after, "                    self.signal.__dict__.update(result['signal'])", "                    if self.signal_detection_enabled:\n                        self.signal.__dict__.update(result['signal'])")
after = replace(after, '                        signal_mode=self.signal_mode,\n', '                        signal_mode=self.signal_mode, signal_detection_enabled=self.signal_detection_enabled,\n')
after = replace(after, "        return dict(mode='process', signal_mode=self.signal_mode, epoch=self.epoch, fault=self.fault, stats=dict(self.stats),", "        return dict(mode='process', signal_mode=self.signal_mode, signal_detection_enabled=self.signal_detection_enabled,\n                    epoch=self.epoch, fault=self.fault, stats=dict(self.stats),")
changes[path] = (before, after)

for source in sorted((OUT / 'test_drafts').glob('test_*.py')):
    changes['tests/' + source.name] = ('', source.read_text(encoding='utf-8'))

manifest, patch = [], []
for path, (before, after) in changes.items():
    ast.parse(after, filename=path)
    destination = STAGED / path
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(after, encoding='utf-8', newline='\n')
    original_path = ROOT / path
    manifest.append(dict(path=path, original_sha256=hashlib.sha256(original_path.read_bytes()).hexdigest()
                         if original_path.exists() else None,
                         staged_sha256=hashlib.sha256(destination.read_bytes()).hexdigest()))
    patch.append(f'diff --git a/{path} b/{path}\n')
    if not before:
        patch.append('new file mode 100644\n')
    patch.extend(difflib.unified_diff(before.splitlines(keepends=True), after.splitlines(keepends=True),
                                    fromfile='a/'+path if before else '/dev/null', tofile='b/'+path))
(OUT / 'manual_visual_start.patch').write_text(''.join(patch), encoding='utf-8', newline='\n')
(OUT / 'manifest.json').write_text(json.dumps(dict(stage='prepared_not_applied', ast_passed=True,
    runtime_tests_executed=False, files=manifest), ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(staged_files=len(manifest), ast_passed=True, runtime_tests_executed=False)))
