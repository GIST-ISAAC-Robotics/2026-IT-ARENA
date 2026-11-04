"""어댑터 기록 전달 정리, CLI·진입점 연결. 치환 실패 시 중단."""
from pathlib import Path


def patch(path, pairs):
    target = Path(path)
    raw = target.read_bytes()
    crlf = raw.count(b'\r\n') == raw.count(b'\n') and b'\r\n' in raw
    text = raw.replace(b'\r\n', b'\n').decode('utf-8')
    for old, new in pairs:
        assert text.count(old) == 1, old
        text = text.replace(old, new)
    data = text.encode('utf-8')
    target.write_bytes(data.replace(b'\n', b'\r\n') if crlf else data)


patch('src/arena_autonomy/arena_autonomy/race_ops_core.py', [
("""        self.rows = deque(maxlen=MAX_ROWS)
""", """        self.rows = deque(maxlen=MAX_ROWS)
        self.unlogged = deque(maxlen=MAX_ROWS)
"""),
("""        self.events.append(event)
        self.rows.append(dict(row='event', **event))
        return event
""", """        self.events.append(event)
        self._row(dict(row='event', **event))
        return event

    def _row(self, row):
        self.rows.append(row)
        self.unlogged.append(row)
"""),
("""        row = self._sample(status, wall)
        self.rows.append(row)
        return row
""", """        row = self._sample(status, wall)
        self._row(row)
        return row
"""),
("""    def set_log_state(self, state):
""", """    def drain_rows(self):
        with self.lock:
            rows, self.unlogged = list(self.unlogged), deque(maxlen=MAX_ROWS)
            return rows

    def set_log_state(self, state):
"""),
])

patch('src/arena_autonomy/arena_autonomy/race_ops_node.py', [
("""        self.log = QueuedLog(self.write_log, capacity=2048)
        self.logged = 0
""", """        self.log = QueuedLog(self.write_log, capacity=2048)
"""),
("""            self.last_written, self.last_record = key, wall
            self.record_writer.submit(json.dumps(payload, ensure_ascii=False))
""", """            self.last_written, self.last_record = key, wall
            self.core.set_record(payload)
            self.record_writer.submit(json.dumps(payload, ensure_ascii=False))
"""),
("""    def tick_log(self, wall):
        rows = self.core.rows
        total = getattr(self.core, 'row_total', None)
        with self.core.lock:
            fresh = list(rows)[self.logged:] if len(rows) >= self.logged else list(rows)
            self.logged = len(rows) if len(rows) < rows.maxlen else 0 if not fresh else self.logged
            if len(rows) == rows.maxlen:
                rows.clear()
                self.logged = 0
        for row in fresh:
            self.log.submit(json.dumps(row, ensure_ascii=False, allow_nan=False, default=str))
""", """    def tick_log(self):
        for row in self.core.drain_rows():
            self.log.submit(json.dumps(row, ensure_ascii=False, allow_nan=False, default=str))
"""),
("""        self.tick_log(wall)
""", """        self.tick_log()
"""),
])

patch('src/arena_autonomy/setup.py', [
("""                                         "lidar_safety = arena_autonomy.lidar_safety:main"]})""",
 """                                         "lidar_safety = arena_autonomy.lidar_safety:main",
                                         "race_ops = arena_autonomy.race_ops_node:main"]})"""),
])

patch('scripts/race_operator.py', [
("""        'mode', 'session', 'race_epoch', 'intervention', 'latch', 'started', 'visual_start_fact',
""", """        'mode', 'session', 'race_id', 'race_epoch', 'intervention', 'latch', 'started', 'start_source',
        'recovery_ready', 'resume', 'last_resume', 'visual_start_fact',
"""),
("""    if op not in ('stop', 'new_race', 'acknowledge'):
        raise ValueError('unsupported operation')
""", """    if op not in ('stop', 'new_race', 'acknowledge', 'recovery_reset', 'resume_start'):
        raise ValueError('unsupported operation')
"""),
("""        if intervention is not None or hazards_cleared or operator_reviewed:
            raise ValueError('new_race does not take acknowledgement flags')
    else:
""", """        if intervention is not None or hazards_cleared or operator_reviewed:
            raise ValueError('new_race does not take acknowledgement flags')
    elif op in ('recovery_reset', 'resume_start'):
        if not natural(intervention) or hazards_cleared or operator_reviewed:
            raise ValueError('explicit intervention required; acknowledgement flags are not used')
        result.update(intervention=intervention)
    else:
"""),
("""    if command['op'] == 'acknowledge':
        if command['intervention'] != status['intervention'] or status['latch'] is None:
""", """    if command['op'] in ('acknowledge', 'recovery_reset', 'resume_start'):
        if command['intervention'] != status['intervention'] or status['latch'] is None:
"""),
("""                 and result.get('latch') == 'new_race')
    else:
""", """                 and result.get('latch') == 'new_race')
    elif op in ('recovery_reset', 'resume_start'):
        # 수락은 입력 경계 생성까지다. 정지를 유지하므로 래치가 남아 있어야 한다.
        valid = (result['session'] == command['session'] and result['race_epoch'] == command['race_epoch']
                 and result['intervention'] == command['intervention'] and result.get('latch') is not None)
    else:
"""),
("""    parser.add_argument('operation', choices=('status', 'stop', 'new-race', 'acknowledge'))
""", """    parser.add_argument('operation', choices=('status', 'stop', 'new-race', 'acknowledge', 'recovery-reset', 'resume-start'))
"""),
])
