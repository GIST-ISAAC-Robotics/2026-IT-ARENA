#!/usr/bin/env python3
"""Windows steering setup window; a single SSH channel lives with the window."""
import json
from pathlib import Path
import queue
import shlex
import socket
import sys
import threading
import tkinter as tk
from tkinter import ttk

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'build/jetson_tools'))
import paramiko

STAGES = ('straight', 'left_max', 'right_max')
LABELS = ('중앙', '왼쪽 최대', '오른쪽 최대')


class Link:
    def __init__(self, settings, requests, results):
        self.settings, self.requests, self.results = settings, requests, results
        self.client = None

    def run(self):
        cfg = self.settings
        client = paramiko.SSHClient()
        self.client = client
        sock = None
        try:
            client.load_host_keys(cfg['known_hosts'])
            client.set_missing_host_key_policy(paramiko.RejectPolicy())
            sock = socket.create_connection((cfg['connect_address'], cfg['port']), timeout=8)
            client.connect(cfg['host'], cfg['port'], cfg['user'], cfg['password'], sock=sock,
                           look_for_keys=False, allow_agent=False, auth_timeout=8, banner_timeout=8)
            client.get_transport().set_keepalive(15)
            cmd = 'sudo -S -p ' + shlex.quote('') + ' -- bash -lc ' + shlex.quote(cfg['command'])
            stdin, stdout, _ = client.exec_command(cmd, timeout=12)
            stdin.write(cfg.pop('password') + '\n')
            stdin.flush()
            if json.loads(stdout.readline()) != {'ready': True}:
                raise RuntimeError('Jetson 조향 프로그램 시작 실패')
            self.results.put(('ready', {}))
            while (request := self.requests.get()) is not None:
                stdin.write(json.dumps(request) + '\n')
                stdin.flush()
                response = json.loads(stdout.readline())
                self.results.put((request['action'], response))
            stdin.close()
        except Exception as exc:
            self.results.put(('offline', {'error': f'{type(exc).__name__}: {exc}'}))
        finally:
            client.close()
            if sock is not None:
                sock.close()


class Panel:
    def __init__(self, root, settings):
        self.root = root
        self.requests, self.results = queue.Queue(), queue.Queue()
        self.link = Link(settings, self.requests, self.results)
        self.online = self.busy = self.checked = False
        self.points = {}
        self.safe = tk.BooleanVar()
        self.ends = tk.BooleanVar()
        self.step_size = tk.StringVar(value='12')
        self.status = tk.StringVar(value='Jetson 연결 중…')
        self.pos = tk.StringVar(value='—')
        self.path = tk.StringVar()
        root.title('IT ARENA · 조향 교정')
        root.minsize(590, 480)
        root.protocol('WM_DELETE_WINDOW', self.close)
        frame = ttk.Frame(root, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='조향 교정', font=('맑은 고딕', 19, 'bold')).pack(anchor='w')
        ttk.Label(frame, textvariable=self.status, wraplength=540, padding=(0, 10)).pack(anchor='w')
        ttk.Checkbutton(frame, text='모터 전원 분리 · 앞바퀴 띄움 · 손과 도구를 링크에서 뗌',
                        variable=self.safe, command=self.refresh).pack(anchor='w')
        row = ttk.Frame(frame, padding=(0, 9))
        row.pack(fill='x')
        self.read = ttk.Button(row, text='서보 연결 확인', command=lambda: self.send('read'))
        self.read.pack(side='left')
        self.release = ttk.Button(row, text='토크 해제', command=lambda: self.send('release'))
        self.release.pack(side='left', padx=5)
        ttk.Label(row, textvariable=self.pos, font=('맑은 고딕', 12, 'bold')).pack(side='right')
        sizes = ttk.Frame(frame, padding=(0, 4))
        sizes.pack(fill='x')
        ttk.Label(sizes, text='한 번에 이동:').pack(side='left')
        self.size_box = ttk.Combobox(sizes, textvariable=self.step_size, values=('12', '24', '36', '48'), width=5,
                                    state='readonly')
        self.size_box.pack(side='left', padx=6)
        self.size_box.bind('<<ComboboxSelected>>', lambda _: self.refresh())
        self.step_degrees = ttk.Label(sizes)
        self.step_degrees.pack(side='left')
        row = ttk.Frame(frame, padding=(0, 3))
        row.pack(fill='x')
        self.steps = []
        for sign in (-1, 1):
            button = ttk.Button(row, command=lambda s=sign: self.send(
                'step', delta=s*int(self.step_size.get()), find_ends=self.ends.get()))
            button.pack(side='left', fill='x', expand=True, padx=3, ipady=8)
            self.steps.append(button)
        self.end_box = ttk.Checkbutton(frame, text='양쪽 끝 탐색 허용 (중앙 저장 후, 한 칸씩 관찰)', variable=self.ends)
        self.end_box.pack(anchor='w', pady=4)
        angles = ttk.Frame(frame, padding=(0, 4))
        angles.pack(fill='x')
        ttk.Label(angles, text='끝점 바퀴각(도, 선택):').pack(side='left')
        self.angles = []
        for side in ('왼쪽 바퀴', '오른쪽 바퀴'):
            ttk.Label(angles, text=side, padding=(6, 0)).pack(side='left')
            entry = ttk.Entry(angles, width=7)
            entry.pack(side='left')
            self.angles.append(entry)
        grid = ttk.Frame(frame, padding=(0, 8))
        grid.pack(fill='x')
        self.saves = []
        self.values = []
        for i, (key, label) in enumerate(zip(STAGES, LABELS)):
            grid.columnconfigure(i, weight=1)
            btn = ttk.Button(grid, text=f'{i+1}. {label} 저장', command=lambda k=key: self.capture(k))
            btn.grid(row=0, column=i, sticky='ew', padx=3)
            value = tk.StringVar(value='—')
            ttk.Label(grid, textvariable=value, anchor='center').grid(row=1, column=i, sticky='ew', pady=6)
            self.saves.append(btn)
            self.values.append(value)
        ttk.Label(frame, text='숫자의 +/−와 실제 좌우는 다를 수 있습니다. 링크가 닿기 전에 멈추세요.',
                  wraplength=540).pack(anchor='w')
        ttk.Label(frame, textvariable=self.path, wraplength=540, padding=(0, 8)).pack(anchor='w')
        self.refresh()
        threading.Thread(target=self.link.run, daemon=True).start()
        root.after(80, self.poll)

    def send(self, action, **data):
        if self.busy or not self.online:
            return
        self.busy = True
        self.status.set('서보 확인 중…' if action != 'step' else f'{data["delta"]:+d} counts 이동 중…')
        self.requests.put(dict(action=action, **data))
        self.refresh()

    def capture(self, stage):
        angles = None
        raw = [entry.get().strip() for entry in self.angles]
        if stage != 'straight' and any(raw):
            try:
                angles = [float(value) for value in raw]
            except ValueError:
                self.status.set('각도계로 잰 두 바퀴각을 모두 적거나 두 칸을 비우세요.')
                return
        self.send('capture', stage=stage, angles=angles)

    def refresh(self):
        free = self.online and not self.busy
        active = free and self.checked and self.safe.get()
        self.read.configure(state='normal' if free else 'disabled')
        self.release.configure(state='normal' if free else 'disabled')
        self.end_box.configure(state='normal' if active and 'straight' in self.points else 'disabled')
        self.size_box.configure(state='readonly' if active else 'disabled')
        size = int(self.step_size.get())
        self.step_degrees.configure(text=f'counts (서보축 약 {size*360/4096:.2f}°)')
        for sign, button in zip((-1, 1), self.steps):
            button.configure(text=f'{sign*size:+d} counts 이동', state='normal' if active else 'disabled')
        for key, button, value in zip(STAGES, self.saves, self.values):
            point = self.points.get(key)
            value.set(f'{point["shaft_count"]} counts' if point else '—')
            next_key = next((s for s in STAGES if s not in self.points), None)
            button.configure(state='normal' if active and key == next_key else 'disabled')

    def poll(self):
        while not self.results.empty():
            kind, data = self.results.get_nowait()
            self.busy = False
            if kind == 'ready':
                self.online = True
                self.status.set('Jetson 연결됨. 서보 전원을 연결한 후 ‘서보 연결 확인’을 누르세요.')
            elif kind == 'offline':
                self.online = self.checked = False
                self.status.set('연결 종료. 필요하면 서보 전원을 끄고 창을 다시 여세요. ' + data['error'])
            elif not data['ok']:
                if data.get('check_again'):
                    self.checked = False
                self.status.set(data['error'] + (' · 서보 연결을 다시 확인하세요.' if not self.checked else ''))
            else:
                self.checked = kind != 'release'
                if 'position' in data:
                    self.pos.set(f'{data["position"]} counts')
                if 'points' in data:
                    self.points = data['points']
                if 'folder' in data:
                    self.path.set('Jetson 기록: ' + data['folder'])
                self.status.set('토크 OFF 확인' if kind != 'capture' else '위치 저장 · 토크 OFF 확인')
                if kind == 'step':
                    seen = data.get('position_before_release', data.get('last_position_before_release'))
                    outcome = '도달' if data['step_reached'] else '미도달'
                    self.status.set(f'목표 {data["target"]} · 실제 {seen} counts ({seen-data["start"]:+d})'
                                    f' · {outcome} · 토크 OFF')
                if kind == 'capture':
                    for entry in self.angles:
                        entry.delete(0, 'end')
                    if 'right_max' in self.points:
                        self.status.set('세 지점 저장 완료 · 토크 OFF 확인')
                if kind == 'release':
                    self.status.set('토크 OFF 확인. 다시 움직이려면 서보 연결을 확인하세요.')
            self.refresh()
        self.root.after(80, self.poll)

    def close(self):
        self.requests.put(None)
        if self.link.client is not None:
            self.link.client.close()
        self.root.destroy()


if __name__ == '__main__':
    settings = json.load(sys.stdin)
    root = tk.Tk()
    Panel(root, settings)
    root.mainloop()
