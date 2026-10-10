#!/usr/bin/env python3
"""동결 시점 README 해시가 현재 README의 줄 경계 앞부분과 일치하는지(뒤에 추가만 했는지) 확인한다."""
import hashlib
from pathlib import Path

base = Path(__file__).resolve().parent
data = (base/'README.md').read_bytes()
cuts = [i+1 for i, b in enumerate(data) if b == 0x0A]
prefix = {hashlib.sha256(data[:c]).hexdigest(): c for c in cuts}
prefix[hashlib.sha256(data).hexdigest()] = len(data)
for name in ('pre_run_freeze.sha256', 'addendum_v2_freeze.sha256', 'addendum_c3_freeze.sha256'):
    line = next(l for l in (base/name).read_text().splitlines() if l.endswith('/README.md'))
    value = line.split()[0]
    print(name, 'README prefix bytes' if value in prefix else 'NOT A PREFIX', prefix.get(value), 'of', len(data))
