"""C3: 같은 C2 시설/상대/속도에서 .8초 근접 면 관측 창만 대조한다."""
from pathlib import Path
A=Path(__file__).resolve().parent
for ext in ('py','sh'):
    out=A/f'c3_run.{ext}';assert not out.exists()
    s=(A/f'c2_run.{ext}').read_text(encoding='utf8')
    s=s.replace('run2','run3').replace('c2_run.py','c3_run.py')
    out.write_text(s,encoding='utf8',newline='\n')
print('C3 scripts prepared; not executed')
