"""디스플레이를 종료하지 않고 10초 유휴 비용만 측정한다."""
from pathlib import Path
import json,os,re,subprocess,time

def snapshot():
    processes={}
    for p in Path('/proc').iterdir():
        if not p.name.isdigit():continue
        try:
            name=(p/'comm').read_text().strip()
            if not (name in ('Xorg','gnome-shell','gdm3','gdm-session-wor','gdm-x-session') or name.startswith('nx')):continue
            stat=(p/'stat').read_text().rsplit(')',1)[1].split()
            roll=(p/'smaps_rollup').read_text()
            rss=int(re.search(r'^Rss:\s+(\d+)',roll,re.M)[1])
            pss=int(re.search(r'^Pss:\s+(\d+)',roll,re.M)[1])
            processes[p.name]=dict(name=name,ticks=int(stat[11])+int(stat[12]),rss_kib=rss,pss_kib=pss)
        except (OSError,ValueError,IndexError,TypeError):continue
    return processes

start=time.monotonic();before=snapshot();time.sleep(10);after=snapshot();elapsed=time.monotonic()-start
rows=[]
for pid,state in after.items():
    if pid not in before:continue
    row=dict(pid=int(pid),**state)
    row['cpu_pct_one_core']=100*(state['ticks']-before[pid]['ticks'])/os.sysconf('SC_CLK_TCK')/elapsed
    row.pop('ticks');rows.append(row)
try:
    raw=subprocess.check_output(['timeout','6','tegrastats','--interval','1000'],text=True,stderr=subprocess.STDOUT)
except subprocess.CalledProcessError as e:raw=e.output
except OSError as e:raw=str(e)
connections=subprocess.check_output(['ss','-Htn','state','established'],text=True)
nx_count=sum(1 for line in connections.splitlines() if re.search(r':4000\s',line))
print(json.dumps(dict(scope='idle snapshot; no services stopped; no driving or benchmark',sample_s=elapsed,
    cpu_pct_is_percent_of_one_core=True,processes=rows,
    gui_cpu_pct_one_core=sum(r['cpu_pct_one_core'] for r in rows),
    gui_pss_kib=sum(r['pss_kib'] for r in rows),
    tcp_4000_established=nx_count,meminfo=Path('/proc/meminfo').read_text().splitlines()[:5],
    tegrastats=raw.splitlines()),indent=2))
