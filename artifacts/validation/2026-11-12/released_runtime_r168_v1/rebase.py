import importlib.util
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
def load(name,path):
 s=importlib.util.spec_from_file_location(name,path);m=importlib.util.module_from_spec(s);s.loader.exec_module(m);return m
previous=load('r168_previous',ROOT/'artifacts/validation/2026-11-12/adjacent_live_r152_v1/rebase.py')
released=load('r168_released',ROOT/'artifacts/validation/2026-11-12/released_current_r167_v1/released.py')
def supported(output):return previous.supported(output) or released.supported(output)
def rebase(output,submitted,current,scene,history,cfg):
 if previous.supported(output):return previous.rebase(output,submitted,current,scene,history,cfg)
 return released.rebase(output,submitted,current,scene,history,cfg)
