"""중단 시 stage/문서를 보존한 뒤 v2 작업을 시작한다. 원 ROS run1/아카이브 불변."""
import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parent;revision=root/'revision_v2'
revision.mkdir(exist_ok=False)
shutil.copytree(root/'stage',revision/'stage_before')
for name in ('changes.json','stage.diff','result.json','source_integrity.json','PAUSED.md','README.md','RESULT.md'):
    shutil.copyfile(root/name,revision/('before_'+name))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
(revision/'preserved.json').write_text(json.dumps({name:sha(root/name) for name in
    ('stage_source.zip','execution_v1/source.zip','execution_v1/run1/result.json','execution_v1/pause_pid_audit.json')},indent=2)+'\n')
print('revision_v2 backup prepared; v1 archives/run1 unchanged')
