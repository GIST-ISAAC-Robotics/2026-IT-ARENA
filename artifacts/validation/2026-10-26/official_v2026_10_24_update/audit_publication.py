from pathlib import Path
import subprocess,json,re
r=Path.cwd();o=r/'artifacts/validation/2026-10-26/official_v2026_10_24_update'
def git(*args):return subprocess.check_output(['git','-c','core.quotepath=false',*args],cwd=r)
files=set(git('ls-files','--others','--exclude-standard','-z').decode().split('\0'))|set(git('diff','--name-only','-z').decode().split('\0'));files.discard('')
patterns={'private_key':rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----','github_token':rb'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{35,})','openai_token':rb'sk-(?:proj-)?[A-Za-z0-9_-]{35,}','credential_url':rb'https?://[^\s/:]+:[^\s/@]{3,}@'}
flags=[];meta=[]
for n in sorted(files):
 p=r/n
 if not p.is_file():continue
 meta.append(dict(path=n,size_bytes=p.stat().st_size))
 if p.suffix.lower() in ('.py','.json','.md','.yaml','.yml','.toml','.ps1','.sh','.txt','.log','.sdf','.xml'):
  data=p.read_bytes()
  for label,rx in patterns.items():
   if re.search(rx,data):flags.append(dict(path=n,kind=label))
result=dict(candidate_files=len(meta),candidate_bytes=sum(x['size_bytes'] for x in meta),largest=sorted(meta,key=lambda x:x['size_bytes'],reverse=True)[:15],potential_secret_paths=flags,scope='Windows Git candidate list; high confidence pattern scan; existing raw exclusions preserved')
(o/'publication_audit.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='largest'},ensure_ascii=False))
ignored=git('ls-files','--others','--ignored','--exclude-standard','-z','--','artifacts/validation/2026-10-24','artifacts/validation/2026-10-25','artifacts/validation/2026-10-26').decode().split('\0')
rows=[]
for n in ignored:
 if not n:continue
 p=r/n
 if p.is_file():rows.append(dict(path=n,size_bytes=p.stat().st_size))
(o/'local_only_inventory.json').write_text(json.dumps(dict(scope='raw inputs, detailed logs and source archives remain local; no files removed',files=rows),ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print('local_only_files',len(rows),'bytes',sum(x['size_bytes'] for x in rows))
