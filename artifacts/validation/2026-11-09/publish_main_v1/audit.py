"""스테이징된 게시 파일의 크기·비밀값 패턴·등록 바이트를 검사한다. 파일 내용을 출력하지 않는다."""
from pathlib import Path
import hashlib
import io
import json
import re
import subprocess
import zipfile

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT)
def paths(*args):
    return [s.decode('utf8') for s in git(*args).split(b'\0') if s]

def main():
    changed = paths('diff', '--cached', '--name-only', '--diff-filter=ACMRT', '-z')
    # 실제 비밀값 형태만 찾는다. 일치한 값은 결과/표준 출력에 넣지 않는다.
    rules = [rb'-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----',
             rb'gh[pousr]_[A-Za-z0-9]{30,}', rb'github_pat_[A-Za-z0-9_]{60,}',
             rb'sk-(?:proj-|svcacct-)?[A-Za-z0-9_-]{40,}',
             rb'AKIA[0-9A-Z]{16}']
    findings, normalizations, sizes, bad_archives = [], [], [], []
    batch = subprocess.Popen(['git', 'cat-file', '--batch'], cwd=ROOT,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    def inspect(name, data):
        if any(re.search(rule, data) for rule in rules):
            findings.append(name)
    for name in changed:
        batch.stdin.write((':' + name + '\n').encode('utf8')); batch.stdin.flush()
        hdr = batch.stdout.readline().split()
        if len(hdr) != 3:
            raise RuntimeError('missing staged blob: ' + name)
        count = int(hdr[2]); data = batch.stdout.read(count)
        assert batch.stdout.read(1) == b'\n'
        sizes.append({'path': name, 'bytes': count})
        local = ROOT / name
        if name.startswith(('src/', 'scripts/', 'config/', 'tests/', 'artifacts/validation/')):
            if local.read_bytes() != data:
                normalizations.append(name)
        inspect(name, data)
        if name.endswith('.zip'):
            try:
                with zipfile.ZipFile(io.BytesIO(data)) as z:
                    for member in z.infolist():
                        if not member.is_dir() and member.file_size < 20_000_000:
                            inspect(name + '::' + member.filename, z.read(member))
            except zipfile.BadZipFile:
                bad_archives.append(name)
    batch.stdin.close(); batch.wait()
    result = dict(staged_files=len(changed), staged_bytes=sum(s['bytes'] for s in sizes),
                  largest_files=sorted(sizes, key=lambda s: s['bytes'], reverse=True)[:10],
                  too_large=[s for s in sizes if s['bytes'] >= 100*1024*1024],
                  secret_pattern_paths=findings, byte_normalization_paths=normalizations,
                  bad_archives=bad_archives)
    (A/'staged_audit.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k!='largest_files'}, ensure_ascii=False))

if __name__ == '__main__': main()
