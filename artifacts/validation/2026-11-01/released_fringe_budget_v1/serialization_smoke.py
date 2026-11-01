"""재생 없이 NumPy 집계·개별 결과 보존·기존 결과 덮어쓰기 거부만 검사한다."""
from collections import Counter
import json
from pathlib import Path
import numpy as np
from compare import write_json

HERE=Path(__file__).resolve().parent
folder=HERE/'serialization_smoke';folder.mkdir(exist_ok=False)
counts=Counter(new_zero=np.int64(0),matched=np.int64(17))
path=folder/'checkpoint.json'
write_json(path,dict(counts={k:int(v) for k,v in counts.items()},
                     detail=dict(raw=np.int64(9),flag=np.bool_(True),value=np.float64(.35))))
saved=json.loads(path.read_text())
assert saved['counts']==dict(new_zero=0,matched=17)
assert type(saved['counts']['matched']) is int and saved['detail']['flag'] is True
assert saved['detail']['raw']==9 and saved['detail']['value']==.35
before=path.read_bytes()
try:write_json(path,dict(changed=True))
except FileExistsError:pass
else:raise AssertionError('existing result overwritten')
assert path.read_bytes()==before
print('NUMPY_JSON_CHECKPOINT_AND_NO_OVERWRITE_OK')
