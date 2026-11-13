from pathlib import Path
from collections import defaultdict
import json
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent
counts=defaultdict(lambda:dict(passed=0,failed=0));failures=[]
for case in ET.parse(A/'pytest.xml').iter('testcase'):
    failure=case.find('failure')
    counts[case.get('classname')]['failed' if failure is not None else 'passed']+=1
    if failure is not None:
        failures.append(dict(test=case.get('classname')+'::'+case.get('name'),
                             message=failure.get('message')))
result=dict(counts=dict(counts),failures=failures,
    all_failures_same_marker_fixture=all(row['message']==
        "AttributeError: 'types.SimpleNamespace' object has no attribute 'vehicle_marker_channel'"
        for row in failures))
(A/'failure_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(dict(counts)))
