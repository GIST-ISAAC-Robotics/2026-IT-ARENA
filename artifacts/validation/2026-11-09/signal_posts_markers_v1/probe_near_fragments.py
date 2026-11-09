"""단일면 사본과 같은 원시 C 입력에 독립 근접 조각 사본을 비교한다."""
from pathlib import Path
import sys
from near_fragments_candidate import NearbyFragments
import near_face_candidate
near_face_candidate.NearbyFace=NearbyFragments
A=Path(__file__).resolve().parent
s=(A/'probe_near_face.py').read_text().replace('near_face_rows.json','near_fragments_rows.json').replace('near_face_summary.json','near_fragments_summary.json')
exec(compile(s,str(A/'probe_near_face.py'),'exec'))
