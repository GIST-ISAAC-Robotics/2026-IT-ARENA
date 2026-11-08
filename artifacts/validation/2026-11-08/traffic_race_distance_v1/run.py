"""공용 제어기는 그대로, 명시 평가 프로세스의1/5랩 경로 검사만 연결한다."""
import json,hashlib,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
def main(case):
    f=HERE/case;r=json.loads((f/'registered.json').read_bytes())
    assert not (f/'run1').exists()
    assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in r['sources'].items())
    import g1_course_eval
    from course_geometry import check_full_course
    g1_course_eval.check_full_course=check_full_course
    import validate_g1_traffic
    args=[str(f/'run1') if value=='OUTPUT' else value for value in r['args']]
    sys.argv=args
    return validate_g1_traffic.main()
if __name__=='__main__':raise SystemExit(main(sys.argv[1]))
