"""파일 표본만 읽는 간이 조정 CLI. 장치 접근/기존 profile 덮어쓰기 없음."""
import argparse
import json
from pathlib import Path
from arena_autonomy.vehicle_marker_calibration import adjust_profile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile',type=Path,required=True)
    parser.add_argument('--samples',type=Path,required=True)
    parser.add_argument('--method',choices=('focal_pixels','range_scale'),default='focal_pixels')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    result=adjust_profile(json.loads(args.profile.read_text(encoding='utf-8')),
        json.loads(args.samples.read_text(encoding='utf-8')),args.method)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,ensure_ascii=False,indent=2);stream.write('\n')


if __name__=='__main__':main()
