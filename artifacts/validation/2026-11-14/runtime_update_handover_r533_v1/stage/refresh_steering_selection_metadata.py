"""선택 프로필의 설명만 바뀐 경우 새 SHA 선택본 작성. 원선택/수치/ZIP/장치는 변경하지 않는다."""
import argparse
import copy
import hashlib
import json
from pathlib import Path

import steering_launch_selection as launch


def operational_value(profile):
    value=copy.deepcopy(profile)
    value.pop('evidence_note',None)
    if isinstance(value.get('selection_basis'),dict):value['selection_basis'].pop('flag_meanings',None)
    return value


def refresh(selection_path,previous_profile,output):
    selection=launch.installed.read_json(selection_path)
    previous_raw=Path(previous_profile).read_bytes()
    if hashlib.sha256(previous_raw).hexdigest()!=selection['steering_profile']['sha256']:
        raise ValueError('previous profile does not match preserved selection SHA')
    name=selection['steering_profile']['path'];rel=launch.PurePosixPath(name)
    if (not name.startswith('config/hardware/') or '..' in rel.parts or '\\' in name or ':' in name
            or rel.is_absolute() or (launch.ROOT/name).is_symlink()):
        raise ValueError('invalid selected steering source path')
    current_raw=(launch.ROOT/name).read_bytes()
    if operational_value(json.loads(previous_raw))!=operational_value(json.loads(current_raw)):
        raise ValueError('operating values or measured provenance changed; root must select a new contract')
    selected=copy.deepcopy(selection)
    selected['steering_profile']['sha256']=hashlib.sha256(current_raw).hexdigest()
    # 전체 선택 검사는 기록하기 전 수행한다. vehicle SHA/L은 자동 갱신하지 않는다.
    import tempfile
    with tempfile.TemporaryDirectory(prefix='steering-metadata-') as temp:
        candidate=Path(temp)/'selection.json'
        candidate.write_text(json.dumps(selected),encoding='utf-8')
        launch.selected_arguments(candidate)
    with Path(output).open('x',encoding='utf-8') as stream:
        stream.write(json.dumps(selected,ensure_ascii=False,indent=2)+'\n')
    return dict(metadata_only=True,selection_file=str(output),previous_sha256=selection['steering_profile']['sha256'],
                current_sha256=selected['steering_profile']['sha256'],archive_created=False,device_io=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--steering-selection',type=Path,required=True)
    parser.add_argument('--previous-profile',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    try:
        result=refresh(args.steering_selection,args.previous_profile,args.output)
        print(json.dumps(result));return 0
    except (ValueError,OSError,KeyError,TypeError) as error:
        print(json.dumps(dict(error=str(error),device_io=False)));return 2


if __name__=='__main__':raise SystemExit(main())
