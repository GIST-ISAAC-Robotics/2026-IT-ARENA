"""r530 월드의 외부 자원을 원본 월드 기준으로 해석하고 경로만 교체한다."""
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
OLD=A.parent/'field_wall_drive_r520_v1'
ORIGINAL=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/world.sdf'
INPUT=OLD/'field_world.sdf';OUTPUT=A/'field_world.sdf'
INPUT_SHA='9bf5ba7491153b8ef045a96e2428f44bffc9d308f4d3431c25bee1f30ee166a2'
ORIGINAL_SHA='4c2136c7eec1615293db9f0cd8a5acb5133fc5f433c4a16f292b527bca851b3b'
RESOURCE_TAGS={'uri','albedo_map','diffuse_map','normal_map','roughness_map','metalness_map','environment_map',
    'emissive_map','light_map','specular_map','glossiness_map','texture','texture_file','filename','vertex_shader','fragment_shader'}
FILE_PATTERN=re.compile(r'\.(?:png|jpe?g|bmp|tga|dds|tiff?|webp|stl|dae|obj|mtl|mesh|material|glsl|vert|frag|sdf|urdf)(?:$|[?#])',re.I)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())


def canonical(e):return ET.canonicalize(ET.tostring(e,encoding='unicode'),strip_text=True)


def inventory(root):
    files=[];plugins=[];unknown=[]
    def walk(e,path):
        label=e.tag+("[@name='"+e.get('name')+"']" if e.get('name') else '')
        here=path+'/'+label;text=(e.text or '').strip()
        if text:
            if e.tag in RESOURCE_TAGS:files.append((here,e,None,text))
            elif FILE_PATTERN.search(text) or '://' in text:unknown.append((here,text))
        for key,value in e.attrib.items():
            if key=='filename' and e.tag=='plugin' and not any(t in value for t in ('/','\\',':')):
                plugins.append(dict(path=here+'/@filename',value=value,kind='package_symbol'))
            elif key in ('filename','uri','texture','file'):
                files.append((here+'/@'+key,e,key,value))
            elif FILE_PATTERN.search(value) or '://' in value:unknown.append((here+'/@'+key,value))
        for child in e:walk(child,here)
    walk(root,'')
    assert not unknown,unknown
    return files,plugins


def resolve(value,base,repository_only=True):
    if value.startswith('file://'):value=value[7:]
    elif '://' in value:raise ValueError('Unresolved non-file resource scheme: '+value)
    if os.name=='nt' and re.match(r'^/mnt/[a-zA-Z]/',value):value=value[5]+':/'+value[7:]
    path=Path(value)
    if not path.is_absolute():path=base/path
    path=path.resolve(strict=True)
    assert path.is_file(),path
    if repository_only:path.relative_to(ROOT)
    return path


def geometry_digest(root,tag):
    return hashlib.sha256('\n'.join(canonical(e) for e in root.iter(tag)).encode()).hexdigest()


def create():
    assert sha(INPUT)==INPUT_SHA and sha(ORIGINAL)==ORIGINAL_SHA
    assert not OUTPUT.exists() and not (A/'resource_manifest.json').exists()
    old=ET.parse(INPUT).getroot();new=copy.deepcopy(old);original=ET.parse(ORIGINAL).getroot()
    before,plugins=inventory(old);after,_=inventory(new);original_refs,_=inventory(original)
    assert [(p,t) for p,_,_,t in before]==[(p,t) for p,_,_,t in original_refs]
    changes=[]
    for (path,_,_,value),(newpath,node,attribute,newvalue) in zip(before,after):
        assert path==newpath and value==newvalue
        target=resolve(value,ORIGINAL.parent)
        rewritten=os.path.relpath(target,A).replace('\\','/')
        if attribute:node.set(attribute,rewritten)
        else:node.text=rewritten
        assert resolve(rewritten,A)==target
        changes.append(dict(xml_path=path,before=value,after=rewritten,
            resolved_repository_path=target.relative_to(ROOT).as_posix(),sha256=sha(target),size_bytes=target.stat().st_size))
    assert len(changes)==4 and all(c['xml_path'].endswith('/albedo_map') for c in changes)
    restored=copy.deepcopy(new)
    for item,(_,node,attribute,_) in zip(changes,inventory(restored)[0]):
        if attribute:node.set(attribute,item['before'])
        else:node.text=item['before']
    assert canonical(restored)==canonical(old),'non-resource content changed'
    for tag in ('geometry','collision','pose','surface','physics'):
        assert geometry_digest(new,tag)==geometry_digest(old,tag),tag
    # XML 직렬화 차이도 만들지 않는다. 원문에서 확인된 네 텍스트만 바꾼다.
    payload=INPUT.read_bytes()
    for item in changes:
        before_text=('<albedo_map>'+item['before']+'</albedo_map>').encode()
        after_text=('<albedo_map>'+item['after']+'</albedo_map>').encode()
        assert payload.count(before_text)==1
        payload=payload.replace(before_text,after_text)
    assert canonical(ET.fromstring(payload))==canonical(new)
    with OUTPUT.open('xb') as f:f.write(payload)
    manifest=dict(source_original_world=ORIGINAL.relative_to(ROOT).as_posix(),source_original_world_sha256=ORIGINAL_SHA,
        preserved_r520_world=INPUT.relative_to(ROOT).as_posix(),preserved_r520_world_sha256=INPUT_SHA,
        output_world=OUTPUT.relative_to(ROOT).as_posix(),output_world_sha256=sha(OUTPUT),
        root_basis='Resource filenames in r520 are interpreted relative to the original official derivative world directory',
        changes=changes,changed_resource_fields=4,path_fields_only=True,byte_change_only_four_resource_texts=True,
        geometry_digests={tag:geometry_digest(new,tag) for tag in ('geometry','collision','pose','surface','physics')},
        plugins=plugins,package_plugins_unchanged=True,
        plugin_loader_check_deferred='Symbolic Gazebo package libraries are not filesystem asset paths; WSL remains root-owned and was not accessed',
        all_world_asset_files_exist=True,originals_unchanged=sha(INPUT)==INPUT_SHA and sha(ORIGINAL)==ORIGINAL_SHA,
        WSL_used=False,frozen=False,simulator_started=False)
    with (A/'resource_manifest.json').open('x',encoding='utf-8') as f:json.dump(manifest,f,ensure_ascii=False,indent=2);f.write('\n')
    print(json.dumps(dict(resource_fields=4,unique_world_files=4,output_world_sha256=manifest['output_world_sha256'],path_only=True)))


def verify_resources():
    info=read(A/'resource_manifest.json')
    assert sha(INPUT)==info['preserved_r520_world_sha256']==INPUT_SHA
    assert sha(ORIGINAL)==info['source_original_world_sha256']==ORIGINAL_SHA
    assert sha(OUTPUT)==info['output_world_sha256']
    files,plugins=inventory(ET.parse(OUTPUT).getroot())
    assert len(files)==len(info['changes']) and plugins==info['plugins']
    checked=[]
    for (path,_,_,text),item in zip(files,info['changes']):
        assert path==item['xml_path'] and text==item['after']
        local=resolve(text,OUTPUT.parent)
        assert local.relative_to(ROOT).as_posix()==item['resolved_repository_path'] and sha(local)==item['sha256']
        checked.append(dict(source='field_world.sdf',xml_path=path,reference=text,repository_path=local.relative_to(ROOT).as_posix(),sha256=sha(local)))
    facility=A/'facilities_only.sdf'
    assert sha(facility)==sha(OLD/'facilities_only.sdf')
    refs,facility_plugins=inventory(ET.parse(facility).getroot())
    for path,_,_,value in refs:
        local=resolve(value,facility.parent)
        checked.append(dict(source='facilities_only.sdf',xml_path=path,reference=value,repository_path=local.relative_to(ROOT).as_posix(),sha256=sha(local)))
    assert len(refs)==6
    return dict(passed=True,asset_reference_count=len(checked),unique_asset_file_count=len({r['repository_path'] for r in checked}),
        asset_references=checked,world_plugin_symbols=plugins,facility_plugin_symbols=facility_plugins,
        package_plugin_loader_tested=False,WSL_used=False,
        scope='All declared file resources in field world and unchanged facilities:4 albedo maps and6 mesh references; package plugin symbols separately classified')


def verify_runtime_resources(world):
    """후속 승인된 실행에서 common 변환 직후 파일 자원을 확인한다. 현재는 호출하지 않는다."""
    world=Path(world);refs,plugins=inventory(ET.parse(world).getroot());checked=[]
    for path,_,_,value in refs:
        target=resolve(value,world.parent,repository_only=False)
        checked.append(dict(xml_path=path,reference=value,resolved_path=str(target),sha256=sha(target)))
        if target.suffix.lower()=='.obj':
            # 현재 common 배치 OBJ는 외부 MTL을 만들지 않는다. 다른 입력이면 누락시키지 않는다.
            assert not any(line.lstrip().startswith('mtllib ') for line in target.read_text(encoding='utf-8').splitlines()),target
    return dict(passed=True,asset_reference_count=len(checked),asset_references=checked,
        package_plugin_symbols=plugins,package_plugin_loader_tested=False,
        scope='Runtime SDF declared assets after common conversion, before Gazebo starts')


if __name__=='__main__':create()
