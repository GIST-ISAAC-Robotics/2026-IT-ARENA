"""저장 대조용: 만료 결과의 기하 참조와 선택 벽만 현재 관측 비교에 남긴다.

운영 연결 없음. 원 method의 최근 참조 분기에만 벽 변경 시 전방 호 일치를
추가한다. 해당 참조가 없으면 원 함수와 같다.
"""
import inspect
import textwrap
import arena_autonomy.local_follow as local


def install():
    original=local.LocalFollow._baseline_command
    source=textwrap.dedent(inspect.getsource(original))
    anchor="                if (recent is not None and eligible and sufficient(candidate, info)\n"
    addition="""            reference_wall = reference[3] if reference is not None and len(reference) == 4 else None
            if (recent is not None and reference_wall is not None
                    and candidate_wall != reference_wall
                    and not path_agrees_arc(candidate, recent, cfg.boundary_agreement_m)):
                eligible = False
                detail['expiry_route_switch_rejected'] = 'forward_path_diverges'
"""
    assert source.count(anchor)==1
    source=source.replace(anchor,textwrap.indent(addition,'    ')+anchor)
    old="                        if sufficient(path, details) and path_agrees(path, recent, cfg.boundary_agreement_m):\n"
    new="""                    if (sufficient(path, details) and path_agrees(path, recent, cfg.boundary_agreement_m)
                            and (reference_wall is None or wall_side == reference_wall
                                 or path_agrees_arc(path, recent, cfg.boundary_agreement_m))):
"""
    assert source.count(old)==1
    source=source.replace(old,textwrap.indent(new,'    '))
    source=source.replace('(self.path, self.path_ref, self.path_source)',
                          '(self.path, self.path_ref, self.path_source, self.path_wall)')
    namespace=dict(vars(local))
    exec(compile(source,str(__file__)+':stored_candidate','exec'),namespace)
    local.LocalFollow._baseline_command=namespace['_baseline_command']
    return source


def reset_reference(owner):
    normal=owner.normal
    reference=normal.recent_path
    if normal.path is not None and normal.path_source is not None:
        if reference is None or normal.path_source>=reference[2]:
            reference=(normal.path.copy(),normal.path_ref,normal.path_source,normal.path_wall)
    metadata={key:getattr(normal,key) for key in ('epoch','side','last_ref','last_source')}
    owner.reset()
    if reference is not None and len(reference)==4 and reference[3] in ('left','right'):
        normal.recent_path=reference
        for key,value in metadata.items():setattr(normal,key,value)
    assert normal.path is None and normal.held and normal.path_wall is None
    return normal.recent_path is not None
