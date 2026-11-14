"""계산 결과 만료 뒤 새 관측과 비교할 최근 경로만 보존하는 명시 선택 정책."""


def reset_after_source_expiry(owner):
    normal = getattr(owner, 'normal', None)
    if normal is None or not normal.config.expiry_route_reference_trial:
        owner.reset()
        return False
    reference = normal.recent_path
    if normal.path is not None and normal.path_source is not None:
        if reference is None or normal.path_source >= reference[2]:
            reference = (normal.path.copy(), normal.path_ref, normal.path_source, normal.path_wall)
    metadata = {key: getattr(normal, key) for key in ('epoch', 'side', 'last_ref', 'last_source')}
    owner.reset()
    if reference is not None and len(reference) == 4 and reference[3] in ('left', 'right'):
        normal.recent_path = reference
        for key, value in metadata.items():
            setattr(normal, key, value)
    # 활성 경로·조향 명령·우회/추월 상태는 돌아오지 않는다.
    assert normal.path is None and normal.held and normal.path_wall is None
    return normal.recent_path is not None
