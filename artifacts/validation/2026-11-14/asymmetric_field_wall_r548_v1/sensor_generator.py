def sensor(now=.7, shift=0., end=2., curved=False, obstacle=False, closed=False, actor=None, amplitude=0., gap=0.):
    # 임의 시험 벽과 actor는 교차거리 생성에만 사용하고 제어기에는 label을 전달하지 않는다.
    joints = [.40, 1.00]
    intervals = [(j-gap/2,j+gap/2) for j in joints] if gap else []
    xs = np.unique(np.r_[np.linspace(-2.,end,801),[x for pair in intervals for x in pair]])
    xs = xs[(xs>=-2.) & (xs<=end)]
    # 좌벽만100mm 안쪽, 좌/우 사인 위상0/pi/2. 모두 미측정 민감도 가정.
    walls_y = {1.: CFG.wall_offset_m-.10+amplitude*np.sin(math.tau*xs),
              -1.: -CFG.wall_offset_m+amplitude*np.sin(math.tau*xs+math.pi/2)}
    segments = []
    for sign in (-1.,1.):
        points = np.c_[xs,walls_y[sign]]
        for a,b in zip(points[:-1],points[1:]):
            middle=(a[0]+b[0])/2
            if not any(lo<middle<hi for lo,hi in intervals):
                segments.append((a,b))
    if obstacle:
        segments.append((np.array([.18, -.22]), np.array([.18, .22])))
    if actor is not None:
        corners = np.array([[actor-.1, -.06], [actor+.1, -.06], [actor+.1, .06],
                            [actor-.1, .06], [actor-.1, -.06]])
        segments.extend(zip(corners[:-1], corners[1:]))
    if closed:
        for index in (0, -1):
            segments.append((np.array([xs[index], walls_y[-1.][index]]),
                             np.array([xs[index], walls_y[1.][index]])))
    origin = np.array([.06, 0.])
    starts = np.array([s[0] for s in segments])
    deltas = np.array([s[1]-s[0] for s in segments])
    ranges = []
    for angle in -math.pi+np.arange(500)*math.tau/500:
        direction = np.array([math.cos(angle), math.sin(angle)])
        denom = determinant(direction, deltas)
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = determinant(starts-origin, deltas)/denom
            proportion = determinant(starts-origin, direction)/denom
        valid = ((distance >= .05) & (distance <= 12.) & (proportion >= 0.)
                 & (proportion <= 1.) & (abs(denom) > 1e-12))
        ranges.append(float(np.min(distance[valid])) if np.any(valid) else math.inf)
    first_sec = int(now-.1)
    first_nanosec = round((now-.1-first_sec)*1e9)
    first = first_sec+first_nanosec*1e-9
    scene = scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., acquired_at_s=[first+i*.0002 for i in range(500)],
        t_ref_s=now, source_epoch=0, rear_half_angle_deg=0.)
    raw = SimpleNamespace(ranges=ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., time_increment=.0002,
        header=SimpleNamespace(frame_id='laser_frame',
            stamp=SimpleNamespace(sec=first_sec, nanosec=first_nanosec)))
    return scene, raw
