def sweep_queries(scene,steer,distance,config):
    """2cm 자세 외곽 + 각 unknown 교차 polygon. 얇은 결측은 일반 격자와 별개 검사."""
    curve=math.tan(steer)/config.wheelbase_m
    lx,wy=config.length_m/2,config.width_m/2
    corner=np.array([[-lx,-wy],[lx,-wy],[lx,wy],[-lx,wy]])
    border=np.unique(np.vstack(([(x,y) for x in np.linspace(-lx,lx,11) for y in (-wy,wy)],
                                [(x,y) for x in (-lx,lx) for y in np.linspace(-wy,wy,9)])),axis=0)
    unknown=[r for r in scene.rays if r.status!='return']
    queries=[]; thin=0
    half=math.pi/500
    co,si=math.cos(half),math.sin(half)
    origins=np.array([r.ray_origin_xy_m for r in unknown]).reshape(-1,2)
    directions=np.array([r.direction_xy for r in unknown]).reshape(-1,2)
    lower_all=directions@np.array([[co,-si],[si,co]])
    upper_all=directions@np.array([[co,si],[-si,co]])
    for travel in np.r_[np.arange(.02,distance,.02),distance]:
        yaw=curve*travel; c,s=math.cos(yaw),math.sin(yaw)
        x=travel if curve==0 else math.sin(yaw)/curve
        # 1-cos(yaw)는 직진에 가까울 때 소거 오차로 0이 된다.
        # 반각식은 같은 원호이며 작은 조향을 실제보다 크게 옆으로 흔들지 않는다.
        y=0. if curve==0 else 2*math.sin(yaw*.5)**2/curve
        center=np.array([x-config.wheelbase_m/2+config.wheelbase_m/2*c,y+config.wheelbase_m/2*s])
        rotation=np.array([[c,s],[-s,c]])
        b=border@rotation+center
        queries.extend(b[(abs(b[:,0])>lx+1e-10)|(abs(b[:,1])>wy+1e-10)])
        vertices=corner@rotation+center
        polygon=vertices.tolist()
        possible=np.zeros(len(unknown),dtype=bool)
        # 현재 몸체 안에만 걸치는 측후방 부채꼴에는 깊이 질의를 만들지 않는다.
        # 단순 전방 각도 삭제가 아니라 원래부터 제외한 명목 몸체와 같은 기하다.
        for exterior in outside_body(polygon,config.length_m,config.width_m):
            delta=np.array(exterior)[None]-origins[:,None]
            low=lower_all[:,None,0]*delta[:,:,1]-lower_all[:,None,1]*delta[:,:,0]
            high=upper_all[:,None,0]*delta[:,:,1]-upper_all[:,None,1]*delta[:,:,0]
            forward=np.sum(directions[:,None]*delta,axis=2)
            possible|=~(np.all(low<0,axis=1)|np.all(high>0,axis=1)|np.all(forward < -1e-12,axis=1))
        for i in np.flatnonzero(possible):
            r=unknown[i]
            cut=clip_halfplane(clip_halfplane(polygon,r.ray_origin_xy_m,lower_all[i].tolist(),1),
                               r.ray_origin_xy_m,upper_all[i].tolist(),-1)
            for piece in outside_body(cut,config.length_m,config.width_m):
                queries.extend(piece); queries.append(np.mean(piece,axis=0)); thin+=1
    queries=np.asarray(queries).reshape(-1,2)
    # clip의 닫힌 교차 조각에는 현재 몸체 경계 꼭짓점도 들어간다.
    # 일반 외곽 표본과 동일한 기존 1e-10 m 수치 허용치로 제외한다.
    # 새 진입 영역이나 unknown을 자유 공간으로 치환하는 검사가 아니다.
    exterior=(abs(queries[:,0])>lx+1e-10)|(abs(queries[:,1])>wy+1e-10)
    return np.unique(queries[exterior],axis=0),thin
