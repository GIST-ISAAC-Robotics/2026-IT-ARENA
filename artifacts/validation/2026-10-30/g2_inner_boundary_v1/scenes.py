"""평가 전용 장면. 제어기는 이 값을 받지 않고 합성 광선으로만 본다.

cutin은 차량 기구를 무시하고 옆으로 미끄러지는 최악 도형이다. 실제 차량 궤적이 아니다.
pullout은 정지 상태에서 최대 조향으로 출발하는 bicycle 궤적이다(가속/반경은 평가 가정).
"""
import math

from g0_geometry import Footprint, TruthBox


def walls():
    boxes=[TruthBox(2.,sign*.45,.4,Footprint(8.,.05),label='wall') for sign in (-1,1)]
    return boxes+[TruthBox(x,0.,.4,Footprint(.05,2.),label='wall') for x in (-2.025,6.025)]


def pullout_pose(scene, at):
    """정지 출발 후 원호, 이후 직선. 표적 뒤축이 아닌 중심 근사다."""
    elapsed=max(0.,at-scene['start']);a=scene.get('acceleration',1.);top=scene.get('speed',.5)
    reach=top/a
    s=.5*a*elapsed*elapsed if elapsed<=reach else .5*a*reach*reach+top*(elapsed-reach)
    radius=scene.get('radius',.35);limit=scene.get('yaw_limit',.6)
    arc=min(s,radius*limit);turn=arc/radius
    x=radius*math.sin(turn);y=-radius*(1-math.cos(turn))
    rest=s-arc
    return 1.2+x+rest*math.cos(turn),y-rest*math.sin(turn),-turn


def make_boxes(scene):
    kind=scene['kind']

    def boxes(at,target_y=0.,restart=False):
        base=walls()
        if kind=='fixed':
            return base+[TruthBox(scene.get('x',1.2),scene.get('y',0.),.12,yaw_rad=scene.get('yaw',0.))]
        if kind=='cutin':
            moved=min(scene['distance'],scene['speed']*max(0.,at-scene['start']))
            return base+[TruthBox(1.2,scene['direction']*moved,.12)]
        if kind=='pullout':
            x,y,yaw=pullout_pose(scene,at)
            return base+[TruthBox(x,y,.12,yaw_rad=yaw)]
        if kind=='second':
            return base+[TruthBox(scene['x2'],scene['y2'],.12),TruthBox(1.2,0.,.12)]
        if kind=='second_moving':
            moved=min(scene['distance'],scene['speed']*max(0.,at-scene['start']))
            return base+[TruthBox(scene['x2'],scene['y2']+scene['direction']*moved,.12),TruthBox(1.2,0.,.12)]
        if kind=='pillars':
            size=Footprint(.02,.02)
            rest=[TruthBox(1.2+dx,dy,.12,size) for dx in (-.09,.09) for dy in (-.065,.065)]
            return base+rest
        if kind=='bump':
            # 표적 오른쪽 옆면의 작은 실제 돌출. 후보 정렬이 가릴 수 있는 크기를 일부러 포함한다.
            delta=scene['delta']
            bump=TruthBox(1.2+scene.get('dx',0.),-(.075+delta/2),.12,Footprint(scene['length'],delta))
            return base+[bump,TruthBox(1.2,0.,.12)]
        if kind=='wallbump':
            delta=scene['delta']
            bump=TruthBox(1.2,-(.425-delta/2),.12,Footprint(scene['length'],delta),label='bump')
            return base+[bump,TruthBox(1.2,0.,.12)]
        raise ValueError('unknown evaluation scene')
    return boxes


def biased(original, bias):
    """표면별 상관 거리 편향을 평가 쪽 광선에만 넣는다. 반환 시 측정 범위는 그대로다."""
    kind=bias['kind'];value=bias.get('value',.03)

    def distance(origin,direction,box,low,high):
        hit=original(origin,direction,box,low,high)
        if hit is None:
            return None
        wall=box.label=='wall'
        if kind=='all':
            return hit+value
        if kind=='wall':
            return hit+value if wall else hit
        if kind=='target':
            return hit if wall else hit+value
        if kind=='target_view':
            # 옆면을 비스듬히 볼수록 길게 읽는 시야 의존 편향. 뒷면은 그대로다.
            if wall:
                return hit
            hy=origin[1]+direction[1]*hit
            side=abs(abs(hy-box.y_m)-box.footprint.width_m/2)<.003
            return hit+value*abs(direction[0]) if side else hit
        raise ValueError('unknown range bias')
    return distance
