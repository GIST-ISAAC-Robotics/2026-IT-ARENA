"""저장된 E1 기록을 포스터용 지도/센서 그림으로 작성. 제어/시뮬 실행 없음."""
from pathlib import Path
import hashlib
import json
import sys
import shutil
import math

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection, LineCollection
from matplotlib.colors import Normalize
from matplotlib.font_manager import FontProperties, fontManager
from matplotlib.patches import Rectangle
import numpy as np

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parents[1]
RUN = ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1/e_all_posts_blank'
SUMMARY = RUN.parent/'e1_summary.json'
SCENE = ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json'
FONT = Path('C:/Windows/Fonts/malgun.ttf') if sys.platform == 'win32' else Path('/mnt/c/Windows/Fonts/malgun.ttf')
fontManager.addfont(str(FONT))
plt.rcParams.update({'font.family': FontProperties(fname=str(FONT)).get_name(), 'axes.unicode_minus': False,
                     'svg.fonttype': 'path', 'font.size': 13})

def read(p):
    return json.loads(p.read_text(encoding='utf-8'))

def sha(p):
    return hashlib.file_digest(p.open('rb'), 'sha256').hexdigest()

summary = read(SUMMARY)
trace = read(RUN/'trace.json')
scene = read(SCENE)
start = summary['race_start_sim_s']
rows = [r for r in trace if r['time'] >= start]
t = np.array([r['time'] for r in rows])
xy = np.array([[r['x'],r['y']] for r in rows])
speed = np.array([abs(r['dynamics']['truth_longitudinal_mps'])*3.6 for r in rows])
assert summary['passed'] and summary['completed_laps'] == 1
assert np.isfinite(xy).all() and np.isfinite(speed).all() and (np.diff(t)>0).all()
assert abs(t[-1]-start-summary['elapsed_to_last_sample_s']) < 1e-6
assert abs(speed.max()/3.6-summary['peak_ground_speed_mps']) < 1e-8

sources = [RUN/'trace.json', SUMMARY, RUN/'report.json', SCENE, RUN/'follow_decisions.jsonl']
geometry = {}
def obj_top_polygons(path):
    vertices=[]; polys=[]; seen=set()
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('v '): vertices.append([float(x) for x in line.split()[1:4]])
        elif line.startswith('f '):
            p=np.array([vertices[int(q.split('/')[0])-1] for q in line.split()[1:]])
            if np.ptp(p[:,2])>1e-7: continue
            a=p[:,:2]
            key=tuple(sorted(map(tuple,np.round(a,8))))
            if key not in seen:
                seen.add(key);polys.append(a)
    return polys

for name in ('grass','track_surface','walls'):
    path=RUN/f'batched_{name}_0.obj';sources.append(path)
    geometry[name]=obj_top_polygons(path)
limits=np.concatenate(geometry['walls'])
lo=limits.min(axis=0)-.28;hi=limits.max(axis=0)+.28
assert (xy>=lo).all() and (xy<=hi).all()
norm=Normalize(0,11)
cmap=plt.get_cmap('turbo')
items=[]

def palette(theme):
    if theme=='dark':return dict(bg='#09131e',ink='#edf5fa',muted='#91a9ba',road='#243547',grass='#142e2c',wall='#6b8293',grid='#819aaa',line='#1c3043')
    return dict(bg='#ffffff',ink='#142a3b',muted='#5c7180',road='#e6edf2',grass='#d7e7df',wall='#879cac',grid='#526676',line='#dce5ec')

def map_ax(ax,p,course_geometry=geometry,positions=xy,velocities=speed,scene_data=scene):
    ax.set_facecolor(p['bg'])
    for k in ('grass','track_surface','walls'):
        ax.add_collection(PolyCollection(course_geometry[k],facecolors=p[{'track_surface':'road'}.get(k,k[:-1] if k=='walls' else k)],edgecolors='none',antialiaseds=False,zorder=1))
    for slot in scene_data['starting_grid']['slots']:
        ax.add_patch(Rectangle((slot['x']-.085,slot['y']-.125),.17,.25,facecolor=p['grid'],edgecolor='none',alpha=.7,zorder=2))
    seg=np.stack([positions[:-1],positions[1:]],axis=1)
    # 표본 위치를 직선으로 잇고 양 끝 속도의 평균으로 선분 색을 표시한다. 평활화 없음.
    lc=LineCollection(seg,cmap=cmap,norm=norm,linewidths=4.7,zorder=5,capstyle='round')
    lc.set_array((velocities[:-1]+velocities[1:])/2);ax.add_collection(lc)
    # 출발과 마지막 표본은 같은 직선 위의 서로 다른 지점이다.
    ax.scatter(*positions[0],s=80,facecolor=p['bg'],edgecolor=p['ink'],linewidth=1.6,zorder=7)
    ax.annotate('출발',xy=positions[0],xytext=(8.6,2.75),color=p['ink'],fontsize=12,
                arrowprops=dict(arrowstyle='-',color=p['muted'],lw=.8))
    ax.scatter(*positions[-1],s=48,marker='s',facecolor=p['ink'],edgecolor=p['bg'],linewidth=.7,zorder=8)
    ax.annotate('기록 종료',xy=positions[-1],xytext=(8.15,6.15),color=p['ink'],fontsize=12,
                arrowprops=dict(arrowstyle='-',color=p['muted'],lw=.8))
    bounds=np.concatenate(course_geometry['walls']);lower=bounds.min(axis=0)-.28;upper=bounds.max(axis=0)+.28
    ax.set_xlim(lower[0],upper[0]);ax.set_ylim(lower[1],upper[1]);ax.set_aspect('equal');ax.axis('off')
    return lc

def save(fig,relative,title,note,category,extra=None):
    stem=OUT/relative;stem.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(stem.with_suffix('.png'),dpi=300,facecolor=fig.get_facecolor())
    fig.savefig(stem.with_suffix('.svg'),facecolor=fig.get_facecolor())
    w,h=fig.get_size_inches()*300
    row=dict(file=stem.with_suffix('.png').relative_to(OUT).as_posix(),title=title,category=category,
             method='저장된 시뮬레이션 수치 재시각화',width=round(w),height=round(h),
             sha256=sha(stem.with_suffix('.png')),vector=stem.with_suffix('.svg').relative_to(OUT).as_posix(),
             date='2026-11-09',note=note,evidence=SUMMARY.relative_to(ROOT).as_posix())
    if extra:row.update(extra)
    items.append(row);plt.close(fig)

for theme in ('dark','light'):
    p=palette(theme);fig=plt.figure(figsize=(12,14),facecolor=p['bg'])
    fig.text(.075,.951,'코스 속도 분포',size=30,weight='bold',color=p['ink'])
    fig.text(.075,.920,'IT ARENA  /  v2026.11.06  /  2026.11.09 E1',size=13,color=p['muted'])
    ax=fig.add_axes([.065,.15,.87,.72]);lc=map_ax(ax,p)
    cb=fig.colorbar(lc,cax=fig.add_axes([.25,.106,.5,.014]),orientation='horizontal',ticks=[0,2,4,6,8,10])
    cb.outline.set_visible(False);cb.ax.tick_params(colors=p['ink'],labelsize=12,length=0,pad=8)
    cb.set_label('차체 속도  [km/h]',color=p['ink'],labelpad=10,size=13)
    fig.text(.075,.032,'Gazebo 시뮬레이션 저장 기록 · 실차 시험 아님',color=p['muted'],size=10)
    fig.text(.925,.032,'출발부터 STOP 직전까지',color=p['muted'],size=10,ha='right')
    save(fig,f'02_speed_maps/course_speed_{theme}','최신 코스 속도 지도 · '+('어두운 배경' if theme=='dark' else '밝은 배경'),
         'E1 완료 기록의 평가용 x·y와 차체 종방향 속도 절댓값. 지도·정답은 평가 그림에만 사용. 출발~STOP 직전 전체 구간; 짧은 저속을 제거하지 않음.','속도 지도')

p=palette('dark');fig=plt.figure(figsize=(16,10),facecolor=p['bg'])
fig.text(.045,.934,'주행 기록, 한눈에.',size=30,weight='bold',color=p['ink'])
fig.text(.045,.887,'IT ARENA  /  최신 코스 속도 지도와 시간별 차체 속도',size=13,color=p['muted'])
ax=fig.add_axes([.025,.10,.56,.74]);lc=map_ax(ax,p)
fig.text(.61,.79,'46.65 m',color=p['ink'],size=27,weight='bold')
fig.text(.61,.744,'기록된 본선 진행 거리',color=p['muted'],size=12)
fig.text(.61,.66,'29.917 s',color=p['ink'],size=27,weight='bold')
fig.text(.61,.614,'출발 → 마지막 기록 표본',color=p['muted'],size=12)
fig.text(.815,.66,f'{speed.max():.2f} km/h',color='#67dfd2',size=23,weight='bold')
fig.text(.815,.614,'구간 최고 차체 속도',color=p['muted'],size=12)
tx=fig.add_axes([.62,.27,.33,.255]);tx.set_facecolor(p['bg'])
tt=t-start;segments=np.stack([np.column_stack((tt[:-1],speed[:-1])),np.column_stack((tt[1:],speed[1:]))],axis=1)
lc2=LineCollection(segments,cmap=cmap,norm=norm,linewidths=1.8);lc2.set_array((speed[:-1]+speed[1:])/2);tx.add_collection(lc2)
tx.set(xlim=(0,30),ylim=(0,11),xlabel='출발 후 경과 시간 [s]',ylabel='차체 속도 [km/h]')
tx.tick_params(colors=p['muted'],labelsize=10);tx.xaxis.label.set_color(p['muted']);tx.yaxis.label.set_color(p['muted'])
tx.grid(color=p['line'],lw=.6);tx.set_axisbelow(True)
for spine in tx.spines.values():spine.set_color(p['line'])
cb=fig.colorbar(lc,cax=fig.add_axes([.62,.17,.33,.016]),orientation='horizontal',ticks=[0,2,4,6,8,10]);cb.outline.set_visible(False);cb.ax.tick_params(colors=p['ink'],labelsize=10,length=0);cb.set_label('지도 색상: km/h',color=p['muted'],size=11)
fig.text(.045,.04,'Gazebo 시뮬레이션 · v2026.11.06 · 2026.11.09 E1 · 단독 주행 1회 기록',size=10,color=p['muted'])
save(fig,'02_speed_maps/course_speed_dashboard','코스 속도 지도와 주행 프로필',
     '거리·시간은 원 보고 정의. 공식 계시선 랩타임이 아니며 STOP 이후 제동 구간 제외. 원 결과의 짧은 저속을 그대로 표시.','속도 지도')

# 기존 관측 보정 함수만 사용한다. 제어 판단/경로를 다시 생성하지 않고 저장된 path_points를 표시한다.
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
for mod in ('follow_trace','lidar_motion','scene_motion'):
    sources.append(ROOT/f'src/arena_autonomy/arena_autonomy/{mod}.py')
snapshots=[]
with (RUN/'follow_decisions.jsonl').open(encoding='utf-8') as stream:
    header=json.loads(next(stream))
    for line in stream:
        event=json.loads(line)
        if event['kind']!='call':continue
        data=event['data'];ref=data['input']['reference_s']
        if not any(abs(ref-target)<1e-7 for target in (14.52,23.12)):continue
        info,scan,history=restore_call(data['input'],MotionConfig(**header['data']['motion_config']))
        ms=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
        points=np.asarray(ms.occupied_endpoints_xy_m);path=np.asarray(data['output'][2]['path_points'])
        assert np.isfinite(points).all() and np.isfinite(path).all()
        snapshots.append(dict(ref=ref,seq=event['seq'],points=points,path=path))
assert len(snapshots)==2
for theme in ('dark','light'):
    p=palette(theme);fig,axes=plt.subplots(1,2,figsize=(16,8),facecolor=p['bg'])
    fig.subplots_adjust(left=.07,right=.965,top=.79,bottom=.14,wspace=.22)
    fig.text(.065,.925,'LiDAR로 보고, 경로를 만들다',size=27,weight='bold',color=p['ink'])
    fig.text(.065,.865,'움직임을 보정한 관측점과 당시 기록된 지역 경로',size=14,color=p['muted'])
    for ax,s in zip(axes,snapshots):
        ax.set_facecolor(p['bg']);pts=s['points'];path=s['path']
        ax.scatter(pts[:,0],pts[:,1],s=10,facecolor='#48c4d3' if theme=='dark' else '#28748e',edgecolor='none',label='LiDAR 관측점',zorder=4)
        ax.plot(path[:,0],path[:,1],lw=2.5,color='#ffa95b' if theme=='dark' else '#d45b19',label='기록된 계획 경로',zorder=5)
        ax.add_patch(Rectangle((-.1,-.075),.2,.15,facecolor=p['ink'],alpha=.85,zorder=6))
        ax.annotate('',xy=(.22,0),xytext=(0,0),arrowprops=dict(arrowstyle='->',color=p['ink'],lw=1.4),zorder=7)
        ax.set(xlim=(-.5,2.1),ylim=(-2.15,2.15),xlabel='전방 x [m]',ylabel='좌측 y [m]')
        ax.set_aspect('equal');ax.grid(color=p['line'],lw=.7);ax.set_axisbelow(True)
        ax.set_title(f"시뮬레이션 t = {s['ref']:.2f} s",color=p['ink'],fontsize=14,pad=12)
        ax.tick_params(colors=p['muted'],labelsize=10);ax.xaxis.label.set_color(p['muted']);ax.yaxis.label.set_color(p['muted'])
        for spine in ax.spines.values():spine.set_color(p['line'])
        leg=ax.legend(loc='lower left',facecolor=p['bg'],edgecolor=p['line'],fontsize=10)
        for txt in leg.get_texts():txt.set_color(p['ink'])
    fig.text(.065,.045,'Gazebo E1 저장 기록 · 경로선은 실제 궤적과 다름 · 점이 없는 영역은 자유 공간을 뜻하지 않음',size=10,color=p['muted'])
    save(fig,f'03_lidar/lidar_paths_{theme}','LiDAR 관측점과 지역 경로 · '+('어두운 배경' if theme=='dark' else '밝은 배경'),
         'E1의 t=14.52/23.12초 저장 스캔을 원 운동보정 함수로 같은 시각에 변환. 경로는 해당 출력 path_points 원값. 새 제어 판단·주행·자유공간 채움 없음.','LiDAR·경로',
         dict(evidence=(RUN/'follow_decisions.jsonl').relative_to(ROOT).as_posix()))

originals=[
('artifacts/validation/2026-11-08/c1_corner_observation_probe_v1/same_scan_corner_path.png','01_noisy_corner_comparison.png','잡음이 있는 코너의 경로 표현 비교','같은 저장 관측에서 경로 제안 두 가지를 비교한 분석. 개선 주행 결과가 아님.'),
('artifacts/validation/2026-10-13/gapped_boundary_v3/fragments.png','02_observed_wall_fragments.png','가려진 벽의 관측 조각과 경로','2026-10-13 과거 분석. 빈 공간을 자유 공간으로 간주하지 않는 관측 처리.'),
('artifacts/validation/2026-10-28/traffic_corridor_roles_v1/boundary_roles_130s.png','03_corner_boundary_roles.png','코너에서 안쪽·바깥쪽 벽 구분','2026-10-28 저장 스캔 분석. 후보 경로 그림이며 실제 통과 증명은 아님.'),
('artifacts/validation/2026-11-04/lidar_packaging_v3/top_view.png','04_sensor_packaging_study.png','센서와 부품 배치 검토','2026-11-04 임시 기구 배치 연구. 최종 제작도나 실물 조립 검증이 아님.')]
from PIL import Image
for src,name,title,note in originals:
    source=ROOT/src;dest=OUT/'04_reference_plots'/name;dest.parent.mkdir(exist_ok=True)
    before=sha(source);shutil.copy2(source,dest);assert sha(dest)==before==sha(source)
    with Image.open(dest) as im:w,h=im.size
    items.append(dict(file=dest.relative_to(OUT).as_posix(),title=title,category='기존 분석 그림',source_repo=src,source_sha256=before,sha256=before,width=w,height=h,method='원본 복사',date=src.split('/')[2],note=note,evidence=str(source.parent.relative_to(ROOT)/'README.md')))

# 직각 코스로 옮기기 전 마지막 완료 단독 증속 기록. 해당 실행의 곡선 노면/벽 메시를 사용한다.
legacy_run=ROOT/'artifacts/validation/2026-11-05/solo_straight_closed_loop_v1/run1'
legacy_report=read(legacy_run/'report.json')
legacy_scene_path=ROOT/'src/arena_gazebo/worlds/it_arena_official/scene.json'
legacy_scene=read(legacy_scene_path)
legacy_start=legacy_report['official_signal_evidence']['first_start_sim_s']
legacy_rows=[r for r in read(legacy_run/'trace.json') if r['time']>=legacy_start]
legacy_xy=np.array([[r['x'],r['y']] for r in legacy_rows])
legacy_t=np.array([r['time'] for r in legacy_rows])
legacy_v=np.array([abs(r['dynamics']['truth_longitudinal_mps'])*3.6 for r in legacy_rows])
legacy_duration=float(legacy_t[-1]-legacy_start)
legacy_geo={name:obj_top_polygons(legacy_run/f'batched_{name}_0.obj') for name in ('grass','track_surface','walls')}
assert legacy_report['passed'] and legacy_report['completed_laps']==1
assert np.isfinite(legacy_xy).all() and np.isfinite(legacy_v).all() and (np.diff(legacy_t)>0).all()
assert abs(legacy_v.max()/3.6-legacy_report['peak_ground_speed_mps'])<1e-8
sources.extend([legacy_run/'report.json',legacy_run/'trace.json',legacy_scene_path])
sources.extend(legacy_run/f'batched_{name}_0.obj' for name in legacy_geo)
legacy_evidence=legacy_run.parent.relative_to(ROOT).as_posix()+'/README.md'
for theme in ('dark','light'):
    p=palette(theme);fig=plt.figure(figsize=(12,14),facecolor=p['bg'])
    fig.text(.075,.951,'코스 속도 분포',size=30,weight='bold',color=p['ink'])
    fig.text(.075,.920,'곡선 코스  /  v2026.11.01  /  2026.11.05 주행 기록',size=13,color=p['muted'])
    ax=fig.add_axes([.065,.15,.87,.72]);lc=map_ax(ax,p,legacy_geo,legacy_xy,legacy_v,legacy_scene)
    cb=fig.colorbar(lc,cax=fig.add_axes([.25,.106,.5,.014]),orientation='horizontal',ticks=[0,2,4,6,8,10])
    cb.outline.set_visible(False);cb.ax.tick_params(colors=p['ink'],labelsize=12,length=0,pad=8)
    cb.set_label('차체 속도  [km/h]',color=p['ink'],labelpad=10,size=13)
    fig.text(.075,.032,'Gazebo 시뮬레이션 저장 기록 · 실차 시험 아님',color=p['muted'],size=10)
    fig.text(.925,.032,'코스 변경 전 버전',color=p['muted'],size=10,ha='right')
    save(fig,f'02_speed_maps/curved_course_speed_{theme}','이전 곡선 코스 속도 지도 · '+('어두운 배경' if theme=='dark' else '밝은 배경'),
         '코스 변경 직전 마지막 완료 단독 증속 기록(2026-11-05). 당시 곡선 코스 실행 메시와 당시 차체 속도 사용. 직각 코스 데이터를 옮긴 것이 아님.','속도 지도',dict(evidence=legacy_evidence,date='2026-11-05'))

p=palette('dark');fig=plt.figure(figsize=(16,10),facecolor=p['bg'])
fig.text(.045,.934,'곡선 위의 속도.',size=30,weight='bold',color=p['ink'])
fig.text(.045,.887,'IT ARENA  /  코스 변경 전 주행 기록 · v2026.11.01',size=13,color=p['muted'])
ax=fig.add_axes([.025,.10,.56,.74]);lc=map_ax(ax,p,legacy_geo,legacy_xy,legacy_v,legacy_scene)
fig.text(.61,.79,f"{legacy_report['progress_m']:.2f} m",color=p['ink'],size=27,weight='bold')
fig.text(.61,.744,'기록된 본선 진행 거리',color=p['muted'],size=12)
fig.text(.61,.66,f'{legacy_duration:.3f} s',color=p['ink'],size=27,weight='bold')
fig.text(.61,.614,'출발 → 마지막 기록 표본',color=p['muted'],size=12)
fig.text(.815,.66,f'{legacy_v.max():.2f} km/h',color='#67dfd2',size=23,weight='bold')
fig.text(.815,.614,'구간 최고 차체 속도',color=p['muted'],size=12)
tx=fig.add_axes([.62,.27,.33,.255]);tx.set_facecolor(p['bg']);lt=legacy_t-legacy_start
segs=np.stack([np.column_stack((lt[:-1],legacy_v[:-1])),np.column_stack((lt[1:],legacy_v[1:]))],axis=1)
lc2=LineCollection(segs,cmap=cmap,norm=norm,linewidths=1.8);lc2.set_array((legacy_v[:-1]+legacy_v[1:])/2);tx.add_collection(lc2)
tx.set(xlim=(0,math.ceil(legacy_duration)),ylim=(0,11),xlabel='출발 후 경과 시간 [s]',ylabel='차체 속도 [km/h]')
tx.tick_params(colors=p['muted'],labelsize=10);tx.xaxis.label.set_color(p['muted']);tx.yaxis.label.set_color(p['muted'])
tx.grid(color=p['line'],lw=.6);tx.set_axisbelow(True)
for spine in tx.spines.values():spine.set_color(p['line'])
cb=fig.colorbar(lc,cax=fig.add_axes([.62,.17,.33,.016]),orientation='horizontal',ticks=[0,2,4,6,8,10]);cb.outline.set_visible(False);cb.ax.tick_params(colors=p['ink'],labelsize=10,length=0);cb.set_label('지도 색상: km/h',color=p['muted'],size=11)
fig.text(.045,.04,'Gazebo 시뮬레이션 · v2026.11.01 · 2026.11.05 단독 주행 1회 기록',size=10,color=p['muted'])
save(fig,'02_speed_maps/curved_course_speed_dashboard','이전 곡선 코스 속도 지도와 주행 프로필',
     '시각은 최신 지도와 같은 출발~마지막 표본 정의(공식 랩타임 아님). 기존 보고서의 31.850초는 초록 평가 표본~거리 한랩 정의이므로 서로 다른 수치. STOP 이후 제외, 43.61m 짧은 정지 보존.','속도 지도',dict(evidence=legacy_evidence,date='2026-11-05'))

report=dict(scope='저장된 시뮬레이션 결과 재시각화; 신규 주행/검증 아님',run=RUN.relative_to(ROOT).as_posix(),
            race_start_sim_s=start,last_sample_sim_s=float(t[-1]),plotted_sample_count=len(rows),
            peak_kmh=float(speed.max()),progress_m=summary['progress_m'],duration_s=summary['elapsed_to_last_sample_s'],
            speed_definition='abs(dynamics.truth_longitudinal_mps) * 3.6; commands/odometry not used',
            interpolation='position segments linear; color mean of endpoint speeds; no smoothing or omitted slow sections',
            lidar_snapshots=[dict(ref_s=s['ref'],seq=s['seq'],observed_points=len(s['points']),path_points=len(s['path'])) for s in snapshots],
            legacy=dict(run=legacy_run.relative_to(ROOT).as_posix(),start_sim_s=legacy_start,last_sample_sim_s=float(legacy_t[-1]),
                        duration_s=legacy_duration,peak_kmh=float(legacy_v.max()),samples=len(legacy_rows),progress_m=legacy_report['progress_m']),
            sources={p.relative_to(ROOT).as_posix():sha(p) for p in sources},items=items)
(OUT/'figure_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps({'images':len(items),'peak_kmh':report['peak_kmh'],'samples':len(rows),'map_px':[3600,4200]},ensure_ascii=False))
