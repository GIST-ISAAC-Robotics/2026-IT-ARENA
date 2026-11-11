"""저장 비교 전용. 실행 소스는 변경하지 않고 두 연산의 묶음 크기만 비교한다."""
import inspect
import math
from arena_autonomy import observation_support as support
from arena_autonomy import observed_lateral_trial as lateral


def depth_variant(chunk):
    if chunk not in (128, 512, 1024):
        raise ValueError('registered batch size required')
    text = inspect.getsource(support.depth_support)
    assert text.count('range(0, len(q), 128)') == 1 and text.count('q[start:start+128]') == 1
    text = text.replace('range(0, len(q), 128)', f'range(0, len(q), {chunk})')
    text = text.replace('q[start:start+128]', f'q[start:start+{chunk}]')
    scope = dict(support.__dict__)
    exec(compile(text, f'<depth_chunk_{chunk}>', 'exec'), scope)
    return scope['depth_support']


def border_variant():
    text = inspect.getsource(lateral.path_observation_queries)
    old = '''    for index, (pose, polygon) in enumerate(zip(poses, corners)):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        world_border = border@np.array([[c, s], [-s, c]])+pose[:2]
        queries.extend(world_border)
'''
    new = '''    # Scalar sin/cos and per-matrix multiplication are retained.
    cs = [(math.cos(pose[2]), math.sin(pose[2])) for pose in poses]
    rotations = np.asarray([[[c, s], [-s, c]] for c, s in cs]).reshape(-1, 2, 2)
    world_border = np.matmul(border[None, :, :], rotations)+np.asarray(poses)[:, None, :2]
    queries.extend(world_border.reshape(-1, 2))
'''
    assert text.count(old) == 1
    scope = dict(lateral.__dict__)
    exec(compile(text.replace(old, new), '<batched_border>', 'exec'), scope)
    return scope['path_observation_queries']
