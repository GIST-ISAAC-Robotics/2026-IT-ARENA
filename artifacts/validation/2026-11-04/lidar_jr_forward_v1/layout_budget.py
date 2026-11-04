"""JR 앞 이동 기하. mm, 평면 원기둥, +x 전방; stdout JSON. 주행·강도 계산 아님."""
import json
import math


def budget(sensor, front, harness=False):
    posts = [(front, y, 7) for y in (62, -62)] + [(-80, y, 7) for y in (52, -52)]
    obstacles = posts.copy()
    if harness:
        # 뒤 지주에 접하게 광축 반대 방향으로 배선관을 고정한 실장 예.
        for (x, y, dia), tube in zip(posts[2:], (10.49, 9.42)):
            dx = x - sensor
            delta = (dia+tube) / (2*math.hypot(dx, y))
            obstacles.append((x+delta*dx, y+delta*y, tube))
    intervals = []
    for x, y, dia in obstacles:
        dx = x-sensor
        a = math.degrees(math.atan2(y, dx)) % 360
        h = math.degrees(math.asin(dia/2/math.hypot(dx, y)))
        intervals.append((a-h, a+h))
    intervals.sort()  # 어느 그림자도 0도나 180도를 가로지르지 않는 이 배치에 한함
    assert all(0 < lo < hi < 180 or 180 < lo < hi < 360 for lo, hi in intervals)
    union = []
    for l, r in intervals:
        if union and union[-1][1] >= l:
            union[-1][1] = max(union[-1][1], r)
        else:
            union.append([l, r])
    return dict(front_clear_deg=round(2*min(union[0][0],360-union[-1][1]),2),
                rear_clear_deg=round(2*min(abs(edge-180) for pair in union for edge in pair),2),
                full_circle_blind_deg=round(sum(r-l for l,r in union),2))


def layout(sensor, front):
    return dict(sensor_x_mm=sensor, front_post_x_mm=front,
                nose_margin_mm=round(100-sensor-27.8,1), near_range_gap_on_centerline_mm=max(0,sensor-50),
                deck_overhang_to_x90_mm=90-front, posts_only=budget(sensor,front),
                rear_tubes_10_49_and_9_42_mm=budget(sensor,front,harness=True))


if __name__ == "__main__":
    print(json.dumps([layout(s,f) for s,f in [(50,10),(50,0),(55,10),(55,15),(60,10),(60,15),(60,20),(65,10),(65,25)]], indent=2))
