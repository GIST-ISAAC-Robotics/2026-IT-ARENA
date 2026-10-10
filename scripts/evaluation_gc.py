"""짧은 평가 프로세스의 순환 GC만 선택적으로 미룬다. 대회 제어기에는 사용하지 않는다.

참조 카운팅은 계속 동작한다. 새 로그 객체도 누적되므로 freeze만으로는 반복 GC를 막지 못한다.
defer는 시간/RSS 예산을 가진 한 번의 probe에서만 허용하고, 종료 처리 뒤 원래 상태를 복원한다.
"""
import gc
import time


def peak_rss_kib():
    # 실제 실행 환경은 Linux/WSL. ru_maxrss는 현재 RSS가 아닌 프로세스 최대 RSS다.
    import resource
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss


class EvaluationGC:
    def __init__(self, mode='native', *, grid=False, collector=gc, clock=time.monotonic,
                 rss=peak_rss_kib, max_wall_s=600., max_rss_kib=2*1024*1024):
        if mode not in ('native', 'defer') or (mode == 'defer' and not grid):
            raise ValueError('GC defer is evaluation sim_grid only')
        self.mode, self.gc, self.clock, self.rss = mode, collector, clock, rss
        self.max_wall_s, self.max_rss_kib = max_wall_s, max_rss_kib
        self.started = self.finished = False
        self.record = {'mode': mode, 'scope': 'evaluator process only; reference counting remains enabled',
                       'wall_limit_s': max_wall_s, 'peak_rss_limit_kib': max_rss_kib}

    def start(self):
        if self.started:
            raise RuntimeError('GC scope cannot be reused')
        self.started = True
        self.was_enabled = self.gc.isenabled()
        # RSS 모듈의 첫 import·진단 dict 생성도 GC를 일으킬 수 있다.
        # defer를 먼저 적용한 다음 계수 기준을 잡아 준비 중 GC를 창 안 GC로 세지 않는다.
        if self.mode == 'defer':
            self.gc.disable()
        self.begin = self.clock()
        self.next_check = self.begin
        self.record.update(enabled_before=self.was_enabled, thresholds_before=list(self.gc.get_threshold()),
                           begin_wall_s=self.begin,
                           collections_before=[s['collections'] for s in self.gc.get_stats()],
                           peak_rss_before_kib=self.rss())
        self.record['enabled_during'] = self.gc.isenabled()

    def check(self):
        """벽시계 1초마다 평가 전용 예산을 감시한다. 차량의 보호 기준과는 별개다."""
        now = self.clock()
        if now < self.next_check:
            return
        self.next_check = now+1.
        if self.mode == 'defer' and self.gc.isenabled():
            raise RuntimeError('evaluation GC defer unexpectedly re-enabled')
        if self.mode == 'defer' and (now-self.begin > self.max_wall_s or self.rss() > self.max_rss_kib):
            raise RuntimeError('evaluation time/RSS budget exceeded')

    def restore(self):
        if not self.started or self.finished:
            return
        # 원래 상태 복원은 진단 수집 실패 여부와 독립적으로 보장한다.
        try:
            self.record.update(wall_duration_s=self.clock()-self.begin, end_wall_s=self.clock(),
                               peak_rss_after_kib=self.rss(),
                               enabled_before_restore=self.gc.isenabled(),
                               collections_after=[s['collections'] for s in self.gc.get_stats()])
        finally:
            if self.was_enabled:
                self.gc.enable()
            else:
                self.gc.disable()
            self.finished = True
            self.record['enabled_after_restore'] = self.gc.isenabled()
            self.record['restored'] = self.gc.isenabled() == self.was_enabled

