"""명령 만료 출력은 0으로 유지하며 유효 관측 평가만 계속하는 사본."""
import ast
from pathlib import Path
import arena_autonomy.lidar_safety as runtime

path=Path(runtime.__file__)
source=path.read_text()
old='''            if (not 0 <= now - seconds(self.command.header.stamp) <= .2 or
                not 0 <= now - seconds(self.scan.header.stamp) <= .3 or
                max(time.monotonic() - self.scan_wall, time.monotonic() - self.command_wall) > self.wall_timeout):
                raise MotionUnavailable("stale_input")'''
new='''            command_age = now - seconds(self.command.header.stamp)
            if (command_age < 0 or not 0 <= now - seconds(self.scan.header.stamp) <= .3 or
                time.monotonic() - self.scan_wall > self.wall_timeout):
                raise MotionUnavailable("stale_input")
            command_stale = command_age > .2 or time.monotonic() - self.command_wall > self.wall_timeout
            if command_stale and getattr(self, 'partial_policy', None) is None:
                raise MotionUnavailable("stale_input")'''
assert source.count(old)==1
source=source.replace(old,new)
needle="                if not partial['allowed']:\n"
assert source.count(needle)==1
source=source.replace(needle,"                if command_stale:\n                    raise MotionUnavailable('stale_input')\n"+needle)
cls=next(n for n in ast.parse(source).body if isinstance(n,ast.ClassDef) and n.name=='LidarSafety')
fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='control')
namespace=vars(runtime).copy()
exec(compile(ast.Module(body=[fn],type_ignores=[]),'r228_stale_command_candidate','exec'),namespace)
control=namespace['control']
