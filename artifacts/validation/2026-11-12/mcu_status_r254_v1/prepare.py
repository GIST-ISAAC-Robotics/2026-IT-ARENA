"""실행 중인 주행 소스는 건드리지 않고 송신/시계 연결 후보를 만든다."""
from pathlib import Path
import shutil

A=Path(__file__).resolve().parent
R=A.parents[3]
source=R/'src/arena_vehicle_interface/portable/actuation_tx.hpp'
s=source.read_text('utf-8')
before='  std::int64_t line_ns_per_byte = 0;'
assert before in s
s=s.replace(before,'  bool offer_status_v1 = false;  // 명시 센서 연결에서만 원 MCU 상태/시각 포함\n'+before,1)
old='const Lease& lease, bool output_ready) {'
assert old in s
s=s.replace(old,'const Lease& lease, bool output_ready, bool with_status, bool estop_held, const char* reason) {',1)
old='    w.boolean(output_ready);\n    w.lit("}");'
assert old in s
s=s.replace(old,'''    w.boolean(output_ready);
    if (with_status) {
      w.lit(",\\\"status_now_us\\\":");
      w.i64(lease.issued_us);
      w.lit(",\\\"estop_held\\\":");
      w.boolean(estop_held);
      w.lit(",\\\"reason\\\":");
      w.text(reason);
    }
    w.lit("}");''',1)
old='lease.lease, output_ready_);'
assert old in s
s=s.replace(old,'lease.lease, output_ready_, config_.offer_status_v1, receiver_.estop_held(), receiver_.reason());',1)
# 같은 writer를 쓰는 worst_payload도 선택한 필드 크기를 포함한다.
old='Lease{}, false);'
assert old in s
s=s.replace(old,'Lease{}, false, config_.offer_status_v1, false, "");',1)
directory=A/'portable'
directory.mkdir(exist_ok=True)
(directory/'actuation_tx.hpp').write_text(s,encoding='utf-8')
print('prototype prepared; common sources unchanged')
