# r557 r553 최초 current radial 정지 원인

**같은 scan의 reference가 2 ms 늦어져 모든 점의 운동 반경이 1.000 mm 커진 것이 직접 원인이다.** 새 원시 장애물·point 순서 오류·다른 고정 margin·shape 메타 누락은 아니다. s=0에서 source176/177의 운동 원판이 고정 여유 상자에 각각20.66/26.65 µm 겹쳐 clearance가 0이 된다.

입력은 r553 `stall_extract/first_zero_publication_call_call.json['record']['data']`의 `input`과 `current_rebase.input`이다. 저장 header의 MotionConfig/FollowConfig 및 각 입력의 MotionHistory로 실제 `motion_scene`을 복원했다. Windows에서 점·반경과 실제 `swept_limit`만 계산했으며 경로 fit/전체 제어기·공통 소스 변경·WSL/장치 실행은 없다.

| 항목 | 계획 | current |
|---|---:|---:|
| reference |59.820 s|59.822 s|
| scan source first |59.70200000000058 s|동일|
| 원시 scan 전체/return source 순서 |480개|모두 동일|
| 실제 속도 |.061225042 m/s|동일|
| 현재/직전 조향 |−.28475436568260193 rad|동일|
| fixed margin |.025+.03=.055 m|동일|
| 이동 반경 min/max |.009098/.059000 m|.010098/.060000 m|
| margin0 raw sweep |1.00 m|1.00 m|
| fixed만 sweep |.92 m|.92 m|
| 기존 축별 운동 사각 sweep |0 m|0 m|
| r552 원판 운동 sweep |**.92 m**|**0 m**|

history는 같지 않다. wheel 마지막 시각59.813은 같고 gyro 마지막 시각은59.815→59.820이다. 새 기준으로 보정된 각 점의 이동량은 .168~1.916 mm다. 다만 교차 대조에서 **계획 점+current 반경=0**, **current 점+계획 반경=.92**이므로 원점 보정 이동보다 +1 mm 반경이 원인이다. 대조용으로만 계산했으며 제어에 오래된 반경을 전달하지 않았다.

## 최초 간섭점

고정 여유 상자의 반길이=.1585 m, 반폭=.130 m다. s=0에서는 `hypot(max(abs(x)-.1585,0),max(abs(y)-.130,0)) <= .5*(reference-acquired)`를 검사한다.

| source | 단계 | 점 (x,y) m | 필요 이동 norm mm | 허용 반경 mm | 여유 mm |
|---|---|---|---:|---:|---:|
|176|계획|(.171307040,.170801469)|42.764239|41.812500|+0.951739|
|176|current|(.171121683,.170888077)|42.791842|42.812500|−0.020658|
|177|current|(.173516251,.169959910)|42.688197|42.714844|−0.026647|

source176의 취득 시각59.73637500000058은 보존된다. 계획의 첫 radial 간섭 표본은 s=.96 m로 기존 .04 m 차감 뒤 .92 m다. current는 시작부터 겹친다. 이는 부동소수점 eps만의 오류가 아니라 1 mm age 증가가 모델 경계를 넘은 것이다.

## 제한된 fixed-only 저속 정책에 대한 의견

root가 검토 중인 **요청 상한 .1 m/s 및 매번 현재 |실측속도|≤.2 m/s에서만 55 mm fixed box를 적용**하는 선택은 원인에 맞고 포기하는 보장이 명확하다. 같은 raw점·현재/직전 조향 sweep·standstill/braking·STOP을 유지하고, 일반/고속 후보는 기존 bound를 유지하는 범위가 적절하다. normal에 명시 정책을 표시하고 current/retained에서도 새 현재속도로 조건을 재평가해야 한다.

다만 취득 이후 상대 물체가 차량 쪽으로 이동했을 가능성을 더 이상 이 bound로 보호하지 않는다. 독립 lidar_safety의 raw sweep도 원래 visible-point 운동 반경을 적용하지 않으므로 이 보장을 대신 복구하지 않는다. actual raw 충돌 정지 유지와 미관측 상대 운동 보장 포기를 구분해야 한다.

더 작은 horizon 변경은 s=0 간섭을 없애지 못한다. 1 mm epsilon·reference 동결·이전 age 복사는 원인 은폐다. 벽만 static으로 분류해 상대차 bound를 유지하는 대안은 가능하지만 별도 분류/오인 검증을 요구하여 이번 최소 정책보다 크다. 구현·채택·회귀/주행은 root 범위이며 여기서는 수행하지 않았다.
