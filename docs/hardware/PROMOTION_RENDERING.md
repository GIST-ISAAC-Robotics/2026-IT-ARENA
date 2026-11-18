# 차량 홍보 이미지 렌더링

야외 서킷 콘셉트는 **Blender 4.5.3 LTS / Cycles**에서 제작했습니다. 배경은 노면·연석·가드레일·관중석을 포함한 편집 가능한 3D 장면입니다. 차량은 Heckler의 홍보용 Fusion v7 조립품에서 가져왔습니다. [게시 이미지](../../README.md#서킷-콘셉트-샷)

## 장면 열기와 렌더링

팀 보관 폴더 `IT ARENA CAD Archive/2026-11-18/rendering/`의 `scenes/`에서 원하는 `.blend`를 엽니다. 노면 맵과 하늘 HDRI는 각 파일 안에 들어 있습니다. 별도 교체나 다운로드 없이 열 수 있습니다.

| 장면 | 용도 |
|---|---|
| `circuit-hero.blend` | README 대표 컷. 전면 대각 구도와 정차 포즈 |
| `circuit-rear.blend` | 후면 대각 구도와 ArUco 마커 |
| `circuit-motion.blend` | 바퀴 회전과 카메라 추적을 포함한 코너 주행 연출 |
| `circuit-background.blend` | 차량을 뺀 야외 서킷. 새 차량이나 소품의 배경으로 사용 |

1. 장면을 열고 수정할 때는 `File → Save As`로 작업 사본을 만듭니다. 카메라 화면은 숫자 키패드 `0`으로 확인합니다.
2. 타임라인에서 **12번 프레임**을 선택하고 `F12`로 렌더링합니다. 렌더 창에서 `Image → Save As`로 PNG를 저장합니다. 게시본 해상도는 **2880 × 1200**입니다.
3. NVIDIA 장비에서 GPU를 쓰려면 `Preferences → System`에서 OptiX 장치를 선택하고, 장면의 Cycles 장치를 `GPU Compute`로 둡니다. 필요하면 Cycles 장치를 CPU로 바꿉니다.

PowerShell에서 Blender가 PATH에 있으면 작업 폴더로 이동한 뒤 아래처럼 실행할 수 있습니다. 배경용 파일이나 다른 컷도 파일명만 바꿔 같은 방식으로 렌더링합니다.

```powershell
New-Item -ItemType Directory -Path ./renders -Force | Out-Null
blender -b ./scenes/circuit-hero.blend -o ./renders/circuit-hero- -F PNG -f 12 -- --cycles-device OPTIX
```

## 다른 홍보 컷 만들기

- 구도는 Outliner의 `Cameras and lighting` 아래 `Telephoto` 카메라를 이동해 잡습니다. 초점은 `Focus at car`에 맞춰져 있습니다. 주행 컷에서 구도를 바꾸려면 카메라 키프레임을 함께 수정합니다.
- 차량은 `Vehicle`, 노면과 트랙 구조물은 `Circuit`에 모았습니다. 새 모델을 넣을 때는 배경 장면을 작업 사본으로 연 뒤 바퀴가 노면에 닿도록 위치와 크기를 맞춥니다. Fusion에서 다시 내보낼 때는 솔리드뿐 아니라 화면에 보이는 표면도 포함합니다. 모터·엔코더 외장과 서보 어댑터에 표면 바디가 있습니다.
- 밝기와 그림자 방향은 `Late afternoon sun`의 강도와 회전으로 조정합니다. 자연광 컷에서는 `Daylight bounce`의 강도가 0입니다. 노면에 인공 광원이 밝은 띠를 만들 수 있으므로 조정 후 아스팔트와 바퀴 아래 그림자를 함께 확인합니다.
- 노면 재질은 Poly Haven의 2K Diffuse·Normal GL·Roughness 맵을 사용합니다. World의 조명은 4K Kloofendal HDRI를 쓰고, 화면에 보이는 하늘은 Blender 노드로 만든 색과 구름을 사용합니다. 보관 폴더의 `textures/`에 원본 맵과 출처를 두었습니다. [Asphalt Track](https://polyhaven.com/a/asphalt_track) · [Kloofendal 43d Clear (Pure Sky)](https://polyhaven.com/a/kloofendal_43d_clear_puresky) · [Poly Haven CC0 라이선스](https://polyhaven.com/license)

보관 폴더의 `source/`에는 Fusion v7에서 추출한 차량 메시와 스크립트가 있습니다. 야외 서킷 장면은 이 입력으로 다시 생성할 수 있고, 공식 직각 코스 컷은 저장소의 공식 월드를 사용하는 별도 스크립트로 다시 렌더링할 수 있습니다. 실행 명령은 보관 폴더의 `README.md`에 있습니다. [공식 코스 검토 기록](../track/OFFICIAL_V2026_11_06_AUDIT.md)
