"""실제 Gazebo 정지 프레임의 비교판. 합성 장면이나 재생성 이미지는 만들지 않는다."""
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont

HERE=Path(__file__).resolve().parent
font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',22)
small=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',17)


def sheet(name,items,columns=2):
    width=640;header=58;body=360;rows=(len(items)+columns-1)//columns
    result=Image.new('RGB',(columns*width,rows*(header+body)),(245,247,250));draw=ImageDraw.Draw(result)
    for i,(title,subtitle,path) in enumerate(items):
        x=(i%columns)*width;y=(i//columns)*(header+body)
        draw.text((x+10,y+5),title,fill=(25,30,35),font=font)
        draw.text((x+10,y+32),subtitle,fill=(60,65,70),font=small)
        image=Image.open(path).convert('RGB').resize((width,body),Image.Resampling.LANCZOS)
        result.paste(image,(x,y+header))
    result.save(HERE/name)


source=HERE/'render_newhead_v3'
for level,title in [('empty','다른 차량 없음'),('same224','다른 차량 높이 22.4 cm')]:
    sheet('grid_'+level+'.png',[(f'그리드 {i} · {title}','카메라 광축 19.84 cm / 신호등 −9 mm 비교안',source/f'{level}_slot{i}_green.png') for i in range(6)])
sheet('occlusion_comparison.png',[(f'맨 뒤 그리드 3 · {text}','동일한 카메라·코스·시점, 앞차 높이만 변경',source/f'{level}_slot3_red4.png')
                                for level,text in [('empty','빈 배치'),('same224','앞차 22.4 cm'),('partial230','앞차 23 cm'),('tall350','앞차 35 cm')]])
if (HERE/'render_plus40/partial250_slot3_green.png').exists():
    sheet('mount_comparison.png',[
        ('광축 19.84 cm / 앞차 22.4 cm','불빛은 보이나 마커 아래쪽은 가림',source/'same224_slot3_green.png'),
        ('광축 21.84 cm / 앞차 22.4 cm','카메라를 20 mm 높인 진단용 비교안',HERE/'render_plus20/same224_slot3_green.png'),
        ('광축 19.84 cm / 앞차 24 cm','현재 높이의 부분 가림 실패 예',source/'partial240_slot3_green.png'),
        ('광축 23.84 cm / 앞차 25 cm','카메라를 40 mm 높인 진단용 비교안',HERE/'render_plus40/partial250_slot3_green.png')])
print('comparison figures written')
