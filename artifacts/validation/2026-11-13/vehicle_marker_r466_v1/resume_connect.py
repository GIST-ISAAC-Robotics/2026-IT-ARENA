"""connect_stage 첫 문자열 개수 오류 이후 미실행 부분만 이어 적용한 영수증용."""
from pathlib import Path
root=Path(__file__).resolve().parent;base=root/'stage/src/arena_autonomy/arena_autonomy'
text=(root/'connect_stage.py').read_text(encoding='utf-8')
definition=text[text.index('def edit('):text.index("edit('local_pursuit.py'")]
tail=text[text.index("edit('g1_g2_node.py','from pathlib import Path") :]
exec(compile(definition+tail,str(root/'connect_stage.py'),'exec'))
