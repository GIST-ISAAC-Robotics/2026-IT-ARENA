"""Build a self-contained Korean hardware handoff page from the C1 experiment."""
import csv
import json
from pathlib import Path
from study_c1_mount import OUT


def main():
    data=json.loads((OUT/"results.json").read_text(encoding="utf-8"))
    data["stack"]=json.loads((OUT/"stack.json").read_text(encoding="utf-8"))
    with (OUT/"tolerance.csv").open(encoding="utf-8") as f:
        data["tolerance"]=list(csv.DictReader(f))
    with (OUT/"plate_screen.csv").open(encoding="utf-8") as f:
        data["plate"]=list(csv.DictReader(f))
    template=Path(__file__).with_name("c1_mount_viewer.html.in").read_text(encoding="utf-8")
    stack=Path(__file__).with_name("c1_stack_figures.html.in").read_text(encoding="utf-8")
    for key in ("plan","side"):
        svg=Path(__file__).with_name("c1_pdb_"+key+".svg").read_text(encoding="utf-8")
        stack=stack.replace('<div id="board'+key+'"></div>',svg)
    template=template.replace('<section id="printplate">',stack+'\n<section id="printplate">')
    (OUT/"index.html").write_text(template.replace("__STUDY_JSON__",json.dumps(data,ensure_ascii=False,separators=(",",":"))),encoding="utf-8")


if __name__=="__main__":
    main()
