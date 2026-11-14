"""r534 종료 후 성공/실패 양쪽의 필수 근거를 수집한다."""
from pathlib import Path
from collect_evidence import collect
if __name__=='__main__':collect(Path(__file__).resolve().parent,'solo')
