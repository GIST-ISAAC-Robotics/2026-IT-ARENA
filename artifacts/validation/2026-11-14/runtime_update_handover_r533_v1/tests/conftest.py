from pathlib import Path
import sys

HERE=Path(__file__).resolve().parents[1]
ROOT=HERE.parents[3]
sys.path[:0]=[str(HERE/'stage'),str(ROOT/'tests')]
