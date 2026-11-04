"""Unified live product entry. No bags, videos, raw data or automatic map saves."""
import os,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'python'))
from online_session import main
if __name__=='__main__':
    sys.argv+=['--engine','native-v2']
    main()
