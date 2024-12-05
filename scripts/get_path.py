'''

'''
from pathlib import Path

def GetRootDir():
    root_dir = Path(__file__).resolve().parent.parent
    return root_dir