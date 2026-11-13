#!/usr/bin/env python3
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'공통모듈/isv_shape_navigation'))
from isv_shape_navigation.lidar_preprocess import main
if __name__=='__main__':main()
