"""Demo 2：六病房、十二床位、中央走廊的动态巡查。"""
# 复用共用仿真入口，通过 corridor 选择中央走廊布局。
from complex_ward_demo import main

if __name__ == '__main__':
    raise SystemExit(main('corridor'))
