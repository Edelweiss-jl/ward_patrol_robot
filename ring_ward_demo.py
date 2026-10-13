"""Demo 3：环形病区、八间病房、中央护士站与移动推车。"""
# 复用共用仿真入口，通过 ring 选择环形病区布局。
from complex_ward_demo import main

if __name__ == '__main__':
    raise SystemExit(main('ring'))
