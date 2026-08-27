"""pytest 导入路径配置。

生产代码使用顶层 ``linkerhand.*`` 导入，而源码树中 ``linkerhand`` 包嵌套在
``linkerhand_retarget/linkerhand_retarget/`` 下。本 conftest 在收集前把包根
（提供 ``linkerhand_retarget``）与内层包目录（提供顶层 ``linkerhand``）加入
``sys.path``，使仓库根目录下 ``python3 -m pytest src/linkerhand_retarget/tests/unit -q``
无需额外设置 PYTHONPATH 即可运行。
"""

import sys
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parents[1]  # src/linkerhand_retarget

for _dir in (str(_PKG_ROOT), str(_PKG_ROOT / "linkerhand_retarget")):
    if _dir not in sys.path:
        sys.path.insert(0, _dir)
