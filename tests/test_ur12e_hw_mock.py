#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
向后兼容: 使用 test_ur_hw_mock.py --robot ur12e 代替。
"""

import sys
from pathlib import Path

# 代理到统一 Mock 测试文件
script_dir = Path(__file__).parent
unified_test = script_dir / "test_ur_hw_mock.py"

# 注入 --robot ur12e 参数
if "--robot" not in sys.argv:
    sys.argv.extend(["--robot", "ur12e"])

# 执行统一测试
exec(open(unified_test).read())
