"""pytest 收集配置: 忽略独立运行的硬件 Mock 测试脚本.

tests/test_ur_hw_mock.py 与 tests/test_ur12e_hw_mock.py 是独立可执行脚本,
通过 `python tests/test_ur_hw_mock.py --robot ur12e` 直接运行,
模块顶层调用 sys.exit 输出汇总, 不参与 pytest 收集.
(在 ur_rtde 未安装的环境中直接以脚本运行, 避免 pytest 收集期 INTERNALERROR.)
"""
collect_ignore = ["test_ur_hw_mock.py", "test_ur12e_hw_mock.py"]


def pytest_configure(config):
    """注册自定义 mark (完整 MuJoCo 仿真回归, 运行较慢)."""
    config.addinivalue_line(
        "markers",
        "simulation: 运行完整 MuJoCo 仿真的回归测试 (约数秒).")
