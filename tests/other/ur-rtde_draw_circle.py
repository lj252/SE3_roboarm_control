import rtde_control
import rtde_receive
import numpy as np
import math
import time

# ======= 1. 连接机械臂 =======
ROBOT_IP = '192.168.1.11'

try:
    print("正在尝试用标准RTDE协议连接机械臂...")
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("✅ 机械臂连接成功！")
except Exception as e:
    print(f"❌ 连接失败: {e}")
    exit(1)

# ======= 2. 读取当前状态 =======
# 获取初始基座坐标，这样如果只传了圆心和半径，姿态可以保持原样
current_pose = rtde_r.getActualTCPPose()
print("当前基座坐标系下位置:", [round(num, 4) for num in current_pose])

# ======= 3. 定义画圆函数 =======
def draw_circle(center, radius, steps=50, orientation=None, speed=0.05, acc=0.10):
    """
    在 X-Y 平面上画一个圆（Z 高度保持恒定）
    
    参数:
    - center: 圆心坐标 [x, y, z]
    - radius: 圆的半径 (单位: 米)
    - steps: 将圆等分为多少个点，数值越大圆越平滑（默认 50）
    - orientation: 姿态 [rx, ry, rz]（可选，不传则保持机械臂当前姿态）
    - speed: 移动速度 (m/s)
    - acc: 加速度 (m/s²)
    """
    cx, cy, cz = center
    
    # 处理姿态
    if orientation is None:
        # 保持当前的工具姿态不变
        current_pose = rtde_r.getActualTCPPose()
        orient = current_pose[3:]
    else:
        orient = list(orientation)
        
    print(f"开始画圆：圆心 {center}，半径 {radius}...")
    
    # 按步数等分圆周角度 (0 ~ 2π)
    for i in range(steps + 1):  # 多走一步以便闭合成完整的圆
        # 当前点在圆周上的角度
        theta = 2 * math.pi * (i / steps)
        
        # 计算当前点的基座坐标 (在 X-Y 平面画圆)
        target_x = cx + radius * math.cos(theta)
        target_y = cy + radius * math.sin(theta)
        target_z = cz  # Z 轴高度固定不变
        
        # 拼接成位姿列表 [x, y, z, rx, ry, rz]
        target_pose = [target_x, target_y, target_z] + orient
        
        # 执行移动 (阻塞式，到达该点才会继续下一次循环)
        rtde_c.moveL(target_pose, speed, acc)
        
    print("✅ 画圆完成！")

# ======= 4. 调用画圆 =======
# 如果你要画圆，先定义圆心（起点）和半径
# 示例：圆心在 (-0.38, 0.2, 0.2)，半径为 0.06 米的圆
circle_center = [-0.38, 0.2, 0.2]  
circle_radius = 0.06  

# 调用函数 (保持现有姿态，用 60 个点，速度慢一点保证精度)
draw_circle(circle_center, circle_radius, steps=60, speed=0.08, acc=0.10)

# ======= 5. 释放控制权 =======
rtde_c.stopScript()
print("🔌 已断开与机械臂的连接")