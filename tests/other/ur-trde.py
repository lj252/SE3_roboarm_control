import rtde_control
import rtde_receive
import numpy as np

# ======= 1. 连接机械臂 (去掉UR Cap标志，使用默认30004端口) =======
ROBOT_IP = '192.168.1.11'

try:
    print("正在尝试用标准RTDE协议连接机械臂...")
    # 不再传 freq 和 flag
    rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
    # RTDEReceive 也不需要改端口，默认就是 30004
    rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
    print("✅ 机械臂连接成功！")
except Exception as e:
    print(f"❌ 连接失败: {e}")
    exit(1)

# ======= 后续代码（完全不用变） =======
base_pos = rtde_r.getActualTCPPose()
print("当前基座坐标系下位置:", [round(num, 4) for num in base_pos])

# ======= 3. 基础移动测试 =======
base_target = [-0.20, -0.40, 0.20]  
orientation = [0, 3.14, 0]  
target_pose = base_target + orientation

print(f"准备移动到目标位姿: {[round(num, 4) for num in target_pose]}")
rtde_c.moveL(target_pose, 0.1, 0.15) 

# ======= 4. 阵列点位移动函数 =======
def move_to_circle(center, p11, p1n, pn1, pnn, x, y, orientation=None):
    p11, p1n, pn1, pnn, center = map(np.array, [p11, p1n, pn1, pnn, center])
    row_vec = (pn1 - p11) / 13
    col_vec = (p1n - p11) / 13
    offset = (x - 1) * row_vec + (y - 1) * col_vec
    target_base = p11 + offset
    target_base[2] = center[2] 
    
    if orientation is None:
        current_pose = rtde_r.getActualTCPPose()
        target_pose = target_base.tolist() + current_pose[3:]
    else:
        target_pose = target_base.tolist() + list(orientation)
        
    print(f"正在移动到阵列 [{x}, {y}]...")
    rtde_c.moveL(target_pose, 0.1, 0.15)

# ======= 6. 释放控制权 =======
rtde_c.stopScript()