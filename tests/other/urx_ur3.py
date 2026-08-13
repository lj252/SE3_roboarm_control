import urx
import time

# ====== 1. 配置 IP ======
ROBOT_IP = '192.168.1.11'

try:
    print(f"正在尝试连接机械臂 {ROBOT_IP}...")
    rob = urx.Robot(ROBOT_IP)  # 连接成功
    
    # ====== 2. 获取当前位置 ======
    # 【核心修改】：urx 获取当前位姿的方法是 getl()，不是 getpose()
    current_pose = rob.getl() 
    print(f"✅ 连接成功！当前基座坐标: {[round(num, 4) for num in current_pose]}")

    # ====== 3. 执行直线移动 ======
    target_pose = [-0.20, -0.40, 0.20, 0, 3.14, 0]
    print(f"👉 准备移动到: {target_pose}")
    
    # 发送直线移动指令
    rob.movex(
        target_pose[0], target_pose[1], target_pose[2], 
        target_pose[3], target_pose[4], target_pose[5], 
        acc=0.1, vel=0.15, wait=True
    )
    
    print("✅ 移动完成！")

except Exception as e:
    print(f"❌ 运行中发生错误: {e}")

finally:
    # 断开连接释放资源
    if 'rob' in locals():
        rob.close()
        print("🔌 已断开与机械臂的连接")