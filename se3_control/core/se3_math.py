"""
SE(3) 数学工具 — 纯 NumPy，零外部依赖
========================================

提供 SE(3) 几何控制所需的数学原语：
  - 𝔰𝔬(3) 操作: hat_map, vee_map
  - SE(3) 变换: adjoint_g_ed, adjoint_g_ed_dual, adjoint_g_ed_deriv
  - 旋转矩阵: rotmat_x, rotmat_slerp, rpy_to_rotmat, rotmat_to_xyz_euler

所有函数为纯 NumPy 实现，无外部依赖（如 Pinocchio / SymPy）。
"""

import numpy as np


# ====================================================================
# 𝔰𝔬(3) 操作
# ====================================================================

def hat_map(w):
    """ℝ³ → 𝔰𝔬(3) 反对称矩阵 (hat map).

    :param w: ndarray (3,) 或 (3,1) — 旋转轴角速度
    :returns: ndarray (3,3) — 反对称矩阵
    """
    w = w.reshape((-1,))
    return np.array([
        [0,    -w[2],  w[1]],
        [w[2],  0,    -w[0]],
        [-w[1], w[0],  0],
    ])


def vee_map(R):
    """𝔰𝔬(3) → ℝ³ 逆映射 (vee map).

    :param R: ndarray (3,3) — 反对称矩阵
    :returns: ndarray (3,1) — 旋转轴角速度
    """
    return np.array([-R[1, 2], R[0, 2], -R[0, 1]]).reshape((-1, 1))


# ====================================================================
# SE(3) 变换
# ====================================================================

def adjoint_g_ed(g_ed):
    """SE(3) 伴随变换 Ad_{g_ed} ∈ ℝ⁶ˣ⁶.

    将物体坐标系 twist 从 g_ed 处变换到当前坐标系:
      Ad_{g_ed} = [[R,  p̂R],
                    [0,  R   ]]

    :param g_ed: ndarray (4,4) — SE(3) 齐次矩阵
    :returns: ndarray (6,6) — 伴随变换矩阵
    """
    p = g_ed[:3, 3]
    R = g_ed[:3, :3]
    p_hat = hat_map(p)
    adj = np.zeros((6, 6))
    adj[:3, :3] = R
    adj[3:, 3:] = R
    adj[:3, 3:] = p_hat @ R
    return adj


def adjoint_g_ed_dual(g_ed):
    """SE(3) 对偶伴随变换 Ad_{g_ed}^{-T}.

    :param g_ed: ndarray (4,4) — SE(3) 齐次矩阵
    :returns: ndarray (6,6) — 对偶伴随变换矩阵
    """
    mat = adjoint_g_ed(np.linalg.inv(g_ed))
    return mat.T


def adjoint_g_ed_deriv(g, gd, v, w, vd, wd):
    """伴随变换的时间导数 d/dt(Ad_{g_ed}).

    :param g:  ndarray (4,4) — 当前位姿
    :param gd: ndarray (4,4) — 期望位姿
    :param v:  ndarray (3,) 或 (3,1) — 当前线速度
    :param w:  ndarray (3,) 或 (3,1) — 当前角速度
    :param vd: ndarray (3,) 或 (3,1) — 期望线速度
    :param wd: ndarray (3,) 或 (3,1) — 期望角速度
    :returns: ndarray (6,6) — d/dt(Ad_{g_ed})
    """
    v = v.reshape((-1, 1))
    w = w.reshape((-1, 1))
    vd = vd.reshape((-1, 1))
    wd = wd.reshape((-1, 1))

    g_ed = np.linalg.inv(g) @ gd
    p_ed = g_ed[:3, 3]
    R_ed = g_ed[:3, :3]

    mat = np.zeros((6, 6))

    dR_ed = hat_map(w) @ R_ed - R_ed @ hat_map(wd)
    dp_ed = -v - hat_map(w) @ p_ed + R_ed @ vd

    mat[:3, :3] = dR_ed
    mat[:3, 3:] = hat_map(p_ed) @ dR_ed + hat_map(dp_ed) @ R_ed
    mat[3:, 3:] = dR_ed

    return mat


# ====================================================================
# 旋转矩阵基本操作
# ====================================================================

def rotmat_x(th):
    """绕 X 轴旋转 th 弧度的基本旋转矩阵.

    :param th: float — 旋转角度 (rad)
    :returns: ndarray (3,3)
    """
    c, s = np.cos(th), np.sin(th)
    return np.array([
        [1, 0, 0],
        [0, c, -s],
        [0, s,  c],
    ])


def rotmat_slerp(R1, R2, alpha):
    """SO(3) 球面线性插值 (SLERP): 从 R1 到 R2, 因子 alpha ∈ [0,1].

    :param R1: ndarray (3,3) — 起始旋转矩阵
    :param R2: ndarray (3,3) — 终止旋转矩阵
    :param alpha: float — 插值因子, 0 → R1, 1 → R2
    :returns: ndarray (3,3) — 插值后的旋转矩阵
    """
    R_rel = R1.T @ R2
    cos_theta = (np.trace(R_rel) - 1.0) / 2.0
    cos_theta = np.clip(cos_theta, -1.0, 1.0)
    theta = np.arccos(cos_theta)

    if theta < 1e-10:
        return R2.copy()

    sin_theta = np.sin(theta)
    omega = np.array([
        R_rel[2, 1] - R_rel[1, 2],
        R_rel[0, 2] - R_rel[2, 0],
        R_rel[1, 0] - R_rel[0, 1],
    ])
    omega_norm = np.linalg.norm(omega)
    if omega_norm < 1e-10:
        # 180° 旋转: 线性插值 + SVD 重正交化
        R_lerp = (1 - alpha) * R1 + alpha * R2
        U, _, Vt = np.linalg.svd(R_lerp)
        return U @ Vt

    axis = omega / (2.0 * sin_theta)
    n_hat = np.array([
        [0,    -axis[2],  axis[1]],
        [axis[2],  0,    -axis[0]],
        [-axis[1], axis[0],  0],
    ])
    cos_at = np.cos(alpha * theta)
    sin_at = np.sin(alpha * theta)
    R_alpha = (np.eye(3)
               + sin_at * n_hat
               + (1 - cos_at) * (n_hat @ n_hat))
    return R1 @ R_alpha


# ====================================================================
# URDF / MuJoCo 兼容工具
# ====================================================================

def rpy_to_rotmat(rpy):
    """URDF RPY (roll-pitch-yaw) → 3×3 旋转矩阵.

    URDF 使用 ZYX 欧拉角 (绕**固定轴** X→Y→Z):
      R = Rz(yaw) @ Ry(pitch) @ Rx(roll)

    :param rpy: ndarray (3,) — [roll, pitch, yaw] (弧度)
    :returns: ndarray (3,3)
    """
    roll, pitch, yaw = float(rpy[0]), float(rpy[1]), float(rpy[2])
    cx, sx = np.cos(roll), np.sin(roll)
    cy, sy = np.cos(pitch), np.sin(pitch)
    cz, sz = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rotmat_to_xyz_euler(R):
    """旋转矩阵 → XYZ 顺序欧拉角 (MuJoCo eulerseq='xyz' 约定).

    MuJoCo 的 eulerseq='xyz' 对应 R = Rx(rx) @ Ry(ry) @ Rz(rz).

    :param R: ndarray (3,3) — 旋转矩阵
    :returns: ndarray (3,) — [rx, ry, rz] (弧度)
    """
    ry = np.arcsin(np.clip(R[0, 2], -1.0, 1.0))
    if abs(R[0, 2]) < 0.999999:
        rx = np.arctan2(-R[1, 2], R[2, 2])
        rz = np.arctan2(-R[0, 1], R[0, 0])
    else:
        # 万向锁: cos(ry) ≈ 0, 设 rz=0 后求 rx
        rx = np.arctan2(R[2, 1], R[1, 1])
        rz = 0.0
    return np.array([rx, ry, rz])


# ====================================================================
# 自检
# ====================================================================

if __name__ == '__main__':
    np.set_printoptions(precision=4, suppress=True)

    # 测试 hat/vee 互逆
    w = np.array([0.3, -0.5, 0.7])
    w_hat = hat_map(w)
    w_back = vee_map(w_hat).ravel()
    assert np.allclose(w, w_back), "hat/vee 互逆失败"
    print("[hat/vee] ✅ 互逆")

    # 测试 adjoint 乘法性质: Ad(g1) @ Ad(g2) ≈ Ad(g1 @ g2)
    g1 = np.eye(4); g1[:3, 3] = [0.1, 0.2, 0.3]
    g2 = np.eye(4); g2[:3, :3] = rotmat_x(0.5); g2[:3, 3] = [0.0, 0.1, 0.0]
    Ad1 = adjoint_g_ed(g1)
    Ad2 = adjoint_g_ed(g2)
    Ad12 = adjoint_g_ed(g1 @ g2)
    assert np.allclose(Ad1 @ Ad2, Ad12), "adjoint 乘法性质失败"
    print("[adjoint] ✅ 乘法性质")

    # 测试 slerp 端点
    R_a = rotmat_x(0.3)
    R_b = rotmat_x(1.5)
    assert np.allclose(rotmat_slerp(R_a, R_b, 0.0), R_a)
    assert np.allclose(rotmat_slerp(R_a, R_b, 1.0), R_b)
    print("[slerp]   ✅ 端点正确")

    # 测试 rpy <-> rotmat 合法旋转矩阵
    rpy = np.array([0.1, -0.2, 0.3])
    R_rpy = rpy_to_rotmat(rpy)
    assert np.allclose(np.linalg.det(R_rpy), 1.0), "det(R) ≠ 1"
    assert np.allclose(R_rpy.T @ R_rpy, np.eye(3)), "RᵀR ≠ I"
    # rotmat_to_xyz_euler 应满足 R = Rx(rx) @ Ry(ry) @ Rz(rz)
    xyz = rotmat_to_xyz_euler(R_rpy)
    R_rebuilt = (rotmat_x(xyz[0])
                 @ rotmat_x(0)  # placeholder, need proper Ry/Rz
                 @ rotmat_x(0))
    # 更严谨: 用 MuJoCo 约定重建
    cx, sx = np.cos(xyz[0]), np.sin(xyz[0])
    cy, sy = np.cos(xyz[1]), np.sin(xyz[1])
    cz, sz = np.cos(xyz[2]), np.sin(xyz[2])
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    R_built = Rx @ Ry @ Rz
    assert np.allclose(R_rpy, R_built, atol=1e-10), "XYZ euler 重建失败"
    print(f"[rpy]    ✅ URDF RPY → R → XYZ euler: {np.round(xyz, 4)}")

    print("\n所有自检通过 ✅")
