"""
带边界约束的轨迹优化模块。

本模块实现了一种序列二次规划（SQP）方法，用于解决带有边界约束的轨迹跟踪和优化问题。
它会加载赛道边界坐标，计算曲率和距离因子所需的矩阵，并使用 OSQP 求解器优化轨迹。
"""

import os

import numpy as np
import osqp
import pandas as pd
import scipy.linalg as la
import scipy.sparse as sp

# 屏蔽 matplotlib 的 GUI 需求，必须在导入 pyplot 之前执行
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # pylint: disable=wrong-import-position


def load_track_data(filepath: str = 'track.csv') -> tuple:
    """从 CSV 文件加载赛道边界数据。

    读取给定的边界坐标，并提取出用于轨迹优化的左边界、右边界以及边界间的相对向量。

    Args:
        filepath: 包含赛道数据的 CSV 文件路径。默认为 'track.csv'。

    Returns:
        包含以下内容的元组：
            p: 内边界坐标 (numpy.ndarray)。
            q: 外边界坐标 (numpy.ndarray)。
            v: 从内边界指向外边界的横向量距 (numpy.ndarray)。
    """
    df = pd.read_csv(filepath)
    p = df[['left_border_x', 'left_border_y']].values
    q = df[['right_border_x', 'right_border_y']].values
    v = q - p
    return p, q, v


def build_difference_matrix(num_points: int) -> sp.csc_matrix:
    """构建用于闭环赛道的一阶差分矩阵。

    Args:
        num_points: 赛道参考点的总数量。

    Returns:
        表示闭环一阶差分的稀疏矩阵 (N x N)。
    """
    diag = np.ones(num_points) * (-1)
    off_diag = np.ones(num_points - 1)
    diff_matrix = sp.diags([diag, off_diag], [0, 1],
                           shape=(num_points, num_points)).tolil()
    diff_matrix[num_points - 1, 0] = 1
    return diff_matrix.tocsc()


def calculate_distance_factor(diff_matrix: sp.csc_matrix, p: np.ndarray,
                              v: np.ndarray) -> tuple:
    """计算距离因子的 Hessian 矩阵和一次项。

    Args:
        diff_matrix: 格式化好的一阶差分矩阵 (N x N)。
        p: 内边界坐标数组。
        v: 从内边界指向外边界的横向方向向量。

    Returns:
        包含以下内容的元组：
            hs_matrix: 距离代价的 Hessian 二阶项矩阵。
            f_s: 距离代价的线性一次项向量。
    """
    px = p[:, 0]
    py = p[:, 1]

    v_x = sp.diags(v[:, 0])
    v_y = sp.diags(v[:, 1])

    ata_matrix = diff_matrix.T @ diff_matrix

    hs_x = v_x.T @ ata_matrix @ v_x
    fs_x = v_x.T @ ata_matrix @ px

    hs_y = v_y.T @ ata_matrix @ v_y
    fs_y = v_y.T @ ata_matrix @ py

    hs_matrix = 2 * (hs_x + hs_y)
    f_s = 2 * (fs_x + fs_y)

    return hs_matrix, f_s


def calculate_derivative_matrices(r_pts: np.ndarray) -> tuple:
    """计算曲率公式里的权重矩阵（涉及一阶导数计算）。

    Args:
        r_pts: 当前参考轨迹的坐标点数组。

    Returns:
        包含三个核心稀疏对角权重矩阵的元组：
            t_xx, t_yy, t_xy。
    """
    rx = r_pts[:, 0]
    ry = r_pts[:, 1]

    # 中心差分近似导数
    rx_prime_unnorm = (np.roll(rx, -1) - np.roll(rx, 1)) / 2.0
    ry_prime_unnorm = (np.roll(ry, -1) - np.roll(ry, 1)) / 2.0

    # 为了避免数值上除以 0 导致溢出，加入微小偏置项
    denominator = (rx_prime_unnorm**2 + ry_prime_unnorm**2)**3 + 1e-8

    t_xx_diag = (ry_prime_unnorm**2) / denominator
    t_yy_diag = (rx_prime_unnorm**2) / denominator
    t_xy_diag = (-2 * rx_prime_unnorm * ry_prime_unnorm) / denominator

    t_xx = sp.diags(t_xx_diag)
    t_yy = sp.diags(t_yy_diag)
    t_xy = sp.diags(t_xy_diag)

    return t_xx, t_yy, t_xy


def calculate_m_matrix(num_points: int) -> sp.csc_matrix:
    """构造三次样条（等距简化版）的二阶导数映射矩阵 M。

    Args:
        num_points: 轨迹上的参考点总数量。

    Returns:
        表示三次样条二阶导数映射的稀疏矩阵。
    """
    b_mat = sp.diags([1, 4, 1], [-1, 0, 1],
                     shape=(num_points, num_points), dtype=float).tolil()
    c_mat = sp.diags([6, -12, 6], [-1, 0, 1],
                     shape=(num_points, num_points), dtype=float).tolil()

    # 处理首尾衔接，维持赛道物理闭环形状
    b_mat[0, num_points - 1] = 1
    b_mat[num_points - 1, 0] = 1
    c_mat[0, num_points - 1] = 6
    c_mat[num_points - 1, 0] = 6

    b_dense = b_mat.toarray()
    c_dense = c_mat.toarray()

    b_inv = la.inv(b_dense)
    m_dense = b_inv @ c_dense

    # 截断极小值消除浮点误差噪声，维持其带状稀疏拓扑性
    m_dense[np.abs(m_dense) < 1e-4] = 0.0
    m_matrix = sp.csc_matrix(m_dense)

    return m_matrix


def calculate_curvature_factor(
    m_matrix: sp.csc_matrix, t_xx: sp.csc_matrix, t_yy: sp.csc_matrix,
    t_xy: sp.csc_matrix, p: np.ndarray, v: np.ndarray
) -> tuple:
    # pylint: disable=too-many-arguments, too-many-positional-arguments, too-many-locals
    """计算曲率因子的 Hessian 二阶矩阵和一次惩罚项。

    Args:
        m_matrix: 二阶导常数映射矩阵 M。
        t_xx:  X 轴方向一阶导曲率权重矩阵。
        t_yy:  Y 轴方向一阶导曲率权重矩阵。
        t_xy: 交叉方向一阶导曲率权重矩阵。
        p: 内边界坐标数组。
        v: 内至外的横向几何向量数组。

    Returns:
        一个包含 (hk_matrix, f_k) 的元组，代表这轮算出的曲率惩罚权重。
    """
    px = p[:, 0]
    py = p[:, 1]

    v_x = sp.diags(v[:, 0])
    v_y = sp.diags(v[:, 1])

    mt_txx_m = m_matrix.T @ t_xx @ m_matrix
    mt_tyy_m = m_matrix.T @ t_yy @ m_matrix
    mt_txy_m = m_matrix.T @ t_xy @ m_matrix

    term1 = v_x.T @ mt_txx_m @ v_x
    term_cross = v_x.T @ mt_txy_m @ v_y
    # 合并交叉项保持对称性，因为这是标准的二次型计算
    term_cross_sym = term_cross + term_cross.T
    term2 = v_y.T @ mt_tyy_m @ v_y

    hk_matrix = 2 * (term1 + term2) + term_cross_sym

    fk1 = v_x.T @ mt_txx_m @ px
    fk_cross1 = v_y.T @ mt_txy_m @ px
    fk_cross2 = v_x.T @ mt_txy_m @ py
    fk2 = v_y.T @ mt_tyy_m @ py

    f_k = 2 * (fk1 + fk2) + fk_cross1 + fk_cross2
    return hk_matrix, f_k


def calculate_boundary_normals(bound_pts: np.ndarray, v: np.ndarray) -> np.ndarray:
    """计算边界点集合上统一全部朝内侧面相的单位法向量。

    Args:
        bound_pts: 边界点坐标集合 (N x 2)。
        v: 指向外边界的方向向量序列 (N x 2)。

    Returns:
        确保指着赛道内部物理腹地的单位法向量集 (N x 2)。
    """
    dp = (np.roll(bound_pts, -1, axis=0) - np.roll(bound_pts, 1, axis=0)) / 2.0
    dp_norm = np.linalg.norm(dp, axis=1, keepdims=True)
    tangent = dp / (dp_norm + 1e-8)

    n1 = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    n2 = np.column_stack([tangent[:, 1], -tangent[:, 0]])

    dot1 = np.sum(n1 * v, axis=1)
    normals = np.where(dot1[:, np.newaxis] > 0, n1, n2)

    return normals


def calculate_wv_per_point(r_pts: np.ndarray, v: np.ndarray, vehicle_length: float,
                           vehicle_width: float) -> np.ndarray:
    """计算出小车在当前赛道点基于横摆角所需的横向物理缓冲半宽。

    Args:
        r_pts: 当前参考轨迹点。
        v: 横向内指向外的距离跨域向量。
        vehicle_length: 车辆底盘总占矩向长。
        vehicle_width: 车辆本身左右两边占总宽。

    Returns:
        每个离散参考点在此动态角度下所需的投影保护带宽度数组。
    """
    d = np.roll(r_pts, -1, axis=0) - r_pts
    d_norm = np.linalg.norm(d, axis=1)
    v_norm = np.linalg.norm(v, axis=1)

    d_norm_safe = np.where(d_norm < 1e-8, 1e-8, d_norm)
    v_norm_safe = np.where(v_norm < 1e-8, 1e-8, v_norm)

    dot_vd = np.sum(v * d, axis=1)
    cos_val = dot_vd / (v_norm_safe * d_norm_safe)
    cos_val = np.clip(cos_val, -1.0, 1.0)

    angle_vd = np.arccos(cos_val)
    angle_vehicle = np.arctan2(vehicle_width, vehicle_length)

    diagonal = np.sqrt(vehicle_length**2 + vehicle_width**2) / 2.0
    w_v = diagonal * np.abs(np.cos(angle_vd - angle_vehicle))

    return w_v


def optimize_trajectory(
    p: np.ndarray, v: np.ndarray, vehicle_length: float, vehicle_width: float,
    safety_margin: float, epsilon: float = 0.0, max_iter: int = 6,
    gamma_normal: float = 0.5, gamma_inaccurate: float = 0.1
) -> tuple:
    # pylint: disable=too-many-arguments, too-many-positional-arguments, too-many-locals, too-many-statements
    """跨越定义的闭环边界条件执行 SQP（序列二次规划）轨迹优化核心控制算法。

    Args:
        p: 内边界原始坐标序列。
        v: 内向外拓展的跨宽约束间隙向量。
        vehicle_length: 设定车辆物理长度。
        vehicle_width: 设定车辆物理宽度。
        safety_margin: 最保守绝对留白安全间隙宽度。
        epsilon: 距离保持紧贴项和最小大圆弧拉扯项的权衡混合比例。
        max_iter: SQP 二次规划内含多步线性最大试探步骤数量。
        gamma_normal:  模型稳定收敛时的正常阻尼放松步长。
        gamma_inaccurate: OSQP返回有噪音未稳局部的极小适应探步长。

    Returns:
        返回一对最终推演极化的 (最终物理参考路线坐标集合, 所对应的 alpha 取值权重记录)。
    """
    num_points = len(p)
    m_matrix = calculate_m_matrix(num_points)
    a_diff = build_difference_matrix(num_points)

    hs_matrix, f_s = calculate_distance_factor(a_diff, p, v)
    alpha_ref = np.zeros(num_points)

    q_bound = p + v
    n_i = calculate_boundary_normals(p, v)
    n_o = calculate_boundary_normals(q_bound, v)

    v_dot_ni = np.sum(n_i * v, axis=1)
    v_dot_no = np.sum(n_o * v, axis=1)

    v_dot_ni = np.maximum(v_dot_ni, 1e-4)
    v_dot_no = np.maximum(v_dot_no, 1e-4)

    a_constr = sp.eye(num_points).tocsc()
    reg = 1e-6 * sp.eye(num_points).tocsc()

    for iteration in range(max_iter):
        r_current = p + v * alpha_ref[:, np.newaxis]
        wv_array = calculate_wv_per_point(r_current, v, vehicle_length,
                                          vehicle_width)

        alpha_min = (wv_array + safety_margin) / v_dot_ni
        alpha_max = 1.0 - (wv_array + safety_margin) / v_dot_no
        alpha_max = np.maximum(alpha_max, alpha_min + 1e-3)

        t_xx, t_yy, t_xy = calculate_derivative_matrices(r_current)
        hk_matrix, f_k = calculate_curvature_factor(
            m_matrix, t_xx, t_yy, t_xy, p, v)

        # 强制将本次的海森矩阵对称化并加微扰，保障其依然服从凸二次规划需要的半正定（PSD）需求
        hk_matrix = (hk_matrix + hk_matrix.T) / 2.0
        hk_matrix = hk_matrix + sp.diags(np.ones(num_points) * 1e-4)

        if iteration == 0:
            hs_matrix = (hs_matrix + hs_matrix.T) / 2.0
            hs_matrix = hs_matrix + sp.diags(np.ones(num_points) * 1e-4)

        p_mat = sp.csc_matrix(hk_matrix + epsilon * hs_matrix + reg)
        q_vec = f_k + epsilon * f_s

        prob = osqp.OSQP()
        prob.setup(P=p_mat, q=q_vec, A=a_constr, l=alpha_min, u=alpha_max,
                   verbose=False, max_iter=20000)

        res = prob.solve()
        alpha_new = res.x

        if res.info.status == 'solved':
            gamma_adaptive = gamma_normal
        elif res.info.status == 'solved inaccurate':
            print(f"迭代 {iteration+1:2d}: 状态 '{res.info.status}', "
                  "正在启动收敛阻尼衰减保护机制。")
            gamma_adaptive = gamma_inaccurate
        else:
            print(f"迭代 {iteration+1:2d}: 出现致死异常状态 '{res.info.status}', "
                  "立刻终止本轮 SQP 迭代以防轨道被错误污染。")
            break

        if alpha_new is None:
            break

        diff = np.max(np.abs(alpha_new - alpha_ref))
        j_k_val = 0.5 * alpha_new.T @ hk_matrix @ alpha_new + f_k.T @ alpha_new
        j_s_val = 0.5 * alpha_new.T @ hs_matrix @ alpha_new + f_s.T @ alpha_new
        j_total = j_k_val + epsilon * j_s_val

        print(f"迭代 {iteration+1:2d}/{max_iter} | max|Δα|: {diff:.6f} | "
              f"J_k: {j_k_val:.2e} | J_s: {j_s_val:.2e} | "
              f"J_total: {j_total:.2e}")

        # 使用 SQP 放松与阻尼（Damping）步骤进行缓冲
        alpha_ref = ((1 - gamma_adaptive) * alpha_ref +
                     gamma_adaptive * alpha_new)

        if diff < 1e-3:
            print("二次规划（SQP）算法已经完全收敛完毕！")
            break

    r_optimal = p + v * alpha_ref[:, np.newaxis]
    return r_optimal, alpha_ref


def main():
    # pylint: disable=too-many-locals, too-many-statements
    """执行轨迹系统优化的主控和测试生成绘图入口函数。"""
    vehicle_length = 4.7
    vehicle_width = 2.0
    safety_margin = 0.5

    print("开始加载赛道几何先验数据...")
    if not os.path.exists('stadium_bounds.csv'):
        print("警告: 缺失 'stadium_bounds.csv' 基准文件。请确认它是否存在当前目录中。")
        return

    p, q_bound, v = load_track_data('stadium_bounds.csv')

    gamma_norm = 0.5
    gamma_inacc = 0.1

    print("\n--- 正在计算极限性能最小曲率内收跑法 (eps = 0.0) ---")
    r_opt_k, alpha_k = optimize_trajectory(
        p, v, vehicle_length, vehicle_width, safety_margin, epsilon=0.0,
        max_iter=50, gamma_normal=gamma_norm, gamma_inaccurate=gamma_inacc
    )

    print("\n--- 正在计算兼顾赛道中心的求稳保底跑法 (eps = 1000.0) ---")
    r_opt_s, alpha_s = optimize_trajectory(
        p, v, vehicle_length, vehicle_width, safety_margin, epsilon=1000.0,
        max_iter=50, gamma_normal=gamma_norm, gamma_inaccurate=gamma_inacc
    )

    # pylint: disable=unused-variable
    fig, axes = plt.subplots(2, 2, figsize=(24, 20))

    p_plot = np.vstack((p, p[0]))
    q_plot = np.vstack((q_bound, q_bound[0]))
    r_k_plot = np.vstack((r_opt_k, r_opt_k[0]))
    r_s_plot = np.vstack((r_opt_s, r_opt_s[0]))

    ax1 = axes[0, 0]
    ax1.plot(p_plot[:, 0], p_plot[:, 1], 'k-', linewidth=1.5,
             label='左侧实测原始墙 (p)')
    ax1.plot(q_plot[:, 0], q_plot[:, 1], 'k-', linewidth=1.5, alpha=0.6,
             label='右侧推算投影墙 (q)')
    ax1.plot(r_k_plot[:, 0], r_k_plot[:, 1], 'r-', linewidth=2.5,
             label='最小曲率 (eps=0)')
    ax1.plot(r_s_plot[:, 0], r_s_plot[:, 1], 'b--', linewidth=2.5,
             label='平衡中心 (eps=1000)')
    ax1.set_title("整圈赛道全域鸟瞰图", fontsize=16)
    ax1.set_xlabel("X向局部坐标 (m)")
    ax1.set_ylabel("Y向局部坐标 (m)")
    ax1.legend(fontsize=12)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)

    ax2 = axes[0, 1]
    ax2.plot(p_plot[:, 0], p_plot[:, 1], 'k-', linewidth=1.5)
    ax2.plot(q_plot[:, 0], q_plot[:, 1], 'k-', linewidth=1.5, alpha=0.6)
    ax2.plot(r_k_plot[:, 0], r_k_plot[:, 1], 'r-', linewidth=3,
             label='最小曲率法')
    ax2.plot(r_s_plot[:, 0], r_s_plot[:, 1], 'b--', linewidth=3,
             label='平衡走线法')
    ax2.set_title("上左极限弯道细节放大", fontsize=16)
    ax2.legend(fontsize=12)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)

    ax3 = axes[1, 0]
    ax3.plot(p_plot[:, 0], p_plot[:, 1], 'k-', linewidth=1.5)
    ax3.plot(q_plot[:, 0], q_plot[:, 1], 'k-', linewidth=1.5, alpha=0.6)
    ax3.plot(r_k_plot[:, 0], r_k_plot[:, 1], 'r-', linewidth=3,
             label='最小曲率法')
    ax3.plot(r_s_plot[:, 0], r_s_plot[:, 1], 'b--', linewidth=3,
             label='平衡走线法')
    ax3.set_title("右下复杂弯道组合放大", fontsize=16)
    ax3.legend(fontsize=12)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)

    ax4 = axes[1, 1]
    indices = np.arange(len(alpha_k))
    ax4.plot(indices, alpha_k, 'r-', linewidth=1.5, label='最小曲法 Alpha 系数',
             alpha=0.8)
    ax4.plot(indices, alpha_s, 'b-', linewidth=1.5, label='平衡走线 Alpha 系数',
             alpha=0.8)
    ax4.axhline(y=0.5, color='gray', linestyle=':', alpha=0.5,
                label='安全理想赛道绝对中心 (alpha=0.5)')
    ax4.set_title("控制偏移偏转系数 Alpha 历遍分布", fontsize=16)
    ax4.set_xlabel("闭环轨迹控制点索引号")
    ax4.set_ylabel("Alpha 系数推值")
    ax4.legend(fontsize=12)
    ax4.grid(True, alpha=0.3)
    ax4.set_ylim(-0.05, 1.05)

    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=150, bbox_inches='tight')
    print("\n大图边界与路线分析对比图表已成功保存: 'trajectory_comparison.png'")

    print(f"\n曲率追求(eps=0)数学结果统计: 极限最小={alpha_k.min():.4f}, "
          f"极限靠外={alpha_k.max():.4f}, 稳态平均偏离={alpha_k.mean():.4f}")
    print(f"求稳算法(eps=1000)数学结果统计: 极限最小={alpha_s.min():.4f}, "
          f"极限靠外={alpha_s.max():.4f}, 稳态平均偏离={alpha_s.mean():.4f}")

    df_optimal = pd.DataFrame(r_opt_k, columns=['x', 'y'])
    df_optimal.to_csv('stadium_optimal_trajectory.csv', index=False)
    print("\n[✔] 最终仅面向下位机的单线二维点距被提纯且导出至: "
          "'stadium_optimal_trajectory.csv'.")


if __name__ == '__main__':
    main()
