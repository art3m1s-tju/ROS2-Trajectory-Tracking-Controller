"""
生成操场（赛道）地图及边界数据的模块。

本模块利用 OpenCV 生成一个典型的“操场型”赛道地图（包含直线和圆弧），
并推导出对应的内、外边界坐标，保存为 CSV 文件供轨迹优化算法使用。
"""

import cv2  # pylint: disable=import-error
import numpy as np
import pandas as pd
import yaml  # pylint: disable=import-error


def save_map_yaml(map_base_name: str, resolution: float, width_px: int, height_px: int):
    """生成并保存 ROS 兼容的 YAML 配置文件。"""
    # 设定地图原点：OpenCV 左上角为 (0,0)，ROS 通常以地图中心或左下角为原点
    origin_x = - (width_px / 2.0) * resolution
    origin_y = - (height_px / 2.0) * resolution

    yaml_content = {
        'image': f'{map_base_name}.png',
        'resolution': float(resolution),
        'origin': [float(origin_x), float(origin_y), 0.0],
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
        'negate': 0
    }

    yaml_path = f'{map_base_name}.yaml'
    with open(yaml_path, 'w', encoding='utf-8') as f:
        yaml.dump(yaml_content, f, default_flow_style=False)

    print(f"YAML 配置文件已保存至: {yaml_path}")


def generate_path(radius: float, length_straight: float,
                  num_straight: int, num_semi: int) -> tuple:
    """根据给定半径生成闭合的操场路径点。"""
    # 下半部分直道
    x_1 = np.linspace(-length_straight/2, length_straight/2,
                      num_straight, endpoint=False)
    y_1 = np.full(num_straight, -radius)

    # 右侧圆弧
    theta_2 = np.linspace(-np.pi/2, np.pi/2, num_semi, endpoint=False)
    x_2 = length_straight/2 + radius * np.cos(theta_2)
    y_2 = radius * np.sin(theta_2)

    # 上半部分直道
    x_3 = np.linspace(length_straight/2, -length_straight/2,
                      num_straight, endpoint=False)
    y_3 = np.full(num_straight, radius)

    # 左侧圆弧
    theta_4 = np.linspace(np.pi/2, 3*np.pi/2, num_semi, endpoint=False)
    x_4 = -length_straight/2 + radius * np.cos(theta_4)
    y_4 = radius * np.sin(theta_4)

    return (np.concatenate([x_1, x_2, x_3, x_4]),
            np.concatenate([y_1, y_2, y_3, y_4]))


def draw_stadium_map(img: np.ndarray, centers: tuple, radii: tuple):
    """在图像上绘制操场赛道轮廓。"""
    cx_l, cx_r, cy_v = centers
    r_in, r_out = radii
    # 1. 绘制外廓区域（白色/255）
    cv2.circle(img, (cx_l, cy_v), r_out, 255, -1)
    cv2.circle(img, (cx_r, cy_v), r_out, 255, -1)
    cv2.rectangle(img, (cx_l, cy_v - r_out), (cx_r, cy_v + r_out), 255, -1)

    # 2. 绘制内廓空块（黑色/0）以抠出赛道路径
    cv2.circle(img, (cx_l, cy_v), r_in, 0, -1)
    cv2.circle(img, (cx_r, cy_v), r_in, 0, -1)
    cv2.rectangle(img, (cx_l, cy_v - r_in), (cx_r, cy_v + r_in), 0, -1)


def generate_and_save_boundaries(r_in_m: float, r_out_m: float, l_m: float):
    """生成用于轨迹优化的赛道边界数据点并保存为 CSV。"""
    print("正在生成用于轨迹优化的赛道边界数据点...")
    left_x, left_y = generate_path(r_in_m, l_m, 100, 100)
    right_x, right_y = generate_path(r_out_m, l_m, 100, 100)

    df_bounds = pd.DataFrame({
        'left_border_x': left_x, 'left_border_y': left_y,
        'right_border_x': right_x, 'right_border_y': right_y
    })

    csv_name = "stadium_bounds.csv"
    df_bounds.to_csv(csv_name, index=False)
    print(f"边界数据已存入: {csv_name}")


def generate_stadium_track():
    """生成操场地图及对应的 YAML 配置文件和边界坐标 CSV。"""
    print("正在生成操场（赛道）地图...")

    # 地图物理参数
    res = 0.05
    l_m, r_in_m, r_out_m = 30.0, 10.0, 15.0

    # 地图像素尺寸
    w_px, h_px = int(80.0 / res), int(60.0 / res)
    map_img = np.zeros((h_px, w_px), dtype=np.uint8)

    cx_v, cy_v = w_px // 2, h_px // 2
    l_px = int(l_m / res)
    ri_px, ro_px = int(r_in_m / res), int(r_out_m / res)

    # 绘制并保存地图
    draw_stadium_map(map_img, (cx_v - l_px // 2, cx_v + l_px // 2, cy_v),
                     (ri_px, ro_px))
    map_base = "stadium_track"
    cv2.imwrite(f"{map_base}.png", map_img)
    save_map_yaml(map_base, res, w_px, h_px)

    # --- 生成边界坐标点 ---
    generate_and_save_boundaries(r_in_m, r_out_m, l_m)


if __name__ == '__main__':
    generate_stadium_track()
