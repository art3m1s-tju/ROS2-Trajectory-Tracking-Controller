"""
ROS 2 全局轨迹规划节点。

该节点负责加载赛道边界数据，利用 SQP 算法优化出最优全局轨迹，
并将其发布到 /global_trajectory 话题，供跟踪节点使用。
"""

import os
# pylint: disable=import-error
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from optimize_with_boundaries import load_track_data, optimize_trajectory


class GlobalTrajectoryPlanner(Node):
    """
    ROS 2 全局轨迹规划器 (Global Planner):
    1. 根据传入的 boundaries CSV 读取赛道左右边界；
    2. 计算 SQP 迭代得出极值切弯路线 (最小曲率或平衡型)；
    3. 用 Transient Local (Latched) 形态不断发布至 /global_trajectory，
       以便哪怕是稍后才能冷启动的跟踪节点也可以即时接收。
    """
    # pylint: disable=too-few-public-methods

    def __init__(self):
        super().__init__('global_trajectory_planner')

        self.declare_parameter('track_csv', 'stadium_bounds.csv')
        self.declare_parameter('epsilon', 1000.0)

        track_csv = self.get_parameter('track_csv').value
        epsilon = self.get_parameter('epsilon').value

        # ROS 2 Transient Local QoS (确保晚加入的节点也能收到轨迹)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_pub = self.create_publisher(Path, '/global_trajectory', qos_profile)

        # 检查地图文件是否存在
        if not os.path.exists(track_csv):
            self.get_logger().error(f"地图文件 {track_csv} 不存在！请检查路径。")
            return

        self.get_logger().info(f"正在读取赛道边界文件: {track_csv}")
        try:
            p_inner, _, v_vec = load_track_data(track_csv)
        except (IOError, ValueError) as err:
            self.get_logger().error(f"加载赛道失败: {err}")
            return

        self.get_logger().info(f"开始全局 SQP 轨迹优化推演 (epsilon={epsilon})...")

        # 车辆物理限制与安全间距
        veh_l, veh_w, safe_m = 4.7, 2.0, 0.5

        # 调用核心算法解算最优坐标系
        r_opt, _ = optimize_trajectory(
            p_inner, v_vec, veh_l, veh_w, safe_m,
            epsilon=epsilon, max_iter=50,
            gamma_normal=0.5, gamma_inaccurate=0.1
        )

        self.get_logger().info("SQP 极致走线已优化完成。正在发布至 /global_trajectory...")
        self.publish_trajectory(r_opt)

    def publish_trajectory(self, r_opt):
        """将优化后的轨迹坐标点打包成 ROS Path 消息并发布。"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # 映射到 ROS 2 的 map 坐标系
        path_msg.header.frame_id = 'map'

        for pt in r_opt:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(pt[0])
            pose.pose.position.y = float(pt[1])
            pose.pose.position.z = 0.0

            # 轨迹点不设置姿态，由后端跟踪逻辑计算
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f"成功投射 {len(r_opt)} 个物理轨迹节点！")
        self.get_logger().info("规划器已进入持续待命状态 (Transient Local)。")


def main(args=None):
    """节点主入口函数。"""
    rclpy.init(args=args)
    planner = GlobalTrajectoryPlanner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
