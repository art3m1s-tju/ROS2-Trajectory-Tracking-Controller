"""
ROS 2 轨迹跟踪演示节点。

该节点实现了 Pure Pursuit（纯跟踪）算法，订阅全局轨迹和车辆里程计信息，
计算并发布 Ackermann 转向控制指令，使车辆沿着规划好的最优路径行驶。
"""

import math
# pylint: disable=import-error
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
from scipy.spatial import KDTree


class PurePursuitTracker(Node):
    """
    ROS 2 轨迹跟踪控制器节点 (Pure Pursuit 纯跟踪算法演示框架)。
    """
    # pylint: disable=too-many-instance-attributes

    def __init__(self):
        super().__init__('pure_pursuit_tracker')

        # 1. 声明并获取核心参数 (移除离线文件读取，仅保留控制相关的动力学界限)
        self.declare_parameter('lookahead_distance', 0.8) # 预瞄距离
        self.declare_parameter('wheelbase', 0.33)        # 轴距
        self.declare_parameter('target_speed', 2.0)       # 目标速度
        self.declare_parameter('scale_factor', 0.1)       # 缩放比例

        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.speed = self.get_parameter('target_speed').value
        self.scale_factor = self.get_parameter('scale_factor').value

        self.waypoints = None
        self.kdtree = None
        self.current_idx = 0

        # 2. 从 Planner 订阅最新轨迹 (采用 Transient Local 锁存设计)
        qos_profile = QoSProfile(depth=1)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_sub = self.create_subscription(Path, '/global_trajectory',
                                                 self.path_callback, qos_profile)

        # 3. 订阅车辆当前的实时位置并发布底层控制指令
        self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom',
                                                 self.odom_callback, 10)

        self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

        self.get_logger().info("轨迹跟踪控制节点已启动 (Pure Pursuit)。等待 Planner 提供路径...")

    def path_callback(self, msg: Path):
        """解析 Planner 节点下发的全局路径，并建立高速检索空间。"""
        pts = [[p.pose.position.x, p.pose.position.y] for p in msg.poses]

        if not pts:
            self.get_logger().warning("收到的 /global_trajectory 路径为空!")
            return

        self.waypoints = np.array(pts) * self.scale_factor
        self.kdtree = KDTree(self.waypoints)
        self.current_idx = 0  # 收到新地图，重置索引
        self.get_logger().info(f"成功接收路线! 包含 {len(self.waypoints)} 个目标点。")

    def _get_current_pose(self, msg: Odometry):
        """从里程计消息中提取位置和偏航角。"""
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        quat = msg.pose.pose.orientation
        # 四元数转偏航角
        yaw = math.atan2(2.0 * (quat.w * quat.z + quat.x * quat.y),
                         1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z))
        return pos_x, pos_y, yaw

    def _find_closest_waypoint(self, current_pos):
        """寻找参考轨迹上局部最近的点。"""
        search_f, search_b = 300, 50
        min_dist = float('inf')
        closest_idx = self.current_idx

        for i in range(self.current_idx - search_b, self.current_idx + search_f):
            idx = i % len(self.waypoints)
            dist = np.linalg.norm(self.waypoints[idx] - current_pos)
            if dist < min_dist:
                min_dist = dist
                closest_idx = idx
        return closest_idx

    def odom_callback(self, msg: Odometry):
        """核心控制循环：定位 -> 寻找预瞄点 -> 计算转角 -> 发布指令。"""
        if self.waypoints is None:
            return

        pos_x, pos_y, yaw = self._get_current_pose(msg)
        curr_vec = np.array([pos_x, pos_y])

        # 1. 更新当前最近点索引
        self.current_idx = self._find_closest_waypoint(curr_vec)

        # 2. 寻找预瞄点 (Lookahead Point)
        target_idx = self.current_idx
        for i in range(self.current_idx, self.current_idx + 800):
            idx = i % len(self.waypoints)
            dist = np.linalg.norm(self.waypoints[idx] - curr_vec)
            if dist >= self.lookahead_dist:
                target_idx = idx
                break

        target_pt = self.waypoints[target_idx]

        # 3. 计算转角 (Pure Pursuit 核心逻辑)
        delta = self._compute_steering_angle(pos_x, pos_y, yaw, target_pt)

        # 4. 发布指令
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.drive.steering_angle = float(delta)
        drive_msg.drive.speed = float(self.speed)

        self.get_logger().info(
            f"x={pos_x:.2f}, y={pos_y:.2f} | target={target_idx} | delta={delta:.2f}"
        )
        self.drive_pub.publish(drive_msg)

    def _compute_steering_angle(self, pos_x, pos_y, yaw, target_pt):
        """计算 Pure Pursuit 转向角。"""
        # 将世界坐标系下的预瞄点转换到车辆局部坐标系
        dx, dy = target_pt[0] - pos_x, target_pt[1] - pos_y
        target_y_veh = -dx * math.sin(yaw) + dy * math.cos(yaw)
        target_x_veh = dx * math.cos(yaw) + dy * math.sin(yaw)

        # 阿克曼转向几何控制律: delta = atan( (2 * L * sin(alpha)) / L_d )
        alpha = math.atan2(target_y_veh, target_x_veh)
        return math.atan((2.0 * self.wheelbase * math.sin(alpha)) / self.lookahead_dist)


def main(args=None):
    """节点主入口。"""
    rclpy.init(args=args)
    tracker = PurePursuitTracker()
    try:
        rclpy.spin(tracker)
    except KeyboardInterrupt:
        pass
    finally:
        tracker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
