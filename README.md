# 轨迹跟踪控制器与 ROS 2 节点化技术路线指南 (Trajectory Tracking & ROS 2 Node Guide)

> **基础代码声明 (Credits)**:
> 本项目代码（特别是底层的全局轨迹规划算法）是基于开源仓库 [art3m1s-tju/Curvature-Distance-Trajectory-SQP](https://github.com/art3m1s-tju/Curvature-Distance-Trajectory-SQP) 进行拓展和二次开发的。它将原先的静态离线规划代码重构成了实时 ROS 2 跟踪控制流。

## 1. 项目概述 (Project Overview)

本项目的目标是把“赛道最优轨迹优化”和“ROS 2 实时轨迹跟踪控制”串成一条完整链路，让我们可以在仿真环境中完成以下流程：

1. 输入赛道几何信息（本项目当前使用标准赛道边界 CSV）。
2. 使用基于 SQP / OSQP 的优化器计算一条全局最优轨迹。
3. 将最优轨迹通过 ROS 2 `nav_msgs/Path` 发布给下游控制节点。
4. 控制节点根据车辆当前里程计信息，实时计算 Ackermann 转向与速度指令。
5. 在 F1TENTH 仿真环境中验证车辆是否能够稳定跟踪最优轨迹。

从仓库内容上看，这个项目主要由两部分组成：

- 离线轨迹优化：`optimize_with_boundaries.py`
- 在线 ROS 2 闭环跟踪：`ros2_planner_node.py`、`ros2_tracking_node_demo.py`

如果只看功能边界，可以这样理解：

- 优化器负责“算出应该走哪条线”。
- ROS 2 规划节点负责“把这条线发布出去”。
- 跟踪节点负责“根据车当前的位置，把车拉回这条线”。

### 1.1 仓库文件说明

当前仓库中的主要文件用途如下：

- `generate_stadium_track.py`：生成操场形测试地图，以及与之匹配的赛道边界 CSV。
- `optimize_with_boundaries.py`：读取标准边界格式的赛道数据并离线计算最优轨迹。
- `ros2_planner_node.py`：在 ROS 2 中在线计算并发布全局轨迹到 `/global_trajectory`。
- `ros2_tracking_node_demo.py`：订阅全局轨迹和里程计，用 Pure Pursuit 输出 `/drive` 控制命令。

### 1.2 输入与输出

本项目当前使用单一赛道输入格式：

- 标准边界输入：CSV 中包含 `left_border_x`、`left_border_y`、`right_border_x`、`right_border_y`

对应输出主要有两类：

- 离线结果：`stadium_optimal_trajectory.csv`、`trajectory_comparison.png`
- 在线 ROS 2 结果：`/global_trajectory`、`/drive`

因此，这个项目既可以作为一个单机轨迹优化工具使用，也可以作为一个 ROS 2 轨迹规划与跟踪示例工程使用。

---

## 2. 系统架构与数据流 (Architecture & Data Flow)

### 2.1 为什么要拆成 Planner 和 Tracker 两个节点？

最优轨迹求解本身是一个计算量较大的过程。对于整圈赛道的 SQP 优化，计算过程包含矩阵构建、二次规划求解和多轮迭代，这类工作更适合作为“低频、一次性或按需触发”的全局规划任务。当前仓库仅保留一份离线优化脚本 `optimize_with_boundaries.py`，用来统一承载这类计算。

而轨迹跟踪控制是一个高频闭环过程。车辆在仿真中运行时，需要持续读取 `/ego_racecar/odom`，并以较高频率输出 `/drive` 控制命令。如果把最优轨迹求解和控制闭环强行塞到同一个循环里，系统结构会变得混乱，也不利于后续替换控制器或接入其他上层模块。

因此当前工程采用了典型的 ROS 2 发布-订阅拆分：

- `ros2_planner_node.py`：负责读取赛道、计算全局最优轨迹、发布 `/global_trajectory`
- `ros2_tracking_node_demo.py`：负责订阅 `/global_trajectory` 与 `/ego_racecar/odom`，输出 `/drive`

这样拆分后，系统有几个明显好处：

- 规划与控制解耦，便于分别调试。
- 规划结果可复用，后续可以替换为其他全局规划器。
- 跟踪控制器可替换，后续可以把 Pure Pursuit 换成 Stanley、LQR 或 MPC。
- 节点职责清晰，更符合 ROS 2 工程组织方式。

### 2.2 当前 ROS 2 数据流

本项目当前的数据流如下：

1. `ros2_planner_node.py` 读取赛道 CSV。
2. 节点内部调用 `load_track_data(...)` 和 `optimize_trajectory(...)` 计算最优轨迹。
3. 计算结果被封装成 `nav_msgs/Path`，发布到 `/global_trajectory`。
4. `ros2_tracking_node_demo.py` 订阅 `/global_trajectory`，将路径点缓存到本地，并建立 KD-Tree 方便最近点搜索。
5. 跟踪节点持续订阅 `/ego_racecar/odom` 获取车辆当前位置与姿态。
6. 跟踪节点使用 Pure Pursuit 算法计算目标预瞄点和前轮转角。
7. 控制指令以 `ackermann_msgs/AckermannDriveStamped` 形式发布到 `/drive`。
8. F1TENTH 仿真器接收 `/drive` 后驱动车辆运动，并继续发布新的 `/ego_racecar/odom`，形成闭环。

### 2.3 当前实现中几个关键设计点

#### 1. 赛道输入兼容两种格式

`ros2_planner_node.py` 目前复用了 `optimize_with_boundaries.py` 中的 `load_track_data(...)`，默认读取完整边界 CSV，并据此完成在线最优轨迹求解。

#### 2. 规划结果采用锁存式发布

规划节点对 `/global_trajectory` 使用了 `Transient Local` QoS。这样做的好处是：即使跟踪节点晚于规划节点启动，只要规划节点还在，跟踪节点一连上也能拿到最近一次发布的轨迹，不需要重新触发规划。

#### 3. 跟踪节点使用 Pure Pursuit

`ros2_tracking_node_demo.py` 中实现的是一个比较轻量的 Pure Pursuit 跟踪器，核心步骤包括：

- 找到车辆附近的轨迹点
- 沿轨迹向前搜索满足预瞄距离的目标点
- 将目标点转换到车辆坐标系
- 根据阿克曼几何关系计算转向角 `delta`
- 发布固定目标速度和实时转向角

这个实现适合作为演示版本与联调基础版本，后续如果需要更高速度、更强鲁棒性，可以在这个节点位置继续升级控制算法。

#### 4. 离线与在线两条路径都保留

本仓库同时保留了：

- 离线脚本：便于单独验证优化器是否正常
- ROS 2 节点：便于在仿真器中做闭环测试

这样做的好处是，当在线联调出现问题时，我们可以先回到离线脚本确认“赛道输入和最优轨迹本身是否正确”，再去排查 ROS 2 话题、坐标系、仿真器配置等问题。

### 2.4 适合怎么使用这个仓库

比较推荐的使用顺序是：

1. 先用 `generate_stadium_track.py` 生成一个可控的测试赛道。
2. 用 `optimize_with_boundaries.py` 验证最优轨迹能否正确算出。
3. 确认离线优化结果没问题后，再启动 `ros2_planner_node.py` 和 `ros2_tracking_node_demo.py` 做在线闭环验证。
4. 等整套流程跑通之后，再替换赛道数据、调整速度参数、修改控制器或接入真实平台。

这样排查效率最高，也最容易定位问题出在“优化器”、“ROS 2 通信”还是“仿真器配置”哪一层。

---

## 3. 从零开始搭建并跑通完整仿真流程 (Simulation Guide)

这一节不再按“方案 A / B / C”拆分，而是直接给出一条按顺序执行的完整流程：从安装 F1TENTH 仿真器开始，到生成测试地图、导入 Docker 仿真环境、计算最优轨迹、启动 ROS 2 规划节点与跟踪节点，直到看到车辆在仿真里沿最优轨迹行驶。

### 3.1 当前仓库文件结构与用途

在开始之前，先明确本仓库里几个核心文件分别负责什么：

- `generate_stadium_track.py`：生成一个标准操场形测试地图，并输出 `stadium_track.png`、`stadium_track.yaml`、`stadium_bounds.csv`。
- `optimize_with_boundaries.py`：本项目唯一保留的离线优化脚本。读取 `stadium_bounds.csv` 这类边界格式输入，并执行 SQP 最优轨迹计算。
- `ros2_planner_node.py`：ROS 2 全局规划节点。读取赛道 CSV，在线计算最优轨迹，并把结果发布到 `/global_trajectory`。
- `ros2_tracking_node_demo.py`：ROS 2 纯跟踪控制节点。订阅 `/global_trajectory` 和 `/ego_racecar/odom`，向 `/drive` 输出 Ackermann 控制命令。

### 3.2 前置环境

推荐环境如下：

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
- Docker
- Rocker
- F1TENTH Gym ROS 2 仿真环境

建议先安装宿主机侧常用工具：

```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions docker.io
pip3 install --user rocker
```

如果宿主机尚未安装 ROS 2 Humble，请先完成 ROS 2 官方安装，并确认下面命令可用：

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### 3.3 安装 F1TENTH 仿真器

F1TENTH 仿真环境可参考官方仓库 [f1tenth/f1tenth_gym](https://github.com/f1tenth/f1tenth_gym) 和 ROS 侧工程 `f1tenth_gym_ros`。

先在宿主机创建 F1TENTH 工作区（这里按您的实际目录使用 `~/sim_ws`）：

```bash
mkdir -p ~/sim_ws/src
cd ~/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```

安装 F1TENTH Python 仿真底层：

```bash
pip3 install --user git+https://github.com/f1tenth/f1tenth_gym.git
```

回到工作区根目录并编译：

```bash
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3.4 启动 F1TENTH Docker 仿真平台

F1TENTH 官方环境通常通过 Rocker 启动带图形界面的容器。请在宿主机终端执行：

```bash
cd ~/sim_ws/src/f1tenth_gym_ros
sudo rocker --nvidia --x11 --volume .:/sim_ws/src/f1tenth_gym_ros -- f1tenth_gym_ros
```

如果您的机器没有 NVIDIA 显卡，可以先去掉 `--nvidia` 再尝试。

进入容器后执行：

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

执行成功后，应该会看到 F1TENTH 仿真窗口 / RViz2 画面打开，并出现 `ego_racecar`。

### 3.5 安装本项目 Python 依赖

回到宿主机，在本项目根目录安装当前仓库脚本依赖：

```bash
cd /home/art3m1s/ROS2-Trajectory-Tracking-Controller
pip3 install --user numpy pandas scipy matplotlib osqp opencv-python-headless pyyaml
```

如果您准备直接运行 ROS 2 节点，还需要确保宿主机已经安装这些 ROS 2 Python 相关包：

```bash
sudo apt install -y \
  ros-humble-rclpy \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-ackermann-msgs
```

### 3.6 生成测试地图与赛道边界文件

本项目已经提供了一个“操场形”测试赛道生成器。执行：

```bash
cd /home/art3m1s/ROS2-Trajectory-Tracking-Controller
python3 generate_stadium_track.py
```

执行完成后，项目根目录会生成以下文件：

- `stadium_track.png`：仿真器使用的地图图像
- `stadium_track.yaml`：仿真器加载地图所需的元数据
- `stadium_bounds.csv`：左右边界形式的赛道数据，供最优轨迹计算使用

### 3.7 把测试地图拷贝到 F1TENTH 工程

在宿主机执行：

```bash
cp /home/art3m1s/ROS2-Trajectory-Tracking-Controller/stadium_track.png ~/sim_ws/src/f1tenth_gym_ros/maps/
cp /home/art3m1s/ROS2-Trajectory-Tracking-Controller/stadium_track.yaml ~/sim_ws/src/f1tenth_gym_ros/maps/
```

之所以复制到 `~/sim_ws/src/f1tenth_gym_ros/maps/`，是因为这个目录会被 Rocker 挂载到容器中的 `/sim_ws/src/f1tenth_gym_ros/maps/`。

### 3.8 修改仿真器地图配置

打开宿主机上的 `~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml`，将地图路径改为新生成的操场地图，并设置一个合理的起跑点：

```yaml
map_path: '/sim_ws/src/f1tenth_gym_ros/maps/stadium_track'
map_img_ext: '.png'

sx: 0.0
sy: -12.5
stheta: 0.0

sx1: -5.0
sy1: -12.5
stheta1: 0.0
```

这里的含义是：

- `map_path` 指向容器内的地图文件前缀，不要写 `.png` 后缀。
- `sx / sy / stheta` 是主车初始位姿。
- `sx1 / sy1 / stheta1` 是另一辆车的初始位姿，放远一些可以避免刚启动就干扰测试。

### 3.9 重启仿真器加载新地图

如果仿真器之前已经开着，请先在容器终端按 `Ctrl + C` 停掉，再重新执行第 3.4 节中的命令：

```bash
cd /sim_ws
source /opt/ros/foxy/setup.bash
colcon build
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

当仿真窗口里出现新生成的操场形赛道时，说明地图加载成功。

### 3.10 先离线计算一份最优轨迹

在真正启动 ROS 2 节点之前，建议先离线跑一次优化器，确认赛道数据和优化过程是正常的。

推荐直接使用 `generate_stadium_track.py` 生成的 `stadium_bounds.csv` 来离线验证优化器：

```bash
cd /home/art3m1s/ROS2-Trajectory-Tracking-Controller
python3 optimize_with_boundaries.py
```

正常情况下，脚本会输出两类结果：

- 终端里打印 SQP 迭代日志、alpha 统计信息
- 当前目录生成 `trajectory_comparison.png` 和 `stadium_optimal_trajectory.csv`

其中 `stadium_optimal_trajectory.csv` 是后续验证最优轨迹结果时最直观的离线产物。

### 3.11 启动 ROS 2 全局规划节点

保持 F1TENTH 仿真器已经启动，再在宿主机新开一个终端：

```bash
source /opt/ros/humble/setup.bash
cd /home/art3m1s/ROS2-Trajectory-Tracking-Controller
python3 ros2_planner_node.py --ros-args -p track_csv:=stadium_bounds.csv -p epsilon:=1000.0
```

说明如下：

- `track_csv:=stadium_bounds.csv` 表示直接使用第 3.6 节生成的边界文件。
- `epsilon:=1000.0` 表示使用更偏向“平衡型”的轨迹优化目标。
- 该节点会在内部完成 SQP 计算，并把结果发布到 `/global_trajectory`。
- 节点使用了 Transient Local QoS，所以跟踪节点晚一点启动也能收到已经发布的轨迹。

看到类似“成功对外投射 N 个物理轨迹节点”的日志，说明规划节点已经工作正常。

### 3.12 启动 ROS 2 轨迹跟踪节点

再开一个宿主机终端，执行：

```bash
source /opt/ros/humble/setup.bash
cd /home/art3m1s/ROS2-Trajectory-Tracking-Controller
python3 ros2_tracking_node_demo.py --ros-args -p scale_factor:=1.0 -p target_speed:=2.0
```

说明如下：

- `scale_factor:=1.0` 表示规划轨迹坐标不再额外缩放，适合直接对接这里生成的操场地图。
- `target_speed:=2.0` 是跟踪节点下发给 `/drive` 的目标速度，可根据仿真稳定性继续调小或调大。
- 节点收到 `/global_trajectory` 后，会在内存中建立 KD-Tree，并持续订阅 `/ego_racecar/odom` 进行 Pure Pursuit 控制。

看到类似“成功接收并装载路线”的日志后，说明跟踪节点已经进入闭环控制状态。

### 3.13 预期结果

整套流程跑通后，您应该看到以下现象：

- F1TENTH 仿真窗口中，车辆沿着操场赛道持续行驶。
- `ros2_planner_node.py` 终端打印最优轨迹计算和发布日志。
- `ros2_tracking_node_demo.py` 终端持续打印车辆当前位置、目标点索引和转向角 `delta`。
- 项目目录下已经存在 `stadium_optimal_trajectory.csv` 与 `trajectory_comparison.png`，可用于离线复核优化结果。

### 3.14 完整执行顺序汇总

如果您只想按顺序照着执行，最短流程如下：

1. 安装 ROS 2、Docker、Rocker、F1TENTH 仿真器。
2. 启动 F1TENTH Docker 容器和 `gym_bridge_launch.py`。
3. 在本项目目录执行 `python3 generate_stadium_track.py` 生成测试地图与 `stadium_bounds.csv`。
4. 把 `stadium_track.png` 和 `stadium_track.yaml` 复制到 `~/sim_ws/src/f1tenth_gym_ros/maps/`。
5. 修改 `~/sim_ws/src/f1tenth_gym_ros/config/sim.yaml`，将地图切换到 `stadium_track`。
6. 重启 F1TENTH 仿真器，确认新地图已经显示出来。
7. 在本项目目录执行 `python3 optimize_with_boundaries.py`，先离线确认最优轨迹可正常计算。
8. 启动 `python3 ros2_planner_node.py --ros-args -p track_csv:=stadium_bounds.csv -p epsilon:=1000.0`。
9. 启动 `python3 ros2_tracking_node_demo.py --ros-args -p scale_factor:=1.0 -p target_speed:=2.0`。
10. 在仿真窗口观察车辆是否能沿着最优轨迹稳定行驶。

### 3.15 常见问题

- 规划节点启动后立刻报“地图文件不存在”：
  请确认当前工作目录就是本仓库根目录，或者把 `track_csv` 参数换成绝对路径。
- 跟踪节点没有任何输出：
  通常是因为 `/global_trajectory` 还没有发布，或者仿真器没有正确发布 `/ego_racecar/odom`。
- 车辆一启动就偏出赛道：
  优先检查 `sim.yaml` 中的 `sx / sy / stheta` 是否与生成地图的坐标系对齐。
- 轨迹和地图看起来有尺度不一致：
  优先检查跟踪节点的 `scale_factor` 是否误设为 `0.1` 等其他值。
- 仿真器中没有加载新地图：
  请确认拷贝的是 `stadium_track.png` 和 `stadium_track.yaml`，并且 `map_path` 指向的是容器内路径 `/sim_ws/src/f1tenth_gym_ros/maps/stadium_track`。
