# Poseidon 四驱四转建模与运动学仿真

根据 `CN SLAMTEC Poseidon Standard Edition_datasheet_v1.2.1-260904.pdf` 建立的简化模型。包含二维交互仿真、三维 URDF、ROS 2 Jazzy 发布节点与 RViz 配置。

## 工作空间结构



```text
siqusizhuan/
├── src/
│   └── sim_car/
│       ├── package.xml
│       ├── setup.py / setup.cfg
│       ├── sim_car/       # Python 模块
│       ├── launch/
│       ├── config/
│       ├── urdf/
│       ├── rviz/
│       ├── resource/
│       └── tests/
├── output/                # 离线仿真结果
├── build/                 # colcon 生成
├── install/               # colcon 生成
├── log/                   # colcon 生成
└── README.md
```

所有构建与启动命令均从工作空间根目录执行。迁移后请在新终端加载新的 `install/setup.bash`，避免旧包环境残留。

## 直接运行二维仿真

在工作空间根目录执行（使用系统 Python，避免 Conda 环境缺少依赖）：

```bash
PYTHONPATH=src/sim_car /usr/bin/python3 -m sim_car.app
```

默认循环演示直行、横移、斜行、原地旋转、圆弧、后退和停止。取消“自动演示”后用滑块输入车体坐标系 vx、vy、ω。支持暂停、重置、减速停止。青色线为轨迹，黄色线为轮子方向，白色箭头为车头。界面时间是固定步长仿真时间，窗口繁忙时不保证与墙钟同步。

## 三维模型与 ROS 2

首次构建：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 -m colcon build --symlink-install
source install/setup.bash
ros2 launch sim_car sim.launch.py
```

关闭 RViz 窗口会同时结束该次启动的仿真节点，避免后台残留。每次只启动一套本模型；重复启动会导致同名 TF 发布冲突。无界面模式用 Ctrl+C 退出。

已有构建时只需 source 两个 setup 文件，然后运行 launch 命令。关闭 RViz 可使用 `rviz:=false`。ROS 默认静止，开启自动演示使用 `demo:=true`。

订阅 `/cmd_vel`（geometry_msgs/Twist）；发布 `/joint_states`、`/odom`、`/tf`，robot_state_publisher 发布模型与其余关节 TF。首次接到 cmd_vel 后进入外部控制，0.5 秒未收到新命令会减速停车，不会恢复自动演示。示例：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.2}, angular: {z: 0.2}}'
```

坐标约定：x 向前，y 向左，z 向上；角速度逆时针为正。fl/fr/rl/rr 分别为左前/右前/左后/右后。

## 参数与假设

修改 `src/sim_car/config/robot.json`，重新启动即可生效。ROS 启动时按当前参数生成模型；独立 URDF 快照可通过 `PYTHONPATH=src/sim_car /usr/bin/python3 -m sim_car.model` 更新。

| 项目 | 数值 | 来源/处理 |
|---|---|---|
| 外形长宽 | 570 × 520 mm | 规格书 |
| 总高 | 322.8 mm | 图示 1；参数表为 320 mm |
| 轴距、轮距 | 450、400 mm | 图示 1/2；暂按转向轴间距使用 |
| 离地间隙 | 40 mm | 规格书；初始电池箱底面对应此值 |
| 轮半径、轮宽 | 70、55 mm | 假设，可替换 |
| 转向速率 | 90°/s | 假设，可替换 |
| 转向范围 | 连续旋转 | 假设，需厂家确认 |
| 轮缘速度限制 | 1.5 m/s | 用规格书最大直线速度暂代单轮限制 |
| 平移加速度 | 1 m/s² | 额定值；未采用 3 m/s² 峰值 |
| 指令角速度限制 | 1 rad/s | 额定值，非已知最大值 |
| 角加速度限制 | π rad/s² | 规格书 |

车体为简化箱体，传感器为示意件，位置及外形不用于标定。模型包括四个转向关节、四个轮子滚动关节。当前 URDF 用于运动学和可视化，未配置刚体惯量与接触动力学，不是 Gazebo 动力学模型。80 kg 自重和负载不参与运动学计算。轮径变化影响轮子转速。

转向轴偏置取零，轮胎纯滚动，地面平坦，悬挂固定。运动学：每个轮的速度向量为 `(vx - ω*y_i, vy + ω*x_i)`；转向最短路径允许轮速反向。整组轮速等比例限幅以保持目标运动方向。切换运动方向时先沿当前速度减速到零，再按转向速率对齐，最后加速，避免在转向期间强行赋予车体理想速度。这是保守的仿真控制策略，并非厂家控制器复现；频繁改变方向会导致反复停车。

不包含碰撞响应、轮胎打滑、悬挂、负载动力学、爬坡越障或传感器仿真。若转向存在机械限位或偏置，应先补全相应参数与模型。

## 离线结果与验证

```bash
PYTHONPATH=src/sim_car /usr/bin/python3 -m sim_car.app --export output
PYTHONPATH=src/sim_car /usr/bin/python3 -m unittest discover -s src/sim_car/tests -v
```

`output/simulation.png` 为 32 秒演示的轨迹、车体速度、转向角及轮缘速度图；`output/trajectory.csv` 为 50 Hz 数据。角度单位 rad，距离 m，速度 m/s，CSV 每行记录步进后的状态。

五项自动测试验证轮速度向量重建、整段演示的无侧滑与速率约束、直行距离、原地旋转和 URDF 关节树。ROS 运行检查验证 joint_states、odom 和 TF 实际发布。二维 GUI 依赖 tkinter，离线绘图依赖 matplotlib；本机系统 Python 已具备。

## Conda 环境下构建 teleop_joy

工作空间根目录的 `colcon.meta` 为 `teleop_joy` 指定系统 Python `/usr/bin/python3`，避免 CMake 选中 Conda Python 后找不到 ROS 的 `catkin_pkg`。在根目录执行 `colcon build` 会自动读取此配置，无需向 Conda 安装 ROS 依赖。

若包重命名后终端仍警告 `install/poseidon_sim` 不存在，是旧终端中的前缀变量残留。可打开未加载旧工作空间的新终端；或在当前终端清理并重新加载本工作空间（此操作同时清除其他叠加工作空间的前缀）：

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build
```
