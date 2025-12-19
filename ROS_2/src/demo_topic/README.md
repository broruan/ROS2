**Demo Topic 教程**

- **用途**: 本包 `demo_topic` 实现了一个简单的 ROS 2 节点，它持续发布 `geometry_msgs/msg/Twist` 到 `/turtle1/cmd_vel`，用于控制 `turtlesim` 中的海龟做圆周运动。

**前置条件**
- **ROS 2 已安装**: 本教程假定你已在系统安装并配置好 ROS 2（如 Humble / Iron / 以你系统版本为准），并且 `colcon` 可用。
- **turtlesim 包**: 需要运行 `turtlesim_node` 来显示海龟并响应 `/turtle1/cmd_vel`。
- **工作目录**: 假定工作空间为 `/home/robot-max/Desktop/ROS_2`，源代码在 `src/demo_topic`。

**文件位置**
- 源码: `src/demo_topic/src/demo_topic.cpp`
- 包清单: `src/demo_topic/package.xml` / `src/demo_topic/CMakeLists.txt`

**一步步操作**

1) 打开终端并进入工作空间根目录

```bash
cd /home/robot-max/Desktop/ROS_2
```

2) 构建包（只构建 `demo_topic`）

```bash
colcon build --packages-select demo_topic
```

说明:
- 若你以前没有运行过 `colcon build`，第一次会花一点时间。
- 若构建失败，请把终端中报错粘贴到 issue 或发送给维护者以进一步诊断。

3) 配置环境（每次打开新终端后都需要）

```bash
source install/setup.bash
```

4) 启动 `turtlesim`（新终端）

```bash
ros2 run turtlesim turtlesim_node
```

5) 运行 `demo_topic` 节点（在已 `source` 的终端）

```bash
ros2 run demo_topic demo_topic_node
```

说明:
- 上面的可执行名 `demo_topic_node` 取决于 `CMakeLists.txt` 中 `add_executable` / `install` 声明的目标名。如果实际名字不同，请用你的可执行文件名替换它。

6) 验证发布是否生效

在另一个终端（同样 `source install/setup.bash`）运行：

```bash
ros2 topic echo /turtle1/cmd_vel
```

你应该看到周期性输出的 `linear` / `angular` 值。如果有输出，turtlesim 窗口里的乌龟会根据速度移动。

**代码详解（`src/demo_topic/src/demo_topic.cpp`）**

- 节点类: `Turtle_circle`，继承自 `rclcpp::Node`。
- 成员变量:
	- `rclcpp::TimerBase::SharedPtr timer_`：定时器，周期性触发回调。
	- `rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr Publisher_`：用于发布速度消息。
- 构造函数:
	- 使用 `Publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);` 初始化发布器。
	- 使用 `timer_ = this->create_wall_timer(1000ms, [this]() { this->time_callback(); });` 创建计时器。
	- 注意：之前代码中使用 `auto Publisher_ = ...` / `auto timer_ = ...` 会创建局部变量并遮蔽类成员，导致成员未被初始化。我们已将其修改为直接给成员赋值。
- 回调 `time_callback()`:
	- 创建并填充 `geometry_msgs::msg::Twist` 消息：`msg.linear.x = 1.0; msg.angular.z = 0.5;`
	- 调用 `Publisher_->publish(msg);` 发送消息。

**常见问题与排查**

- 问: 乌龟还是不动？
	- 检查 `turtlesim_node` 是否在运行（查看 turtlesim 窗口或运行 `ros2 node list`）。
	- 检查 topic 是否有消息：`ros2 topic echo /turtle1/cmd_vel`。
	- 如果 `ros2 topic echo` 没有输出，确认你的节点是否已成功启动（`ros2 node list` 应包含你的节点名，比如 `turtle_circle`）。
	- 检查命名空间：如果你在运行 `turtlesim_node` 时使用了不同的命名空间（或重命名 turtle），请确保 topic 名匹配。

- 问: 程序编译报错或找不到可执行文件？
	- 确认 `CMakeLists.txt` 中有 `add_executable` 并且 `ament_target_dependencies` / `install(TARGETS ...)` 正确配置。
	- 使用 `colcon build --event-handlers console_cohesion+` 可看到更详细的构建输出。

**调试建议**

- 在 `time_callback()` 中加入日志输出以确认回调被触发：

```cpp
RCLCPP_INFO(this->get_logger(), "time_callback called");
```

- 使用 `ros2 topic hz /turtle1/cmd_vel` 来查看发布频率。

**改进建议**
- 缩短定时器周期来提高发布频率，例如改为 `200ms` 或更小。
- 使用 `rclcpp::QoS(10)` 显式设 QoS 策略（如果在分布式/跨机器场景下）。

