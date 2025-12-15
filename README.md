# 使用 `ros2 pkg create` 组织并构建一个 C++ 功能包（详解）

本教程说明如何使用 `ros2 pkg create` 命令创建一个基于 `ament_cmake` 的 C++ 功能包，逐行解释生成的 `CMakeLists.txt` 与 `package.xml`，并演示如何构建与运行节点。

**创建功能包**
- **命令示例**：在工作区根目录运行以下命令来创建包：
```bash
ros2 pkg create ROS_2 --build-type ament_cmake --license Apache-2.0
```
- **作用**：该命令会在当前目录下生成 `ROS_2/` 包目录，内含 `package.xml`、`CMakeLists.txt`、`src/` 等基本模板文件，方便初始化 C++ 功能包。

**`CMakeLists.txt` 逐行解释**
下面给出一个典型的 `CMakeLists.txt`（生成模板或你现在仓库中的文件），并对关键行逐条解释：

```cmake
cmake_minimum_required(VERSION 3.8)
project(ROS_2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(main src/main.cpp)
ament_target_dependencies(main rclcpp)

install(TARGETS
	main
	DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
	find_package(ament_lint_auto REQUIRED)
	set(ament_cmake_copyright_FOUND TRUE)
	set(ament_cmake_cpplint_FOUND TRUE)
	ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- `cmake_minimum_required(VERSION 3.8)`：指定所需的最低 CMake 版本，保障语法兼容性。
- `project(ROS_2)`：定义 CMake 项目名，通常建议与 `package.xml` 中的包名一致。
- `if(CMAKE_COMPILER_IS_GNUCXX ...) add_compile_options(...) endif()`：为常见编译器添加警告选项，帮助捕捉潜在问题。
- `find_package(ament_cmake REQUIRED)`：找到 `ament_cmake` 构建系统，若未找到则停止配置；这是 ament-based 包的必需项。
- `find_package(rclcpp REQUIRED)`：查找 `rclcpp`（C++ 客户端库），若你的节点使用 `rclcpp`，需要在此列出。
- `add_executable(main src/main.cpp)`：将 `src/main.cpp` 编译为一个名为 `main` 的可执行目标。
- `ament_target_dependencies(main rclcpp)`：将 `rclcpp` 的 include 路径、link 库等信息应用到 `main` 目标，等同于链接依赖。
- `install(TARGETS main DESTINATION lib/${PROJECT_NAME})`：安装规则，将编译后的二进制放到 `install/lib/ROS_2/` 下，确保 `ros2 run` 可以找到。
- `if(BUILD_TESTING) ... ament_lint_auto_find_test_dependencies() endif()`：仅在启用测试时查找并运行 linter，常用于开发流程。
- `ament_package()`：标记这是一个 ament 包，生成必要的包配置元数据，供安装时使用。

**`package.xml` 中依赖配置说明**
`package.xml` 是 ROS 包的重要元数据文件，常见元素如下（示例来自生成的模板）：

```xml
<package format="3">
	<name>ROS_2</name>
	<version>0.0.0</version>
	<description>TODO: Package description</description>
	<maintainer email="you@example.com">your_name</maintainer>
	<license>Apache-2.0</license>

	<depend>rclcpp</depend>
	<buildtool_depend>ament_cmake</buildtool_depend>

	<test_depend>ament_lint_auto</test_depend>
	<test_depend>ament_lint_common</test_depend>

	<export>
		<build_type>ament_cmake</build_type>
	</export>
</package>
```

- `format="3"`：指定 package.xml 的格式版本（较新 ROS2 使用 3）。
- `<name>` / `<version>` / `<description>` / `<maintainer>` / `<license>`：基本元信息。
- `<buildtool_depend>ament_cmake</buildtool_depend>`：声明构建工具依赖（CMake 的 ament 后端），`rosdep` 会基于此安装构建时需要的工具包。
- `<depend>rclcpp</depend>`：声明运行时与编译时都需要的包（等价于同时是 build 和 exec 依赖）。也可分别使用 `<build_depend>`、`<exec_depend>` 来更精细地分类。
- `<test_depend>`：仅在测试时需要的依赖。
- `<export><build_type>ament_cmake</build_type></export>`：告知 ament 使用哪种构建类型。

如何添加/修改依赖：
- 在开发时，如果你在代码中使用了新的 ROS 库（例如 `std_msgs`、`sensor_msgs`、`tf2` 等），需要把它们加入 `package.xml`：
	- 如果是编译时也需要的，添加 `<depend>pkg_name</depend>` 或 `<build_depend>` + `<exec_depend>`。
	- 如果是仅运行时需要（较少见），使用 `<exec_depend>`。
- 更新后，运行 `rosdep` 安装系统依赖：
```bash
rosdep install --from-paths src --ignore-src -r -y
```
（在本仓库里替换 `src` 为包含 `ROS_2/` 的父目录或直接使用 `rosdep install --from-paths ROS_2 --ignore-src -r -y`。）

**构建功能包**
在包含 `ROS_2/` 的工作区根目录运行：

```bash
# 安装系统依赖（推荐）
rosdep install --from-paths ROS_2 --ignore-src -r -y

# 构建包
colcon build --packages-select ROS_2 --symlink-install
```

- `--symlink-install` 的作用是在安装时使用源码符号链接，而不是复制二进制到安装目录，便于调试和快速迭代代码（只在开发时使用）。

构建成功后，记得 source 安装的环境：

```bash
source install/setup.bash
```

**运行节点**
- 使用 `ros2 run`：
```bash
ros2 run ROS_2 main
```
- 或直接运行已安装的二进制（用于调试路径或权限问题）：
```bash
./install/lib/ROS_2/main
```

运行后，节点（在 `src/main.cpp` 中）会创建名为 `main` 的节点并打印日志，例如：
```
[INFO] [<timestamp>] [my_node]: Hello, ROS2
```

**常见问题与排查**
- 构建报错找不到 `rclcpp`：确保已安装对应 ROS2 发行版并执行 `source /opt/ros/<distro>/setup.bash`，然后再运行 `colcon build`。
- `ros2 run` 找不到包或可执行：确认已 `source install/setup.bash`，并检查 `install/lib/ROS_2/` 是否含有 `main` 可执行文件。
- 权限问题：若直接运行 `./install/lib/ROS_2/main` 报权限错误，运行 `chmod +x install/lib/ROS_2/main`。

**示例：添加新的依赖（流程）**
1. 在代码中使用某个接口（例如 `std_msgs/msg/String.hpp`）。
2. 在 `package.xml` 中加入 `<depend>std_msgs</depend>`。
3. 在 `CMakeLists.txt` 中加入 `find_package(std_msgs REQUIRED)` 并在 `ament_target_dependencies(main rclcpp std_msgs)` 中添加 `std_msgs`。
4. 运行 `rosdep install --from-paths ROS_2 --ignore-src -r -y` 安装系统依赖。
5. 重新构建：`colcon build --packages-select ROS_2 --symlink-install`。

---
通过以上步骤，你可以成功创建、构建并运行一个基于 `ament_cmake` 的 C++ 功能包，并理解其配置文件的作用与修改方法。祝你在 ROS2 开发中取得成功！