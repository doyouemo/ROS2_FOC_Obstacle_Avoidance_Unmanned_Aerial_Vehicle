# ROS2平衡车仿真项目Makefile

# 默认目标
all: build run

# 构建项目
build:
	@echo "构建ROS2包..."
	colcon build --packages-select my_robot_sim service_demo

# 构建并运行仿真
run: build
	@echo "启动仿真环境..."
	@echo "请在新终端中运行: make keyboard"
	bash -c "source install/setup.bash && ros2 launch my_robot_sim sim.launch.py"

# 仅运行仿真（假设已构建）
run-only:
	@echo "启动仿真环境..."
	bash -c "source install/setup.bash && ros2 launch my_robot_sim sim.launch.py"

# 启动键盘控制节点
keyboard:
	@echo "启动键盘控制..."
	bash -c "source install/setup.bash && ros2 run service_demo keyboard_control_node"

# 快速构建（仅构建修改的包）
fast:
	@echo "快速构建..."
	colcon build --packages-select my_robot_sim service_demo --symlink-install

# 查看所有话题
list-topics:
	bash -c "source install/setup.bash && ros2 topic list"

# 查看IMU数据
imu:
	bash -c "source install/setup.bash && ros2 topic echo /imu"

# 查看控制命令
cmd:
	bash -c "source install/setup.bash && ros2 topic echo /cmd_vel"

# 查看节点列表
nodes:
	bash -c "source install/setup.bash && ros2 node list"

# 帮助信息
help:
	@echo "可用命令:"
	@echo "  make           - 构建并运行仿真"
	@echo "  make build     - 仅构建项目"
	@echo "  make run       - 构建并运行仿真"
	@echo "  make run-only  - 仅运行仿真（假设已构建）"
	@echo "  make keyboard  - 启动键盘控制节点"
	@echo "  make fast      - 快速构建（使用符号链接）"
	@echo "  make clean     - 清理构建文件"
	@echo "  make imu       - 查看IMU数据"
	@echo "  make cmd       - 查看控制命令"
	@echo "  make nodes     - 查看节点列表"
	@echo "  make help      - 显示此帮助信息"

.PHONY: all build run run-only keyboard clean fast list-topics imu cmd nodes help