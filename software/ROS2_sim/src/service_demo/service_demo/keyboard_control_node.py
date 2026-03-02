import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading
import math

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__("keyboard_control_node")
        
        # 创建发布者，发布到/target_pitch话题（避免与平衡控制节点的控制命令冲突）
        self.publisher_ = self.create_publisher(Twist, '/target_pitch', 10)
        
        # IMU基准值控制参数（目标俯仰角，弧度）
        self.target_pitch = 0.0  # 初始目标俯仰角（垂直）
        self.pitch_step = 0.05  # 每次按键调整的角度步长
        self.max_pitch = 0.5    # 最大俯仰角限制
        
        # 当前控制状态
        self.current_twist = Twist()
        
        # 设置终端为非阻塞模式
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info("键盘控制节点已启动\n")
        self.get_logger().info("使用方向键控制IMU基准值（目标俯仰角）\n")
        self.get_logger().info("W/S: 增加/减少目标俯仰角\n")
        self.get_logger().info("A/D: 左转/右转（角速度）\n")
        self.get_logger().info("空格键: 重置目标俯仰角为0\n")
        self.get_logger().info("Q键: 退出\n")
        
        # 启动键盘监听线程
        self.keyboard_thread = threading.Thread(target=self._keyboard_listener)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # 注释掉定时发布，改为只在按键时发布
        # self.timer = self.create_timer(0.1, self._publish_control)
    
    def _keyboard_listener(self):
        """监听键盘输入的线程函数"""
        tty.setcbreak(sys.stdin.fileno())  # 使用cbreak模式，保持基本终端功能
        
        try:
            while rclpy.ok():
                # 非阻塞读取键盘输入
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self._handle_key(key)
                    
                    # 如果按下Q键，退出程序
                    if key.lower() == 'q':
                        break
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
    
    def _handle_key(self, key):
        """处理键盘输入"""
        twist = Twist()
        
        if key.lower() == 'w':  # 增加目标俯仰角（向前倾斜）
            self.target_pitch += self.pitch_step
            if self.target_pitch > self.max_pitch:
                self.target_pitch = self.max_pitch
            pitch_deg = math.degrees(self.target_pitch)
            self.get_logger().info(f"目标俯仰角: {pitch_deg:.1f}° (向前倾斜)\n")
            
        elif key.lower() == 's':  # 减少目标俯仰角（向后倾斜）
            self.target_pitch -= self.pitch_step
            if self.target_pitch < -self.max_pitch:
                self.target_pitch = -self.max_pitch
            pitch_deg = math.degrees(self.target_pitch)
            self.get_logger().info(f"目标俯仰角: {pitch_deg:.1f}° (向后倾斜)\n")
            
        elif key.lower() == 'a':  # 左转
            twist.angular.z = 0.5  # 固定角速度
            self.get_logger().info("左转\n")
            
        elif key.lower() == 'd':  # 右转
            twist.angular.z = -0.5  # 固定角速度
            self.get_logger().info("右转\n")
            
        elif key == ' ':  # 空格键重置目标俯仰角
            self.target_pitch = 0.0
            self.get_logger().info("重置目标俯仰角为0° (垂直)\n")
            
        elif key.lower() == 'q':  # 退出
            self.get_logger().info("退出程序\n")
            return
            
        else:
            # 其他按键忽略
            return
        
        # 设置线速度基于目标俯仰角（平衡控制会使用这个基准值）
        twist.linear.x = self.target_pitch * 2.0  # 比例系数
        self.current_twist = twist
        
        # 按键时立即发布控制消息
        self.publisher_.publish(twist)
    
    def _publish_control(self):
        """定时发布控制消息"""
        # 基于目标俯仰角生成控制命令
        control_twist = Twist()
        control_twist.linear.x = self.target_pitch * 2.0  # 比例系数
        control_twist.angular.z = self.current_twist.angular.z  # 保持角速度
        
        self.publisher_.publish(control_twist)
    
    def destroy_node(self):
        """清理资源"""
        # 发送停止命令
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        
        # 恢复终端设置
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = KeyboardControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()