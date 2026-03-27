import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
import math

class BalanceControlNode(Node):
    def __init__(self):
        super().__init__("balance_control_node")
        
        # 创建IMU数据订阅者
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # 创建速度命令发布者
        self.cmd_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 平衡控制参数（PD控制）
        self.Kp = 15  # 比例增益，快速响应
        self.Kd = 1   # 微分增益，提供阻尼
        self.target_pitch = 0.0  # 目标俯仰角（垂直）
        
        # 订阅键盘控制节点设置的目标俯仰角（使用专用话题）
        self.target_pitch_subscription = self.create_subscription(
            Twist,
            '/target_pitch',
            self.target_pitch_callback,
            10
        )
        
        # 状态变量
        self.last_pitch = 0.0
        self.last_time = self.get_clock().now()
        
        # 定时器用于控制循环（提高频率）
        self.timer = self.create_timer(0.01, self.control_loop)  # 100Hz控制频率
    
    def imu_callback(self, msg):
        """IMU数据回调函数"""
        # 从四元数计算俯仰角
        pitch = self.quaternion_to_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        # 获取当前时间
        current_time = self.get_clock().now()
        
        # 计算角速度（微分）
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 0:
            pitch_velocity = (pitch - self.last_pitch) / dt
        else:
            pitch_velocity = 0.0
        
        # 更新状态
        self.last_pitch = pitch
        self.last_time = current_time
        
        # 存储当前俯仰角和角速度用于控制
        self.current_pitch = pitch
        self.current_pitch_velocity = pitch_velocity
    
    def control_loop(self):
        """平衡控制循环"""
        # 检查是否已收到IMU数据
        if not hasattr(self, 'current_pitch'):
            return
        
        # 计算角度偏差
        error = self.target_pitch - self.current_pitch
        
        # PD控制（只有比例和微分项）
        # 注意：微分项应该提供阻尼，所以符号应该与比例项相反
        control_output = (self.Kp * error - 
                         self.Kd * self.current_pitch_velocity)
        
        # 控制方向：车身前倾 → 车轮向前转（恢复平衡）
        
        # 针对大角度误差的特殊处理
        if abs(error) > 1.0:  # 误差大于57度
            # 使用更强的控制力
            control_output = control_output * 1.5
        
        # 放宽控制输出范围（增强响应能力）
        if control_output > 10.0:
            control_output = 10.0
        elif control_output < -10.0:
            control_output = -10.0
        
        # 创建速度命令
        twist_msg = Twist()
        twist_msg.linear.x = control_output
        
        # 发布控制命令
        self.cmd_publisher.publish(twist_msg)
        
        # 输出车身角度、目标、误差、轮子速度
        pitch_deg = math.degrees(self.current_pitch)
        target_pitch_deg = math.degrees(self.target_pitch)
        error_deg = math.degrees(error)
        
        # 每20次循环输出一次信息
        if hasattr(self, 'debug_counter'):
            self.debug_counter += 1
        else:
            self.debug_counter = 0
            
        if self.debug_counter % 20 == 0:
            # 计算PD各项贡献
            p_term = self.Kp * error
            d_term = self.Kd * self.current_pitch_velocity
            
            # 简化的输出格式：车身角度，目标，误差，轮子速度
            self.get_logger().info(
                f"车身角度: {pitch_deg:6.1f}°, "
                f"目标角度: {target_pitch_deg:6.1f}°, "
                f"误差: {error_deg:6.1f}°, "
                f"轮子速度: {control_output:6.3f}"
            )
    
    def target_pitch_callback(self, msg):
        """接收键盘控制节点设置的目标俯仰角"""
        # 从线速度中提取目标俯仰角（键盘控制节点将目标俯仰角编码在线速度中）
        old_target = self.target_pitch
        
        if abs(msg.linear.x) > 0.001:  # 避免微小数值误差
            self.target_pitch = msg.linear.x / 2.0  # 正确解码：线速度 ÷ 2.0
        else:
            self.target_pitch = 0.0  # 线速度为0表示目标角度为0
            
            # 调试输出
            self.get_logger().info(f"收到键盘控制: 线速度={msg.linear.x:.3f}, 解码后目标角度={math.degrees(self.target_pitch):.1f}°")
            
            # 限制目标俯仰角范围
            if self.target_pitch > 0.5:
                self.target_pitch = 0.5
            elif self.target_pitch < -0.5:
                self.target_pitch = -0.5
    
    def quaternion_to_pitch(self, x, y, z, w):
        """从四元数提取俯仰角"""
        # 计算俯仰角
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        return pitch
    
    def destroy_node(self):
        """清理资源"""
        # 发送停止命令
        stop_twist = Twist()
        self.cmd_publisher.publish(stop_twist)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BalanceControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()