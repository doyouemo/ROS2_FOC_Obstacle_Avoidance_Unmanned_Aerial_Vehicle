import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys  # 用于接收命令行参数

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")  # 节点名
        # 创建客户端：服务名称/add_two_ints，接口类型AddTwoInts
        self.client = self.create_client(AddTwoInts, "/add_two_ints")
        # 等待服务端上线（超时10秒）
        while not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("服务端未上线，等待中...")
        self.get_logger().info("服务端已连接，准备发送请求")

    # 发送服务请求的函数
    def send_request(self, a, b):
        # 创建请求对象，设置参数a和b
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        # 发送异步请求（避免阻塞），并指定响应回调函数
        self.future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, self.future)  # 等待响应
        return self.future.result()  # 返回响应结果

def main(args=None):
    rclpy.init(args=args)
    # 处理args默认值：若为None则设为空列表
    args = args or sys.argv  
    client_node = AddTwoIntsClient()
    
    # 检查参数数量（sys.argv包含程序名，所以需要至少3个元素：client_node a b）
    if len(args) < 3:
        client_node.get_logger().error("请传入两个整数参数！示例：ros2 run service_demo client_node 3 5")
        client_node.destroy_node()
        rclpy.shutdown()
        return
    
    try:
        a = int(args[1])
        b = int(args[2])
    except ValueError:
        client_node.get_logger().error("参数必须是整数！")
        client_node.destroy_node()
        rclpy.shutdown()
        return
    
    # 发送请求并获取响应
    response = client_node.send_request(a, b)
    client_node.get_logger().info(f"请求结果：{a} + {b} = {response.sum}")
    
    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)