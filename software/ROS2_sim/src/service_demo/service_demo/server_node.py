import rclpy
from rclpy.node import Node
# 导入预定义的AddTwoInts服务接口（包含请求Request和响应Response）
from example_interfaces.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__("add_two_ints_server")  # 节点名
        # 创建服务端：服务名称为/add_two_ints，接口类型AddTwoInts，回调函数handle_add_two_ints
        self.srv = self.create_service(AddTwoInts, "/add_two_ints", self.handle_add_two_ints)
        self.get_logger().info("加法服务端已启动，等待请求...")

    # 服务请求处理回调函数（接收请求，返回响应）
    def handle_add_two_ints(self, request, response):
        # 计算两个数的和（request.a和request.b是请求参数）
        response.sum = request.a + request.b
        self.get_logger().info(f"收到请求：a={request.a}, b={request.b}，返回结果：sum={response.sum}")
        return response  # 返回响应

def main(args=None):
    rclpy.init(args=args)          # 初始化ROS 2
    server_node = AddTwoIntsServer()# 创建服务端节点
    rclpy.spin(server_node)        # 保持节点运行，等待请求
    server_node.destroy_node()     # 销毁节点（可选）
    rclpy.shutdown()               # 关闭ROS 2

if __name__ == "__main__":
    main()