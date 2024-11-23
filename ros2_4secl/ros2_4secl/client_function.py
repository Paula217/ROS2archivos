import sys

from ros2_4pkg.srv import Data
import rclpy
from rclpy.node import Node


class Client(Node):

    def __init__(self):
        super().__init__('nodo_client')
        self.cli = self.create_client(Data, 'data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('servicio no disponible, esperando a conexion')
        self.req = Data.Request()

    def send_request(self, num):
        self.req.num = num
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    nodo_client_asin = Client()
    future = nodo_client_asin.send_request(int(sys.argv[1]))
    rclpy.spin_until_future_complete(nodo_client_asin, future)
    response = future.result()
    nodo_client_asin.get_logger().info(
        'El cuadrado del numero %d es %d' %
        (nodo_client_asin.req.num, response.cua))

    nodo_client_asin.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
