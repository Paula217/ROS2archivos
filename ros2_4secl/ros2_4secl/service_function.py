from ros2_4pkg.srv import Data

import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__('nodo_service')
        self.srv = self.create_service(Data, 'data', self.data_callback)

    def data_callback(self, request, response):
        response.cua = request.num * request.num
        self.get_logger().info('Numero digitado:\n %d' % (request.num))

        return response


def main():
    rclpy.init()

    nodo_service = Service()

    rclpy.spin(nodo_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
