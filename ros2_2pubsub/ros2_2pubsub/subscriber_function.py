import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Subscriber(Node):

  def __init__(self):
    super().__init__('nodo_subscriber')
    self.subscription = self.create_subscription(
      String,
      'comunicacion',
      self.listener_callback,
      10)
    self.subscription
  
  def listener_callback(self, msg):
    self.get_logger().info('Recibiendo: "%s"' % msg.data)
    
def main(args=None):
  rclpy.init(args=args)
  
  nodo_subscriber = Subscriber()
  
  rclpy.spin(nodo_subscriber)
  
  nodo_subscriber.destroy_node()
  rcply.shutdown()
  
if __name__=='__main__':
  main()
