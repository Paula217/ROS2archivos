import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class Publisher(Node):

  def __init__(self):
    super().__init__('nodo_publisher')
    self.publisher_ = self.create_publisher(String, 'comunicacion', 10)
    timer_period = 1 #segundos
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
    
  def timer_callback(self):
      msg = String()
      msg.data = 'Mensaje publicado con exito #: %d' % self.i
      self.publisher_.publish(msg)
      self.get_logger().info('Publicando:' "%s" % msg.data)
      self.i += 1
      
def main(args=None):
  rclpy.init(args=args)
   
  nodo_publisher = Publisher()
   
  rclpy.spin(nodo_publisher)
  nodo_publisher.destroy_node()
  rcply.shutdown()
   
if __name__ == '__main__':
  main()  
 
