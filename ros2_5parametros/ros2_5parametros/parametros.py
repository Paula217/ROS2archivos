import rclpy
import rclpy.node

class Param(rclpy.node.Node):
    def __init__(self):
        super().__init__('nodo_parametro')

        self.declare_parameter('speed', 1.0)  
        self.declare_parameter('conteo', 0)

        speed = self.get_parameter('speed').get_parameter_value().double_value
        conteo = self.get_parameter('conteo').get_parameter_value().integer_value
        self.timer = self.create_timer(speed, self.timer_callback)

        self.last_speed = speed 

        self.create_timer(0.5, self.check_and_update_timer)
        self.objetivo_=0

    def check_and_update_timer(self):
        current_speed = self.get_parameter('speed').get_parameter_value().double_value

        if current_speed != self.last_speed:
            self.last_speed = current_speed

            # Si el temporizador ya existe, lo detendremos
            if self.timer:
                self.timer.cancel()

            # Crear un nuevo temporizador con la frecuencia actualizada
            self.timer = self.create_timer(current_speed, self.timer_callback)

    def timer_callback(self):
    
        conteo = self.get_parameter('conteo').get_parameter_value().integer_value
        
        if conteo>self.objetivo_:
       	    self.objetivo_ += 1
       	elif conteo<self.objetivo_:
       	    self.objetivo_ = self.objetivo_-1
       	else:
       	    self.objetivo_=conteo
           
        self.get_logger().info('Conteo: %d' % self.objetivo_)


def main():
    rclpy.init()
    node = Param()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

