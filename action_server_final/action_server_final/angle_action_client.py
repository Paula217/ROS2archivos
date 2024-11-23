import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_definicion.action import Angulo 
import cv2
import numpy as np
import base64

class AngleActionClient(Node):
    def __init__(self):
        super().__init__('angulo_action_client')

        # Declarar el parámetro con un valor predeterminado de 1.0
        self.declare_parameter('goal_angle', 1.0)

        self._action_client = ActionClient(self, Angulo, 'angulo')

        # Obtener el valor del parámetro
        self.goal_angle = self.get_parameter('goal_angle').get_parameter_value().double_value

    def send_goal(self):
        goal_msg = Angulo.Goal()
        goal_msg.target_angle = self.goal_angle
        self.goal_angle = self.get_parameter('goal_angle').get_parameter_value().double_value

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal: {self.goal_angle} degrees')
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Current angle: {feedback.feedback.current_angle:.2f} degrees')

    def result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Goal succeeded!')

        # Decodificar la imagen desde base64
        image_data = result.image_data
        image_bytes = base64.b64decode(image_data)

        # Convertir la imagen en un arreglo de NumPy y mostrarla
        np_arr = np.frombuffer(image_bytes, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Mostrar la imagen capturada
        cv2.imshow('Captured Image', img)
        
        # Mantener la ventana de imagen abierta sin bloquear el terminal
        while True:
            key = cv2.waitKey(1)  # Cambiar a 1 ms para no bloquear el terminal
            if key == ord('q'):  # Cerrar la imagen al presionar 'q'
                break
        
        cv2.destroyAllWindows()  # Cerrar la ventana de imagen después de salir

        # Cerrar el nodo después de mostrar la imagen
        self.get_logger().info('Closing the action client node...')
        self.destroy_node()
        rclpy.shutdown()  # Asegurarse de que ROS se cierra correctamente

def main(args=None):
    rclpy.init(args=args)

    angle_action_client = AngleActionClient()
    angle_action_client.send_goal()  # Enviar el primer goal

    rclpy.spin(angle_action_client)  # Mantener el nodo en ejecución

if __name__ == '__main__':
    main()

