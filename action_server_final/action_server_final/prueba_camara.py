import rclpy
from rclpy.node import Node
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Activating camera...')

        # Inicializa la cámara (cámara por defecto)
        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera.')
            return

        # Temporizador para capturar frames
        self.timer = self.create_timer(0.1, self.capture_frame)

    def capture_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Muestra el frame de la cámara
            cv2.imshow('Camera Feed', frame)

            # Presiona 'q' para salir
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('Shutting down camera...')
                self.cap.release()
                cv2.destroyAllWindows()
                rclpy.shutdown()

    def destroy_node(self):
        # Asegúrate de liberar la cámara al destruir el nodo
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()

    try:
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        camera_node.get_logger().info('Camera node stopped cleanly')
    except Exception as e:
        camera_node.get_logger().error(f'Exception in camera node: {e}')
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


