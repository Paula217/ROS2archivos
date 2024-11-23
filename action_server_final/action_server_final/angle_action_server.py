import cv2
import numpy as np
import math
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from action_definicion.action import Angulo
import base64

# Función para calcular el ángulo en grados
def calculate_angle(displacement, focal_length, object_height, distance_to_object):
    angle_rad = math.atan2(displacement * object_height, focal_length * distance_to_object)
    return math.degrees(angle_rad)

class AngleActionServer(Node):
    def __init__(self):
        super().__init__('angle_action_server')
        self._action_server = ActionServer(
            self,
            Angulo,
            'angulo',
            self.execute_callback
        )

        # Parámetros de la cámara y del objeto
        self.focal_length = 640  # Aproximación para una cámara estándar con resolución 640x480
        self.object_height = 0.22  # Altura del objeto en metros
        self.distance_to_object = 1.0  # Distancia del objeto a la cámara en metros

        # Inicializa la captura de video desde la cámara
        self.cap = cv2.VideoCapture(0)

        # Verifica si la cámara se ha abierto correctamente
        if not self.cap.isOpened():
            self.get_logger().error("Error: La cámara no se pudo abrir.")
            return

        # Configuración del flujo óptico (Lucas-Kanade)
        self.feature_params = dict(maxCorners=100, qualityLevel=0.3, minDistance=7, blockSize=7)
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

        # Lee el primer fotograma y detecta puntos de interés
        ret, old_frame = self.cap.read()
        self.old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        self.p0 = cv2.goodFeaturesToTrack(self.old_gray, mask=None, **self.feature_params)

        # Crea una máscara para dibujar el flujo óptico
        self.mask = np.zeros_like(old_frame)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        
        # Reinicia el ángulo acumulado
        self.accumulated_angle = 0.0
        
        feedback_msg = Angulo.Feedback()
        feedback_msg.current_angle = self.accumulated_angle

        target_angle = goal_handle.request.target_angle

        while True:
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error("Error: No se pudo capturar un fotograma de la cámara.")
                break

            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Calcula el flujo óptico
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_gray, frame_gray, self.p0, None, **self.lk_params)

            # Selecciona puntos que fueron correctamente rastreados
            good_new = p1[st == 1]
            good_old = self.p0[st == 1]

            # Dibuja las trayectorias
            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = int(new[0]), int(new[1])
                c, d = int(old[0]), int(old[1])
                self.mask = cv2.line(self.mask, (a, b), (c, d), (0, 255, 0), 2)
                frame = cv2.circle(frame, (a, b), 5, (0, 0, 255), -1)

            img = cv2.add(frame, self.mask)

            # Calcula el desplazamiento promedio de los puntos de interés en el eje X
            displacements = good_new[:, 0] - good_old[:, 0]
            avg_displacement = np.mean(displacements)

            # Calcula el ángulo basado en el desplazamiento
            angle_change = calculate_angle(avg_displacement, self.focal_length, self.object_height, self.distance_to_object)

            # Actualiza el ángulo acumulado sumando el cambio de ángulo
            self.accumulated_angle += angle_change

            # Publica el feedback
            feedback_msg.current_angle = self.accumulated_angle
            goal_handle.publish_feedback(feedback_msg)

            # Muestra el resultado
            cv2.putText(img, f"Angle: {self.accumulated_angle:.2f} degrees", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('Optical Flow - Angle Measurement', img)

            # Termina si se alcanza el objetivo
            if (target_angle >= 0 and self.accumulated_angle >= target_angle) or (target_angle < 0 and self.accumulated_angle <= target_angle):
                break

            # Actualiza el fotograma anterior y los puntos de interés
            self.old_gray = frame_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)

            # Termina si se presiona 'q'
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break

        # Captura la imagen cuando se alcanza el ángulo objetivo
        _, captured_frame = self.cap.read()
        _, buffer = cv2.imencode('.jpg', captured_frame)
        image_data = base64.b64encode(buffer).decode('utf-8')

        goal_handle.succeed()

        result = Angulo.Result()
        result.image_data = image_data  # Guardar la imagen en formato base64
        cv2.destroyAllWindows()
        return result

def main(args=None):
    rclpy.init(args=args)

    angle_action_server = AngleActionServer()

    rclpy.spin(angle_action_server)

if __name__ == '__main__':
    main()


