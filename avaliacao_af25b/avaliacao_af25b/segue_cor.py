import rclpy
import numpy as np
import cv2
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge


class SegueCor(Node):
    """
    No de ACAO: segue um circulo de uma COR especifica.

    self.cor : 'vermelho' / 'verde' / 'azul'  -> qual circulo seguir
    O no expoe:
      self.cx, self.erro, self.tem_linha   -> da cor atual (self.cor)
      self.area_alvo                       -> area da cor atual na imagem
      self.area(nome)                      -> area de qualquer cor (pra detectar cruzamento)
    """

    def __init__(self):
        super().__init__('segue_cor_node')
        self.timer = None

        self.robot_state = 'done'
        self.state_machine = {
            'seguir': self.seguir,
            'stop': self.stop,
            'done': self.done,
        }

        # ===== AJUSTES RAPIDOS =====
        self.kp = 1.0
        self.v_linear = 0.10
        self.w_max = 0.4
        # ===========================

        # Faixas HSV de cada cor (AJUSTE com o tuner se precisar)
        self.faixas = {
            'vermelho': [(np.array([0, 120, 80]),   np.array([10, 255, 255])),
                         (np.array([170, 120, 80]), np.array([180, 255, 255]))],
            'verde':    [(np.array([40, 80, 60]),   np.array([85, 255, 255]))],
            'azul':     [(np.array([95, 120, 60]),  np.array([130, 255, 255]))],
        }

        self.bridge = CvBridge()
        self.twist = Twist()

        self.cor = 'vermelho'
        self.lado = None              # vies de borda (sentido), por enquanto None
        self.cx = None
        self.w = None
        self.erro = None
        self.tem_linha = False
        self.area_alvo = 0
        self.mascaras = {}            # area de cada cor no ultimo frame
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed', self.image_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))

    def mascara_cor(self, hsv, cor):
        m = None
        for lo, hi in self.faixas[cor]:
            parte = cv2.inRange(hsv, lo, hi)
            m = parte if m is None else (m | parte)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel)
        return m

    def area(self, cor):
        return self.mascaras.get(cor, 0)

    def image_callback(self, msg):
        img = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
        h, width = img.shape[:2]
        self.w = width / 2

        roi = img[h // 2:, :]                       # metade de baixo
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # area de todas as cores (pra saber quando a proxima aparece)
        self.mascaras = {}
        for cor in self.faixas:
            m = self.mascara_cor(hsv, cor)
            self.mascaras[cor] = int(cv2.moments(m)['m00'] / 255)

        # centroide da cor ATUAL
        mask = self.mascara_cor(hsv, self.cor)
        meio = mask.shape[1] // 2
        if self.lado == 'direita':
            mask[:, :meio] = 0
        elif self.lado == 'esquerda':
            mask[:, meio:] = 0

        M = cv2.moments(mask)
        self.area_alvo = int(M['m00'] / 255)
        if M['m00'] > 0:
            self.cx = int(M['m10'] / M['m00'])
            self.erro = -(self.cx - self.w) / self.w
            self.tem_linha = True
            cv2.circle(roi, (self.cx, roi.shape[0] // 2), 8, (0, 255, 255), -1)
        else:
            self.cx = None
            self.erro = None
            self.tem_linha = False

        cv2.imshow('SegueCor', img)
        cv2.imshow('Mask', mask)
        cv2.waitKey(1)

    def reset(self):
        self.twist = Twist()
        self.robot_state = 'seguir'
        if self.timer is None:
            self.timer = self.create_timer(0.1, self.control)

    def seguir(self):
        if self.erro is None:               # perdeu a cor -> anda devagar reto
            self.twist.linear.x = 0.05
            self.twist.angular.z = 0.0
            return
        w = self.kp * self.erro
        w = max(-self.w_max, min(self.w_max, w))
        self.twist.linear.x = self.v_linear
        self.twist.angular.z = float(w)

    def stop(self):
        self.twist = Twist()
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    def control(self):
        self.twist = Twist()
        self.state_machine[self.robot_state]()
        self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = SegueCor()
    rclpy.spin_once(ros_node, timeout_sec=1)
    ros_node.reset()
    while not ros_node.robot_state == 'done':
        rclpy.spin_once(ros_node, timeout_sec=1)
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
