import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from robcomp_interfaces.msg import MudaPista
from robcomp_util.odom import Odom
from avaliacao_af25b.segue_cor import SegueCor


class MudaPista(Node, Odom):
    """
    Exercicio 1 - AF 25b (VERSAO SIMPLES, ignora o sentido).
    Segue o circulo da cor atual; quando o Orquestrador manda mudar, troca
    pra proxima cor (vermelho -> verde -> azul) assim que ela aparece. No
    azul, ignora comandos, da uma volta completa, volta ao inicio e manda
    READY de novo. control() e a unica que publica /cmd_vel.
    """

    def __init__(self):
        super().__init__('muda_pista_node')
        Odom.__init__(self)
        rclpy.spin_once(self)

        self.segue = SegueCor()

        self.robot_state = 'ready'
        self.state_machine = {
            'ready':         self.ready,
            'espera':        self.espera,
            'segue':         self.segue_estado,
            'azul_volta':    self.azul_volta,
            'stop':          self.stop,
            'done':          self.done,
        }
        self.estados_clientes = ['segue', 'azul_volta']
        self.twist = Twist()

        # ===== AJUSTES RAPIDOS =====
        self.nome = 'Henrique Abreu'
        self.area_troca = 1500      # area da proxima cor pra considerar "cheguei no cruzamento"
        self.dist_volta = 1.0       # m: afastou isso do inicio do azul...
        self.dist_fim = 0.4         # m: ...e voltou pra ca = volta completa
        # ===========================

        self.ordem = ['vermelho', 'verde', 'azul']
        self.idx = 0                # cor atual
        self.mudar_pedido = False   # recebeu comando de mudar?
        self.x0 = self.y0 = None    # inicio da volta no azul
        self.afastou = False

        self.create_subscription(MudaPista, '/controle', self.controle_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.controle_pub = self.create_publisher(MudaPista, '/controle', 10)

        self.timer = self.create_timer(0.1, self.control)

    # ================= CALLBACK =================
    def controle_callback(self, msg: MudaPista):
        if msg.status in ('MUDAR_HORARIO', 'MUDAR_ANTIHORARIO'):
            self.mudar_pedido = True
            self.get_logger().info(f'Orquestrador: {msg.status}')

    # ================= HELPERS =================
    def publica_ready(self):
        msg = MudaPista()
        msg.status = 'READY'
        msg.aluno = self.nome
        msg.timestamp = self.get_clock().now().to_msg()
        self.controle_pub.publish(msg)
        self.get_logger().info('READY enviado ao Orquestrador')

    def roda_segue(self, cor):
        self.segue.cor = cor
        if self.segue.robot_state == 'done':
            rclpy.spin_once(self.segue)
            self.segue.reset()
        rclpy.spin_once(self.segue)

    def para_segue(self):
        self.segue.robot_state = 'stop'
        rclpy.spin_once(self.segue)

    def dist_de(self, x, y):
        return np.hypot(self.x - x, self.y - y)

    # ================= ESTADOS =================
    def ready(self):
        self.publica_ready()
        self.idx = 0
        self.mudar_pedido = False
        self.robot_state = 'espera'

    def espera(self):
        self.twist = Twist()                # parado esperando comando
        if self.mudar_pedido:
            self.mudar_pedido = False
            self.robot_state = 'segue'      # entra na pista vermelha

    def segue_estado(self):
        cor_atual = self.ordem[self.idx]
        self.roda_segue(cor_atual)

        # recebeu comando de mudar e nao estamos no azul -> troca quando a proxima cor aparece
        if self.mudar_pedido and cor_atual != 'azul':
            prox = self.ordem[self.idx + 1]
            if self.segue.area(prox) > self.area_troca:
                self.idx += 1
                self.mudar_pedido = False
                self.get_logger().info(f'Mudando para a pista {self.ordem[self.idx]}')
                if self.ordem[self.idx] == 'azul':      # entrou no azul -> conta a volta
                    self.x0, self.y0 = self.x, self.y
                    self.afastou = False
                    self.robot_state = 'azul_volta'

    def azul_volta(self):
        self.roda_segue('azul')             # no azul ignora comandos
        self.mudar_pedido = False
        d = self.dist_de(self.x0, self.y0)
        if d > self.dist_volta:
            self.afastou = True
        if self.afastou and d < self.dist_fim:
            self.para_segue()
            self.get_logger().info('Volta no azul completa, voltando ao ciclo')
            self.robot_state = 'ready'

    def stop(self):
        self.twist = Twist()
        self.robot_state = 'done'

    def done(self):
        self.twist = Twist()

    # ================= CONTROL (IDENTICO ao base_control) =================
    def control(self):
        print(f'Estado Atual: {self.robot_state}')
        self.state_machine[self.robot_state]()
        if self.robot_state not in self.estados_clientes:
            self.cmd_vel_pub.publish(self.twist)


def main(args=None):
    rclpy.init(args=args)
    ros_node = MudaPista()
    while rclpy.ok():
        rclpy.spin_once(ros_node)
        if ros_node.robot_state == 'done':
            ros_node.cmd_vel_pub.publish(Twist())
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
