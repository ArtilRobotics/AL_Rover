import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from motor_controller.msg import MotorCommand, MotorState

class OmnidirectionalController(Node):
    def __init__(self):
        super().__init__('omni_motor_controller')

        # Constantes del robot
        self.r = 0.1
        self.L_plus_W = 1.0

        # Estados de control del rover
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0
        self.delta_altura = 0.0

        # Suscripciones a velocidades deseadas y posicion de z
        self.create_subscription(Float32, 'rover/vx', self.vx_callback, 10)
        self.create_subscription(Float32, 'rover/vy', self.vy_callback, 10)
        self.create_subscription(Float32, 'rover/w', self.w_callback, 10)
        self.create_subscription(Float32, 'rover/pos_z', self.pos_z_callback, 10)

        # Publicadores para motores ruedas
        self.lower_motors = ['FL', 'FR', 'RL', 'RR']
        self.lower_publishers = {
            m: self.create_publisher(MotorCommand, f'motors/{m}_lower/cmd', 10)
            for m in self.lower_motors
        }

        # Publicadores para motores hombros
        self.upper_motors = ['FL', 'FR', 'RL', 'RR']
        self.upper_publishers = {
            m: self.create_publisher(MotorCommand, f'motors/{m}_upper/cmd', 10)
            for m in self.upper_motors
        }

        # Referencias de posición y signos (Para delta z)
        self.pos_ref = {m: 0.0 for m in self.upper_motors}
        self.pos_sign = {'FL': 1.0, 'FR': -1.0, 'RL': 1.0, 'RR': -1.0} 

        self.timer = self.create_timer(0.05, self.publish_commands)  #20 Hz

        self.get_logger().info("Controlador rover inicializado.")

    # Callbacks para leer deseados (velocidad y posicion)
    def vx_callback(self, msg): self.vx = msg.data
    def vy_callback(self, msg): self.vy = msg.data
    def w_callback(self, msg): self.omega = msg.data
    def pos_z_callback(self, msg): self.delta_altura = msg.data

    def publish_commands(self):

         # -------- Control para Ruedas --------
        vx, vy, w = self.vx, self.vy, self.omega
        r, L_plus_W = self.r, self.L_plus_W

        w1 = (1 / r) * (vx - vy - L_plus_W * w)
        w2 = (1 / r) * (vx + vy + L_plus_W * w)
        w3 = (1 / r) * (vx + vy - L_plus_W * w)
        w4 = (1 / r) * (vx - vy + L_plus_W * w)

        speeds = {'FL': w1, 'FR': -w2, 'RL': w3, 'RR': -w4}
        for m, spd in speeds.items():
            cmd = MotorCommand()
            cmd.speed_desired = spd
            cmd.k_spd = 0.4
            cmd.torque_desired = 0.0
            cmd.position_desired = 0.0
            cmd.k_pos = 0.0
            self.lower_publishers[m].publish(cmd)

        # -------- Control para Hombros --------
        delta = self.delta_altura
        if abs(delta) > 1e-3:
            for m in self.upper_motors:
                signo = self.pos_sign[m]
                nueva_pos = self.pos_ref[m] + delta * signo
                self.pos_ref[m] = nueva_pos

                cmd = MotorCommand()
                cmd.position_desired = nueva_pos * self.GEAR_RATIO
                cmd.k_pos = 60.0 / (self.GEAR_RATIO ** 2)
                cmd.k_spd = 5.0 / (self.GEAR_RATIO ** 2)
                cmd.torque_desired = 0.0
                cmd.speed_desired = 0.0

                self.upper_publishers[m].publish(cmd)
                self.get_logger().info(f'[Posición] {m} → {nueva_pos:.3f} rad')
               

def main(args=None):
    rclpy.init(args=args)
    node = OmnidirectionalController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
