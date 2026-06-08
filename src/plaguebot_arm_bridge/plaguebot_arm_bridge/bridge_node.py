#!/usr/bin/env python3
import math
import time
import threading
import serial
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class ArmBridgeNode(Node):
    def __init__(self):
        super().__init__('plaguebot_arm_bridge')

        # Parametros
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('reconnect_attempts', 5)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('interpolation_hz', 20)
        self.declare_parameter('publish_joint_states', True)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.reconnect_attempts = self.get_parameter('reconnect_attempts').value
        self.reconnect_delay_sec = self.get_parameter('reconnect_delay_sec').value
        self.interp_hz = self.get_parameter('interpolation_hz').value
        self.publish_js = self.get_parameter('publish_joint_states').value

        # Mapeo joints (orden del JointTrajectory del arm_controller)
        self.joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        self.joint_to_channel = {
            'joint_1': 6, 'joint_2': 3, 'joint_3': 4,
            'joint_4': 2, 'joint_5': 1, 'joint_6': 0
        }

        self.joint_inverted = {
            'joint_1': False, 'joint_2': False, 'joint_3': False,
            'joint_4': False, 'joint_5': False, 'joint_6': False
        }

        self.joint_offsets_deg = {
            'joint_1': 0.0, 'joint_2': 0.0, 'joint_3': 0.0,
            'joint_4': 0.0, 'joint_5': 0.0, 'joint_6': 0.0
        }

        # Estado actual de los joints (radianes)
        self.current_positions = {name: 0.0 for name in self.joint_names}
        self.target_positions = {name: 0.0 for name in self.joint_names}

        # Serial
        self.ser = None
        self.serial_ok = False
        self.serial_lock = threading.Lock()
        self._connect_serial()

        # Publishers
        self.js_pub = self.create_publisher(JointState, '/arm_bridge/joint_states', 10)

        self.status_pub = self.create_publisher(String, '/arm_bridge/status', 10)

        # Subscriber al arm_controller
        self.js_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10
        )
        self._last_js_time = 0.0

        # Timer de publicacion de joint_states a 50Hz
        self.js_timer = self.create_timer(0.02, self._publish_joint_states)

        self.get_logger().info(
            f'Arm bridge iniciado. Puerto: {self.serial_port} '
            f'Serial: {"OK" if self.serial_ok else "DESCONECTADO (modo simulacion)"}'
        )

        msg = String()
        msg.data = 'READY' if self.serial_ok else 'SIM_MODE'
        self.status_pub.publish(msg)

    # --- Serial ---

    def _connect_serial(self):
        for attempt in range(self.reconnect_attempts):
            try:
                self.ser = serial.Serial(
                    self.serial_port,
                    self.baud_rate,
                    timeout=1.0
                )
                time.sleep(1.5)
                self.ser.reset_input_buffer()
                # Verificar con PING
                self.ser.write(b'PING\n')
                time.sleep(0.3)
                resp = self.ser.read_all().decode(errors='ignore')
                if 'PONG' in resp:
                    self.serial_ok = True
                    self.get_logger().info('ESP32 conectado y respondiendo PONG')
                    return
                else:
                    self.get_logger().warn(f'ESP32 no respondio PONG (intento {attempt+1})')
            except Exception as e:
                self.get_logger().warn(f'Error serial intento {attempt+1}: {e}')
                time.sleep(self.reconnect_delay_sec)

        self.serial_ok = False
        self.get_logger().warn('ESP32 no disponible. Modo simulacion activado.')

    def _send_servo(self, channel: int, angle_deg: float):
        cmd = f'J{channel},{angle_deg:.1f}\n'.encode()
        with self.serial_lock:
            if not self.serial_ok or self.ser is None:
                return False
            try:
                self.ser.write(cmd)
                return True
            except Exception as e:
                self.get_logger().error(f'Error serial al enviar: {e}')
                self.serial_ok = False
                self._attempt_reconnect()
                return False

    def _attempt_reconnect(self):
        self.get_logger().warn('Perdida de conexion. Intentando reconectar...')
        msg = String()
        msg.data = 'RECONNECTING'
        self.status_pub.publish(msg)
        self._connect_serial()
        if not self.serial_ok:
            self.get_logger().error('Reconexion fallida. Deteniendo movimiento.')
            msg.data = 'ERROR_DISCONNECTED'
            self.status_pub.publish(msg)

    # --- Conversiones ---

    def _rad_to_deg(self, joint_name: str, rad: float) -> float:
        deg = math.degrees(rad)
        if self.joint_inverted.get(joint_name, False):
            deg = -deg
        deg += self.joint_offsets_deg.get(joint_name, 0.0)
        return max(-90.0, min(90.0, deg))

    def _send_joint(self, joint_name: str, angle_deg: float):
        channel = self.joint_to_channel.get(joint_name)
        if channel is None:
            return
        self._send_servo(channel, angle_deg)

        # Manejo del joint_3 doble (canales 4 y 5 en espejo)
        if joint_name == 'joint_3':
            self._send_servo(5, -angle_deg)

    # --- Callback joint states ---

    def _joint_states_callback(self, msg: JointState):
        # Diagnostico: loggear cada 100 mensajes recibidos
        if not hasattr(self, '_js_count'):
            self._js_count = 0
        self._js_count += 1
        if self._js_count % 100 == 1:
            self.get_logger().info(
                f'joint_states recibido #{self._js_count}, '
                f'joints: {msg.name[:3]}..., '
                f'pos[0]: {f"{msg.position[0]:.3f}" if msg.position else "N/A"} rad'
            )

        now = time.time()
        # Limitar a interpolation_hz para no saturar el ESP32
        if (now - self._last_js_time) < (1.0 / self.interp_hz):
            return
        self._last_js_time = now

        for i, name in enumerate(msg.name):
            if name not in self.joint_names:
                continue
            if i >= len(msg.position):
                continue
            rad = msg.position[i]
            angle_deg = self._rad_to_deg(name, rad)
            self._send_joint(name, angle_deg)
            self.current_positions[name] = rad

    # --- Joint States ---

    def _publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = [self.current_positions[n] for n in self.joint_names]
        self.js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Bridge detenido.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
