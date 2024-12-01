import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .lidar_sensor import LidarSensor, WallDistances, LEFT, RIGHT, FRONT, NONE
from .odometry_sensor import OdometrySensor, AbsolutePosition
import math


def angular_difference(angle1, angle2):
    """
    Calcula la diferencia mínima entre dos ángulos en grados.
    Los ángulos se consideran en un rango de 0 a 360 grados.

    :param angle1: Primer ángulo en grados (0 a 360).
    :param angle2: Segundo ángulo en grados (0 a 360).
    :return: Diferencia mínima en grados (siempre positiva).
    """
    # Normalizar los ángulos al rango [0, 360)
    angle1 = angle1 % 360
    angle2 = angle2 % 360

    # Calcular la diferencia absoluta
    diff = abs(angle1 - angle2)

    # Asegurarse de que la diferencia mínima no exceda 180°
    return min(diff, 360 - diff)


class WallFollowNode(Node):

    global FORWARD
    global TURNING
    global TURNED
    global SEARCHING
    global ADJUSTING_PARALLEL
    global INITIALIZING
    global REVERSING
    FORWARD = 0
    TURNING = 1
    TURNED = 2
    SEARCHING = 3
    ADJUSTING_PARALLEL = 4
    INITIALIZING = 5
    REVERSING = 6

    global STATES_NAMES
    STATES_NAMES = {
        0: "FORWARD",
        1: "TURNING",
        2: "TURNED",
        3: "SEARCHING",
        4: "ADJUSTING_PARALLEL",
        5: "INITIALIZING",
        6: "REVERSING",
    }

    def __init__(self):
        super().__init__("wall_follow_node")

        self._state = INITIALIZING
        self.lastest_lidar = None
        self.lastest_odom = None
        self.lastest_right_lidar = None
        self.cant_odom = 0
        self.deg_error = 3
        self.desired_distance = 2

        self.lidar_subscription = self.create_subscription(
            LaserScan, "/diff_drive/scan", self.front_lidar_callback, 10
        )  # Suscripción al tópico del sensor lidar
        self.right_lidar_subscription = self.create_subscription(
            LaserScan, "/diff_drive/rscan", self.right_lidar_callback, 10
        )  # Suscripción al tópico del sensor lidar
        self.odometry_subscription = self.create_subscription(
            Odometry, "/diff_drive/odometry", self.odometry_callback, 10
        )  # Suscripción al tópico del sensor odometry
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/diff_drive/cmd_vel", 10
        )  # Publicación en el tópico de comandos de velocidad

        # Parámetros
        self.linear_speed = 0.5  # Velocidad lineal (m/s)
        self.angular_speed = 0.1  # Velocidad angular (rad/s)

        self.front_lidar = LidarSensor(
            640, 3, 0, 180, (150, 180), (60, 120), (0, 30)
        )  # Procesa los datos del lidar
        self.right_lidar = LidarSensor(
            640, 2, 225, 315, (285, 315), (255, 285), (225, 255)
        )  # Procesa los datos del lidar derecho
        self.odometry_sensor = OdometrySensor()  # Procesa los datos del odometry

        self.timer = self.create_timer(1, self.MEF)

    def odometry_callback(self, msg: Odometry):
        self.lastest_odom = msg
        if self.cant_odom < self.odometry_sensor.max_samples:
            self.cant_odom += 1

    def front_lidar_callback(self, msg: LaserScan):
        self.lastest_lidar = msg

    def right_lidar_callback(self, msg: LaserScan):
        self.lastest_right_lidar = msg

    @property
    def state(self):
        return self._state

    @state.setter
    def state(self, new_state):
        if self._state != new_state:  # Solo log si hay un cambio de estado
            self.get_logger().info(
                f"State changed from {STATES_NAMES[self._state]} to {STATES_NAMES[new_state]}"
            )
        self._state = new_state  # Asignar directamente a la variable privada

    def MEF(self):
        twist = Twist()

        if self.state != INITIALIZING:
            wall_distances: WallDistances = self.front_lidar.procesar(
                self.lastest_lidar
            )
            right_wall_distances: WallDistances = self.right_lidar.procesar(
                self.lastest_right_lidar
            )
            absolute_position: AbsolutePosition = self.odometry_sensor.procesar(
                self.lastest_odom
            )

        if self.state == INITIALIZING:
            if (
                self.lastest_lidar is not None
                and self.cant_odom >= self.odometry_sensor.max_samples
            ):
                self.state = SEARCHING
            pass
        elif self.state == SEARCHING:
            if wall_distances.closest_wall == RIGHT:
                self.state = ADJUSTING_PARALLEL
            elif wall_distances.closest_wall == FRONT:
                if wall_distances.front < self.desired_distance:
                    self.state = REVERSING
                else:
                    self.last_yaw = absolute_position.yaw
                    self.state = TURNING
        elif self.state == REVERSING:
            if abs(wall_distances.front - self.desired_distance) <= 0.2:
                self.last_yaw = absolute_position.yaw
                self.state = TURNING
            else:
                twist.linear.x = -0.1
        elif self.state == FORWARD:
            twist.linear.x = self.linear_speed
            if wall_distances.closest_wall == FRONT:
                if wall_distances.front < self.desired_distance:
                    self.state = REVERSING
                else:
                    self.last_yaw = absolute_position.yaw
                    self.state = TURNING
        elif self.state == TURNING:
            print(self.last_yaw, absolute_position.yaw)
            if (
                angular_difference(self.last_yaw, absolute_position.yaw)
                >= 80 - self.deg_error
                and angular_difference(self.last_yaw, absolute_position.yaw)
                <= 80 + self.deg_error
            ):
                self.state = ADJUSTING_PARALLEL
            else:
                twist.angular.z = self.angular_speed
        elif self.state == ADJUSTING_PARALLEL:
            print(right_wall_distances.right, right_wall_distances.left)
            if abs(right_wall_distances.right - right_wall_distances.left) <= 0.01:
                self.state = FORWARD  # El robot está paralelo a la pared
            elif right_wall_distances.right > right_wall_distances.left:
                twist.angular.z = 0.05
            else:
                twist.angular.z = -0.05

        # Publicar el comando de velocidad
        self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
