import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from .lidar_sensor import LidarSensor, WallDistances, LEFT, RIGHT, FRONT, NONE
from .odometry_sensor import OdometrySensor, AbsolutePosition
from .map_writer import MapWriter

def angular_difference(current, target):
    """
    Calcula la diferencia mínima entre dos ángulos en radianes.
    Los ángulos se consideran en un rango de 0 a 2π.

    :param angle1: Primer ángulo en radianes (0 a 2π).
    :param angle2: Segundo ángulo en radianes (0 a 2π).
    :return: Diferencia mínima en radianes (siempre positiva).
    """
    # Normalizar los ángulos al rango [0, 2π)
    if current > 180:
        current = current - 360

    if target > 180:
        target = target - 360

    # Calcular la diferencia absoluta
    diff = target - current

    # Asegurarse de que la diferencia mínima no exceda π
    return diff


class WallFollowNode(Node):

    global FORWARD
    global TURNING_LEFT
    global TURNED
    global ADJUSTING_PARALLEL
    global INITIALIZING
    global REVERSING
    global TURNING_RIGHT
    global ADJUSTING_TURNING_RIGHT
    global FINISHED
    global GETTING_CLOSER
    FORWARD = 0
    TURNING_LEFT = 1
    ADJUSTING_PARALLEL = 2
    GETTING_CLOSER = 3
    ADJUSTING_TURNING_RIGHT = 4
    INITIALIZING = 5
    REVERSING = 6
    TURNING_RIGHT = 7
    FINISHED = 8

    global STATES_NAMES
    STATES_NAMES = {
        0: "FORWARD",
        1: "TURNING_LEFT",
        2: "ADJUSTING_PARALLEL",
        3: "GETTING_CLOSER",
        4: "ADJUSTING_TURNING_RIGHT",
        5: "INITIALIZING",
        6: "REVERSING",
        7: "TURNING_RIGHT",
        8: "FINISHED",
    }

    def __init__(self):
        super().__init__("wall_follow_node")

        self._state = INITIALIZING
        self.lastest_lidar = None
        self.lastest_odom = None
        self.lastest_right_lidar = None
        self.deg_error = 10
        self.desired_distance = 1.9
        self.i = 0

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
        self.mapped_publisher = self.create_publisher(
            Int32, "/diff_drive/mapped", 10
        )  # Publicación en el tópico de finalización

        # Parámetros
        self.linear_speed = 0.6  # Velocidad lineal (m/s)
        self.angular_speed = 0.5  # Velocidad angular (rad/s)

        self.front_lidar = LidarSensor(
            640, 3, 0, 180, (150, 180), (60, 120), (0, 30)
        )  # Procesa los datos del lidar
        self.right_lidar = LidarSensor(
            640, 2, 240, 300, (280, 300), (260, 280), (240, 260)
        )  # Procesa los datos del lidar derecho
        self.odometry_sensor = OdometrySensor()  # Procesa los datos del odometry
        self.map_writer = MapWriter()  # Escribe las coordenadas en un archivo

        if self.state != FINISHED:
            self.timer = self.create_timer(0.1, self.MEF)

    def odometry_callback(self, msg: Odometry):
        self.lastest_odom = msg

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
            absolute_position = self.odometry_sensor.procesar(
                self.lastest_odom
            )

        if self.state == INITIALIZING:
            if (self.lastest_lidar is not None and self.lastest_right_lidar is not None and self.lastest_odom is not None):
                self.state = FORWARD
            pass
        elif self.state == REVERSING:
            if abs(wall_distances.front - self.desired_distance) <= 0.2:
                self.state = TURNING_LEFT
                self.map_writer.add_coordinates(absolute_position.x, absolute_position.y, absolute_position.yaw)
                self.last_yaw = absolute_position.yaw
            else:
                twist.linear.x = -0.2
        elif self.state == FORWARD:
            if len(self.map_writer.coordinates) >= 3:
                if (absolute_position.distanceTo(self.map_writer.coordinates[0][0], self.map_writer.coordinates[0][1]) <= 1):
                    self.state = FINISHED
            if wall_distances.closest_wall == FRONT:
                if wall_distances.front < self.desired_distance:
                    self.state = REVERSING
                elif wall_distances.front > self.desired_distance:
                    self.state = GETTING_CLOSER
                else:
                    self.last_yaw = absolute_position.yaw
                    self.map_writer.add_coordinates(absolute_position.x, absolute_position.y, absolute_position.yaw)
                    self.state = TURNING_LEFT
            elif wall_distances.closest_wall == NONE and right_wall_distances.closest_wall == FRONT:
                    self.angular_speed = self.linear_speed / right_wall_distances.front
                    self.last_yaw = absolute_position.yaw
                    self.last_right_lidar = right_wall_distances
                    self.linear_speed = 0.4
                    self.coordinate_saved = False
                    self.state = TURNING_RIGHT
            else:
                if (wall_distances.front > self.front_lidar.wall_threshold):
                    twist.linear.x = 1.0
                else:
                    twist.linear.x = self.linear_speed
        elif self.state == TURNING_LEFT:
            if abs(abs(angular_difference(self.last_yaw, absolute_position.yaw)) - 90) <= self.deg_error:
                self.state = ADJUSTING_PARALLEL
            else:
                twist.angular.z = self.angular_speed
        elif self.state == ADJUSTING_PARALLEL:
            if abs(right_wall_distances.right - right_wall_distances.left) <= 0.005:
                self.angular_speed = 0.5
                self.linear_speed = 0.6
                self.state = FORWARD  # El robot está paralelo a la pared
            else:
                if (right_wall_distances.right - right_wall_distances.left) > 0:
                    twist.angular.z = 0.05
                else:
                    twist.angular.z = -0.05
        elif self.state == TURNING_RIGHT:
            if abs(abs(angular_difference(self.last_yaw, absolute_position.yaw)) - 90) <= self.deg_error:
                self.state = ADJUSTING_TURNING_RIGHT
            else:
                if (abs(abs(angular_difference(self.last_yaw, absolute_position.yaw)) - 45) <= self.deg_error) and not self.coordinate_saved:
                    self.map_writer.add_coordinates(absolute_position.x, absolute_position.y, absolute_position.yaw) # Se agregan en el medio del giro
                    self.coordinate_saved = True
                twist.angular.z = -self.angular_speed
                twist.linear.x = self.linear_speed
        elif self.state == ADJUSTING_TURNING_RIGHT:
            if self.i < 50:
                self.i += 1
            else:
                self.i = 0
                self.state = ADJUSTING_PARALLEL
            twist.linear.x = self.linear_speed
        elif self.state == GETTING_CLOSER:
            if wall_distances.front < self.desired_distance:
                self.state = REVERSING
            else:
                twist.linear.x = 0.1
        elif self.state == FINISHED:
            twist.linear.x = 0.0
            twist.angular.z = 0.0   
            self.mapped_publisher.publish(Int32(data=1))
            self.cmd_vel_publisher.publish(twist)        
            self.destroy_node()

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
