import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from .lidar_sensor import LidarSensor, WallDistances, LEFT, RIGHT, FRONT, NONE
from .odometry_sensor import OdometrySensor, AbsolutePosition


class WallFollowNode(Node):

    global FORWARD
    global TURNING
    global TURNED
    global SEARCHING
    FORWARD = 0
    TURNING = 1
    TURNED = 2
    SEARCHING = 3

    def __init__(self):
        super().__init__("wall_follow_node")

        self.state = SEARCHING
        self.lastest_lidar = None
        self.lastest_odom = None
        self.deg_error = 3

        self.lidar_subscription = self.create_subscription(
            LaserScan, "/diff_drive/scan", self.lidar_callback, 10
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
            640, 2, 0, 180, (150, 180), (60, 120), (0, 30)
        )  # Procesa los datos del lidar
        self.right_lidar = LidarSensor(
            640, 2, 225, 315, (150, 180), (60, 120), (0, 30)
        )  # Procesa los datos del lidar
        self.odometry_sensor = OdometrySensor()  # Procesa los datos del odometry

        self.timer = self.create_timer(0.1, self.MEF)

    def odometry_callback(self, msg: Odometry):
        self.lastest_odom = msg

    def lidar_callback(self, msg: LaserScan):
        self.lastest_lidar = msg

    def MEF(self):
        twist = Twist()

        # Procesar los datos del lidar
        if self.lastest_lidar is None or self.lastest_odom is None:
            return

        wall_distances: WallDistances = self.front_lidar.procesar(self.lastest_lidar)
        print(wall_distances)
        absolute_position: AbsolutePosition = self.odometry_sensor.procesar(
            self.lastest_odom
        )

        # if self.state == SEARCHING:
        #     twist.angular.z = self.angular_speed
        #     if wall_distances.closest_wall != NONE:
        #         self.state = FORWARD
        if self.state == FORWARD:
            twist.linear.x = self.linear_speed
            if wall_distances.closest_wall == FRONT:
                self.last_yaw = absolute_position.yaw
                self.state = TURNING
        elif self.state == TURNING:
            if (
                abs(absolute_position.yaw - self.last_yaw) <= 90 + self.deg_error
                and abs(absolute_position.yaw - self.last_yaw) >= 90 - self.deg_error
            ):
                self.state = FORWARD
            twist.angular.z = self.angular_speed

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
