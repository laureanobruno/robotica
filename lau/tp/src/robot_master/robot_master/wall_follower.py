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

        self.state = FORWARD
        self.i = 0

        self.lidar_subscription = self.create_subscription(
            LaserScan, "/diff_drive/scan", self.lidar_callback, 10
        )  # Suscripción al tópico del sensor lidar
        self.odometry_subscription = self.create_subscription(
            Odometry, "/diff_drive/odometry", self.odometry_callback, 10
        )  # Suscripción al tópico del sensor odometry
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "/diff_drive/cmd_vel", 10
        )  # Publicación en el tópico de comandos de velocidad

        # Parámetros
        self.linear_speed = 0.5  # Velocidad lineal (m/s)
        self.angular_speed = 0.5  # Velocidad angular (rad/s)

        self.lidar_sensor = LidarSensor()  # Objeto para procesar los datos del lidar
        self.odometry_sensor = OdometrySensor()  # Procesa los datos del odometry

        self.timer = self.create_timer(0.1, self.MEF)

    def odometry_callback(self, msg: Odometry):
        self.lastest_odometry = msg

    def lidar_callback(self, msg: LaserScan):
        self.lastest_lidar = msg

    def MEF(self):
        twist = Twist()

        # Procesar los datos del lidar
        wall_distances: WallDistances = self.lidar_sensor.procesar(
            self.lastest_lidar.ranges
        )
        absolute_position: AbsolutePosition = self.odometry_sensor.procesar(
            self.lastest_odometry
        )

        if self.state == FORWARD:
            twist.linear.x = self.linear_speed
            if wall_distances.closest_wall == FRONT:
                self.state = TURNING
        elif self.state == TURNING:
            twist.angular.z = self.angular_speed


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
