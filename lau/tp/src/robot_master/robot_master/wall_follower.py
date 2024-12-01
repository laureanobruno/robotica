import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WallFollowNode(Node):
    def __init__(self):
        super().__init__("wall_follow_node")
        self.lidar_subscription = self.create_subscription(
            LaserScan, "/diff_drive/scan", self.lidar_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/diff_drive/cmd_vel", 10)
        self.min_distance_to_wall = 0.1  # Distancia deseada a la pared (en metros)
        self.linear_speed = 0.2  # Velocidad lineal (m/s)
        self.angular_speed = 0.5  # Velocidad angular (rad/s)

    def lidar_callback(self, msg: LaserScan):
        # Dividimos el campo de visión en tres zonas

        # print("\n\n Ranges: ", msg.ranges)

        front_ranges = [
            r for r in (msg.ranges[0:10] + msg.ranges[-10:]) if r < float("inf")
        ]
        left_ranges = [r for r in msg.ranges[80:100] if r < float("inf")]
        right_ranges = [r for r in msg.ranges[-100:-80] if r < float("inf")]

        # Print for debugging
        # print("\n\nFront: ", front_ranges)
        # print("\n\nLeft: ", left_ranges)
        # print("\n\nRight: ", right_ranges)

        # Calculamos distancias promedio a la pared en cada zona
        front_distance = min(front_ranges, default=float("inf"))
        left_distance = min(left_ranges, default=float("inf"))
        right_distance = min(right_ranges, default=float("inf"))

        print("\n\nFront distance: ", front_distance)
        print("Left distance: ", left_distance)
        print("Right distance: ", right_distance)

        twist = Twist()

        # Lógica para seguir la pared
        if front_distance < self.min_distance_to_wall:
            # Si hay una pared enfrente, girar a la izquierda
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed
        elif left_distance > self.min_distance_to_wall:
            # Si la pared izquierda está lejos, girar a la izquierda
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed
        elif left_distance < self.min_distance_to_wall:
            # Si la pared izquierda está cerca, girar a la derecha
            twist.linear.x = self.linear_speed
            twist.angular.z = -self.angular_speed
        else:
            # Continuar recto
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        # Publicar el comando de velocidad
        # self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
