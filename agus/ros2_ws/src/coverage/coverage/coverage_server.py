import rclpy #ROS Client Library for the Python language.
import math
import fields2cover as f2c
from rclpy.node        import Node 
from std_msgs.msg      import String
from osgeo             import ogr
from enum              import Enum
import geometry_msgs.msg as gmsg
import nav_msgs.msg as navmsg
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from .odometry_sensor import *

def angular_difference_radians(angle1, angle2):
    """
    Calcula la diferencia mínima entre dos ángulos en radianes.
    Los ángulos se consideran en un rango de 0 a 2π.

    :param angle1: Primer ángulo en radianes (0 a 2π).
    :param angle2: Segundo ángulo en radianes (0 a 2π).
    :return: Diferencia mínima en radianes (siempre positiva).
    """
    # Pasar de [0, 2*pi) a [-pi, pi]
    if (angle1 > math.pi):
        angle1 = angle1 - 2*math.pi

    if (angle2 > math.pi):
        angle2 = angle2 - 2*math.pi

    # Calcular la diferencia absoluta
    diff = angle2 - angle1

    # Asegurarse de que la diferencia mínima no exceda π
    return diff

def dist(pa: gmsg.Vector3, pb: gmsg.Vector3):
    dist = math.sqrt((pb.x - pa.x)**2 + (pb.y - pa.y)**2)
    return dist

def m(pa: gmsg.Vector3, pb: gmsg.Vector3):
    m = math.atan2(pb.y - pa.y , pb.x - pa.x)
    return m

class Coverage_Type(Enum):
    BOUS = 0
    SNAKE = 1

class Movement_Type(Enum):
    POINT = 0
    ROT = 1
    POSE = 2

class CoverageServer(Node):
    path = f2c.Path;
    coverage_type = Coverage_Type.SNAKE;
    latest_pose: gmsg.Pose;
    curr_path = 0;
    curr_path_pose: gmsg.Pose;
    absolute_position: AbsolutePosition;
    mov_type: Movement_Type;

    def __init__(self):
        super().__init__('coverage_server')

        # Generate random field
        rand = f2c.Random(42);
        self.field = rand.generateRandField(1e4, 5);

        # Generate robot
        self.robot = f2c.Robot(2.0, 5.0);
        self.robot_lspeed = 1.0; #m/s
        self.robot_aspeed = 0.3; #rad/s

        # Listener de pose
        self.odometry_listener = self.create_subscription(navmsg.Odometry, "/diff_drive/odometry", self.odometry_callback, 10)
        self.latest_pose = None
        self.odometry_sensor = OdometrySensor();


        # Publisher de Twist
        self.cmd_vel_publisher = self.create_publisher(gmsg.Twist, "/diff_drive/cmd_vel", 10);

    def odometry_callback(self, msg: navmsg.Odometry):
        self.latest_pose = msg.pose.pose;
        self.absolute_position = self.odometry_sensor.procesar(msg);
        #print("Received pose ", self.latest_pose)

        if (self.mov_type == Movement_Type.POINT):
            self.drive_to_point(self.curr_path_pose, self.path[self.curr_path]);
        elif (self.mov_type == Movement_Type.ROT):
            self.drive_to_rot(self.curr_path_pose, self.path[self.curr_path]);
        elif (self.mov_type == Movement_Type.POSE):
            self.drive_to_pose(self.curr_path_pose, self.path[self.curr_path]);
        else:
            self.drive_to_pose(self.curr_path_pose, self.path[self.curr_path]);

    def generate_path(self):
        # Generate cells, add headland and calculate area
        cells = self.field.getField();

        const_hl = f2c.HG_Const_gen();
        no_hl = const_hl.generateHeadlands(cells, 3.0 * self.robot.getWidth());
        print("The complete area is ", cells.area(),
            ", and the area without headlands is ", no_hl.area());

        # Generate swaths by vrute force
        bf = f2c.SG_BruteForce();
        swaths = bf.generateSwaths(math.pi, self.robot.getCovWidth(), no_hl.getGeometry(0));

        if (self.coverage_type == Coverage_Type.BOUS):
            # Order swaths using the Boustrophedon Order
            boustrophedon_sorter = f2c.RP_Boustrophedon();
            swaths = boustrophedon_sorter.genSortedSwaths(swaths);
            print("Using Boustrophedon Order");
        elif (self.coverage_type == Coverage_Type.SNAKE):
            snake_sorter = f2c.RP_Snake();
            swaths = snake_sorter.genSortedSwaths(swaths);
            print("Using Snake Order");

        # Set robot movement constraints
        self.robot.setMinTurningRadius(2)  # m
        self.robot.setMaxDiffCurv(0.1);  # 1/m^2
        path_planner = f2c.PP_PathPlanning()

        # Conection of paths with Dubin Curves
        # Done with continous survature to avoid instant changes of direction

        dubins_cc = f2c.PP_DubinsCurvesCC();
        self.path = path_planner.planPath(self.robot, swaths, dubins_cc);

        print(self.path)

        self.curr_path_pose = self.generate_pose(self.path[0]);
        # Visualise
        f2c.Visualizer.figure();
        f2c.Visualizer.plot(self.field);
        f2c.Visualizer.plot(no_hl);
        f2c.Visualizer.plot(self.path);
        f2c.Visualizer.show();

    def generate_pose(self, path: f2c.PathState):
        pose = gmsg.Pose()
        pose.position.x = path.point.getX()
        pose.position.y = path.point.getY()
        pose.position.z = path.point.getZ()

        # Create quaternion using only pitch
        pose.orientation = gmsg.Quaternion(x=0, y=0, z=math.sin(path.angle), w=math.cos(path.angle))

        # Pose
        return pose
    
    def drive_to_pose(self, pose: gmsg.Pose, path: f2c.PathState):
        # Check if already there
        twist = gmsg.Twist();
        twist.angular = gmsg.Vector3(x=0.0, y=0.0, z=0.0);
        twist.linear = gmsg.Vector3(x=0.0, y=0.0, z=0.0);

        # Current yaw
        curr_angle = self.absolute_position.yaw;
        print("Curr angle: ", curr_angle)

        # Distance to point and angle
        linear_dist = dist(self.latest_pose.position, pose.position);
        angular_dist = angular_difference_radians(curr_angle, m(self.latest_pose.position, pose.position));
        print("Dists: ", linear_dist, " | ", angular_dist)

        # Check if already there
        if (linear_dist < 0.1):
            self.cmd_vel_publisher.publish(twist);
            self.curr_path += 1;
            self.curr_path_pose = self.generate_pose(self.path[self.curr_path]);
            print("Now going to: ", self.curr_path_pose)
            print("Press [ENTER] to continue");
            input();
            return;
        twist.linear.x = self.robot_lspeed;

        
        # Rotate to angle
        if (abs(angular_dist) > math.pi/6):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)
        elif (abs(angular_dist) > 0.001):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)*abs(6*angular_dist/math.pi);

        print("Publishing", twist)
        self.cmd_vel_publisher.publish(twist);

def main(args=None):
    rclpy.init(args=args)
    coverage_server = CoverageServer()
    
    coverage_server.generate_path();
    
    rclpy.spin(coverage_server)
    coverage_server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()