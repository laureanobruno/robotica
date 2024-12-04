import rclpy #ROS Client Library for the Python language.
import math
import fields2cover as f2c
from rclpy.node        import Node 
from std_msgs.msg      import String, Int32
from osgeo             import ogr
from enum              import Enum
import geometry_msgs.msg as gmsg
import nav_msgs.msg as navmsg
import numpy as np
from scipy.spatial.transform import Rotation as R
from .odometry_sensor import *

def angular_difference_radians(angle1, angle2):
    # Pasar de [0, 2*pi) a [-pi, pi]
    if (angle1 > math.pi):
        angle1 = angle1 - 2*math.pi

    if (angle2 > math.pi):
        angle2 = angle2 - 2*math.pi

    # Calcular la diferencia absoluta
    diff = angle2 - angle1

    if (diff > math.pi):
        diff = diff - 2*math.pi;

    # Asegurarse de que la diferencia mínima no exceda π
    return diff

def dist(pa: gmsg.Vector3, pb: gmsg.Vector3):
    dist = math.sqrt((pb.x - pa.x)**2 + (pb.y - pa.y)**2)
    return dist

def m(pa: gmsg.Vector3, pb: gmsg.Vector3):
    m = math.atan2(pb.y - pa.y , pb.x - pa.x)
    return m

def readFromFile():
    try:
        f = open("map.txt", "r");
        pairs = []
        for line in f:
            # Strip any leading/trailing whitespaces and split by space (or any other delimiter)
            # assuming each line contains two real numbers separated by space
            numbers = line.strip().split()
            if len(numbers) == 3:
                try:
                    # Convert the strings to float and store as a tuple in the array
                    num1 = float(numbers[0])
                    num2 = float(numbers[1])
                    num3 = float(numbers[2])
                    pairs.append([num1, num2, num3])
                except ValueError:
                    print(f"Invalid number format in line: {line.strip()}")
            else:
                print(f"Skipping invalid line: {line.strip()}")
        f.close()
        pairs.pop(0); # el primero no es valido
        return pairs;
    except FileNotFoundError:
        print(f"The file map.txt was not found.")

def linearisePoints(points):
    # Compensate for safe distance from wall
    for point in points:
        angle = round(point[2]/30);
        if angle == 0: # 0°
            point[0] = round(point[0]) + 2
            point[1] = round(point[1]) - 2
        elif angle == 3: # 90°
            point[0] = round(point[0]) + 1 # 2?
            point[1] = round(point[1]) + 2
        elif angle == 6 or angle == -6: # +/-180°
            point[0] = round(point[0]) - 2
            point[1] = round(point[1]) + 2
        elif angle == -3: # -90°
            point[0] = round(point[0]) - 2
            point[1] = round(point[1]) - 2
        elif angle == -1: # giros convexos
            point[0] = round(point[0]) - 1
            point[1] = round(point[1]) - 2
        elif angle == 1:
            point[0] = round(point[0]) + 1
            point[1] = round(point[1]) + 2 
        elif angle == 2:
            point[0] = round(point[0]) + 2
            point[1] = round(point[1]) - 1
        elif angle == -2:
            point[0] = round(point[0]) - 2
            point[1] = round(point[1]) + 1
        # reflect x
        #point[0] = -point[1];
        #point[1] = -point[0];
        

def fieldFromPoints(points):
    field = f2c.Field();
    vpoints = f2c.VectorPoint();
    for point in  points:
        vpoints.append(f2c.Point(point[0], point[1]));
    vpoints.append(f2c.Point(points[0][0], points[0][1]));
    field.setField(f2c.Cells(f2c.Cell(f2c.LinearRing(vpoints))))
    return field;

class Coverage_Type(Enum):
    BOUS = 0
    SNAKE = 1

class Movement_Type(Enum):
    POINT = 0
    ROT = 1
    POSE = 2
    ROT_M = 3

class CoverageServer(Node):
    path = f2c.Path;
    coverage_type = Coverage_Type.BOUS;
    latest_pose: gmsg.Pose;
    curr_path = 0;
    curr_path_pose: gmsg.Pose;
    absolute_position: AbsolutePosition;
    mov_type: Movement_Type;
    run = 0;

    def __init__(self):
        super().__init__('coverage_server')

        # Generate random field
        """self.field = f2c.Field(f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint([
            f2c.Point(-9, 0),
            f2c.Point(-10, 10),
            f2c.Point(-20, 10),
            f2c.Point(-20, 20),
            f2c.Point(10, 20),
            f2c.Point(10, 10),
            f2c.Point(0, 10),
            f2c.Point(0, 0),
            f2c.Point(-10, 0)
        ])))));"""

        # Generate robot
        self.robot = f2c.Robot(1.0, 1.0);
        self.robot_lspeed = 1.5; #m/s
        self.robot_aspeed = 0.3; #rad/s

        # Listener de pose
        self.odometry_listener = self.create_subscription(navmsg.Odometry, "/diff_drive/odometry", self.odometry_callback, 10)
        self.odometry_sensor = OdometrySensor();
        self.mov_type = Movement_Type.ROT_M;

        self.start_listener = self.create_subscription(Int32, "/diff_drive/mapped", self.start_callback, 10);

        # Publisher de Twist
        self.cmd_vel_publisher = self.create_publisher(gmsg.Twist, "/diff_drive/cmd_vel", 10);
                
    def start_callback(self, msg: Int32):
        if (msg.data == 1):
            points = readFromFile();
            linearisePoints(points);
            self.field = fieldFromPoints(points);
            self.generate_path();
            self.run = 1;
            print("Press [ENTER] to begin the coverage")

    def odometry_callback(self, msg: navmsg.Odometry):
        if (self.run == 1):
            self.latest_pose = msg.pose.pose;
            self.absolute_position = self.odometry_sensor.procesar(msg);
            #print("Received pose ", self.latest_pose)

            if (self.mov_type == Movement_Type.POINT):
                self.drive_to_point(self.curr_path_pose, self.path[self.curr_path]);
            elif (self.mov_type == Movement_Type.ROT):
                self.drive_to_rot(self.curr_path_pose, self.path[self.curr_path]);
            elif (self.mov_type == Movement_Type.POSE):
                self.drive_to_pose(self.curr_path_pose, self.path[self.curr_path]);
            elif ((self.mov_type == Movement_Type.ROT_M)):
                self.drive_to_rot_m(self.curr_path_pose, self.path[self.curr_path])
            else:
                print("ERROR: mov_type not defined");

    def generate_path(self):
        # Generate cells, add headland and calculate area
        cells = self.field.getField();

        const_hl = f2c.HG_Const_gen();
        no_hl = const_hl.generateHeadlands(cells, 2.0 * self.robot.getWidth());
        print("The complete area is ", cells.area(),
            ", and the area without headlands is ", no_hl.area());

        # Generate swaths by vrute force
        bf = f2c.SG_BruteForce();
        # swaths = bf.generateSwaths(math.pi/4, self.robot.getCovWidth(), no_hl.getGeometry(0));
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
        self.robot.setMinTurningRadius(0)  # m
        self.robot.setMaxDiffCurv(10);  # 1/m^2
        path_planner = f2c.PP_PathPlanning()

        # Conection of paths with Dubin Curves
        # Done without continous survature since it's a closed space

        dubins_cc = f2c.PP_DubinsCurves();
        self.path = path_planner.planPath(self.robot, swaths, dubins_cc);

        # print(self.path)

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
    
    def drive_to_rot(self, pose: gmsg.Pose, path: f2c.PathState):
        # Check if already there
        twist = gmsg.Twist();
        twist.angular = gmsg.Vector3(x=0.0, y=0.0, z=0.0);
        twist.linear = gmsg.Vector3(x=0.0, y=0.0, z=0.0);

        # Current yaw
        curr_angle = self.absolute_position.yaw;
        # print("Curr angle: ", curr_angle)

        # Distance to angle
        angular_dist = angular_difference_radians(curr_angle, path.angle);
        # print("Dists: ", angular_dist)
        
        # Rotate to angle
        if (abs(angular_dist) > math.pi/6):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)
        elif (abs(angular_dist) > 0.001):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)*abs(6*angular_dist/math.pi);
        else:
            self.cmd_vel_publisher.publish(twist);
            # self.curr_path += 1;
            # self.curr_path_pose = self.generate_pose(self.path[self.curr_path]);
            # print("Now going to: ", self.curr_path_pose)
            # print("Press [ENTER] to continue");
            self.mov_type = Movement_Type.POINT;
            return            

        self.cmd_vel_publisher.publish(twist);

    def drive_to_pose(self, pose: gmsg.Pose, path: f2c.PathState):
        # Check if already there
        twist = gmsg.Twist();
        twist.angular = gmsg.Vector3(x=0.0, y=0.0, z=0.0);
        twist.linear = gmsg.Vector3(x=0.0, y=0.0, z=0.0);

        # Current yaw
        curr_angle = self.absolute_position.yaw;
        #print("Curr angle: ", curr_angle)

        # Distance to point and angle
        linear_dist = dist(self.latest_pose.position, pose.position);
        angular_dist = angular_difference_radians(curr_angle, path.angle);
        print("Dists: ", linear_dist, " | ", angular_dist)

        # Check if already there
        if (linear_dist < 0.8):
            self.cmd_vel_publisher.publish(twist);
            self.curr_path += 1;
            self.curr_path_pose = self.generate_pose(self.path[self.curr_path]);
            print("Now going to: ", self.curr_path_pose)
            #print("Press [ENTER] to continue");
            #input();
            return;
        elif (linear_dist < 3):
            twist.linear.x = self.robot_lspeed * (linear_dist) * 0.2 * 0.3;
        else:
            twist.linear.x = self.robot_lspeed;
        
        # Rotate to angle
        if (abs(angular_dist) > math.pi/6):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)
        elif (abs(angular_dist) > 0.005):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)*abs(6*angular_dist/math.pi);

        # print("Publishing", twist)
        self.cmd_vel_publisher.publish(twist);
    
    def drive_to_point(self, pose: gmsg.Pose, path: f2c.PathState):
        # Check if already there
        twist = gmsg.Twist();
        twist.angular = gmsg.Vector3(x=0.0, y=0.0, z=0.0);
        twist.linear = gmsg.Vector3(x=0.0, y=0.0, z=0.0);

        # Current yaw
        curr_angle = self.absolute_position.yaw;
        # print("Curr angle: ", curr_angle)

        # Distance to point and angle
        linear_dist = dist(self.latest_pose.position, pose.position);

        # print("From position ", self.latest_pose.position, " to position ", pose.position)

        # m_angle = m(self.latest_pose.position, pose.position);
        # print("M: ", m_angle)
        # angular_dist = angular_difference_radians(curr_angle, m_angle);
        # print("Dists: ", linear_dist, " | ", angular_dist)
        
        # Check if already there
        if (linear_dist < 0.8):
            self.cmd_vel_publisher.publish(twist);
            self.curr_path += 1;
            self.curr_path_pose = self.generate_pose(self.path[self.curr_path]);
            # print("Now going to: ", self.curr_path_pose)
            # print("Press [ENTER] to continue");
            self.mov_type = Movement_Type.ROT;
            return;
        elif (linear_dist < 2):
            twist.linear.x = self.robot_lspeed * (linear_dist) * 0.2;
        else:
            twist.linear.x = self.robot_lspeed;

        
        # # Rotate to angle
        # if (abs(angular_dist) > math.pi/10):
        #     twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)
        # elif (abs(angular_dist) > 0.005):
        #     twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)*abs(10*angular_dist/math.pi);

        # print("Publishing", twist)
        self.cmd_vel_publisher.publish(twist);

    def drive_to_rot_m(self, pose: gmsg.Pose, path: f2c.PathState):
        # Check if already there
        twist = gmsg.Twist();
        twist.angular = gmsg.Vector3(x=0.0, y=0.0, z=0.0);
        twist.linear = gmsg.Vector3(x=0.0, y=0.0, z=0.0);

        # Current yaw
        curr_angle = self.absolute_position.yaw;
        # print("Curr angle: ", curr_angle)

        # Distance to angle
        angular_dist = angular_difference_radians(curr_angle, m(self.latest_pose.position, pose.position));
        # print("Dists: ", angular_dist)
        
        # Rotate to angle
        if (abs(angular_dist) > math.pi/6):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)
        elif (abs(angular_dist) > 0.01):
            twist.angular.z = math.copysign(self.robot_aspeed, angular_dist)*abs(6*angular_dist/math.pi);
        else:
            self.cmd_vel_publisher.publish(twist);
            # print("Now going to: ", self.curr_path_pose)
            # print("Press [ENTER] to continue");
            self.mov_type = Movement_Type.POINT;
            return            

        self.cmd_vel_publisher.publish(twist);

def main(args=None):
    rclpy.init(args=args)
    coverage_server = CoverageServer()

    rclpy.spin(coverage_server)
    coverage_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()