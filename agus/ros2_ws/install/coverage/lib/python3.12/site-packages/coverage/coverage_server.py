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

class Coverage_Type(Enum):
    BOUS = 0
    SNAKE = 1

class CoverageServer(Node):
    path = f2c.Path;
    coverage_type = Coverage_Type.SNAKE;
    latest_pose: gmsg.Pose;
    curr_path = 0;
    curr_path_pose: gmsg.Pose;

    def __init__(self):
        super().__init__('coverage_server')

        # Generate random field
        rand = f2c.Random(42);
        self.field = rand.generateRandField(1e4, 5);

        # Generate robot
        self.robot = f2c.Robot(2.0, 5.0);
        self.robot_lspeed = 1; #m/s
        self.robot_aspeed = 0.2; #rad/s

        # Listener de pose
        self.odometry_listener = self.create_subscription(navmsg.Odometry, "/diff_drive/odometry", self.odometry_callback, 10)
        self.latest_pose = None

        # Publisher de Twist
        self.cmd_vel_publisher = self.create_publisher(gmsg.Twist, "/diff_drive/cmd_vel", 10);

    def odometry_callback(self, msg: navmsg.Odometry):
        self.latest_pose = msg.pose.pose;
        print("Received pose ", self.latest_pose)
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
        self.robot.setMinTurningRadius(3)  # m
        self.robot.setMaxDiffCurv(0.2);  # 1/m^2
        path_planner = f2c.PP_PathPlanning()

        # Conection of paths with Dubin Curves
        # Done with continous survature to avoid instant changes of direction

        dubins_cc = f2c.PP_DubinsCurvesCC();
        self.path = path_planner.planPath(self.robot, swaths, dubins_cc);

        self.curr_path_pose = self.generate_pose(self.path[0]);
        # Visualise
        f2c.Visualizer.figure();
        f2c.Visualizer.plot(self.field);
        f2c.Visualizer.plot(no_hl);
        f2c.Visualizer.plot(self.path);
        f2c.Visualizer.show();

    def generate_pose(self, path: f2c.PathState):
        pose = gmsg.Pose()
        position = gmsg.Point()
        pose.position.x = path.point.getX()
        pose.position.y = path.point.getY()
        pose.position.z = path.point.getZ()

        # Create quaternion using only pitch
        pose.orientation = gmsg.Quaternion(x=0, y=math.cos(path.angle), z=0, w=math.sin(path.angle))

        # Pose
        return pose


    def follow_path(self):
        # Wait for pose from robot 
        while (self.latest_pose == None):
           # print("Waiting for robot")
            pass

        # Iterate over path
        total_paths = self.path.size();
        for stretch in range(0, total_paths):
            pose = self.generate_pose(self.path[stretch]);
            print("Driving to pose: ", pose, "\n")
            self.drive_to_pose(pose, self.path[stretch])
            #print(pose, '\n');
    
    def drive_to_pose(self, pose: gmsg.Pose, path: f2c.PathState):
        twist = gmsg.Twist();
        curr_angle = math.asin(self.latest_pose.orientation.y)
        
        # Rotate to angle
        if (curr_angle != path.angle):
            twist.angular.y = math.copysign(self.robot_aspeed, path.angle - curr_angle);
            self.cmd_vel_publisher.publish(twist);
        else:
            twist.angular.y = 0
            twist.linear.x = self.robot_lspeed;

            
        



def main(args=None):
    rclpy.init(args=args)
    coverage_server = CoverageServer()
    
    coverage_server.generate_path();
    
    rclpy.spin(coverage_server)
    coverage_server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()