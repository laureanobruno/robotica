import rclpy #ROS Client Library for the Python language.
import math
import fields2cover as f2c
from rclpy.node   import Node 
from std_msgs.msg import String
from osgeo        import ogr
from enum         import Enum

class Coverage_Type(Enum):
    BOUS = 0
    SNAKE = 1

class CoverageServer(Node):
    robot = None;
    field = None;
    coverage_type = Coverage_Type.SNAKE;

    def __init__(self):
        super().__init__('coverage_server')

        # Generate random field
        rand = f2c.Random(42);
        self.field = rand.generateRandField(1e4, 5);

        # Generate robot
        self.robot = f2c.Robot(2.0, 5.0);
        
    def cover(self):
        # Generate cells add headland and calculate area
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
        path_dubins_cc = path_planner.planPath(self.robot, swaths, dubins_cc);
        print("Path: \n");
        print(path_dubins_cc, "\n");

        # Visualise
        f2c.Visualizer.figure();
        f2c.Visualizer.plot(self.field);
        f2c.Visualizer.plot(no_hl);
        f2c.Visualizer.plot(path_dubins_cc);
        f2c.Visualizer.show();


def main(args=None):
    rclpy.init(args=args)
    coverage_server = CoverageServer()
    
    coverage_server.cover()
    
    rclpy.spin(coverage_server)
    coverage_server.destroy_node()
    rclpy.shutdown()
    


if __name__ == '__main__':
    main()