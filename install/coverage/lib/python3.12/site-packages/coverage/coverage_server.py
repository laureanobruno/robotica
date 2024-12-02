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
        self.robot = f2c.Robot(1.0, 2.0);
        
    def cover(self):
        # Generate cells add headland and calculate area
        cells_c = f2c.Cells(f2c.Cell(f2c.LinearRing(f2c.VectorPoint([
        f2c.Point(0,0), f2c.Point(60,0),f2c.Point(60,60),f2c.Point(0,60), f2c.Point(0,0)]))));
        # cells_c.addRing(0, f2c.LinearRing(f2c.VectorPoint([ 
        # f2c.Point(12,12), f2c.Point(12,18),f2c.Point(18,18),f2c.Point(18,12), f2c.Point(12,12)])));
        # cells_c.addRing(0, f2c.LinearRing(f2c.VectorPoint([
        # f2c.Point(36,36), f2c.Point(36,48),f2c.Point(48,48),f2c.Point(48,36), f2c.Point(36, 36)])));

        const_hl = f2c.HG_Const_gen();
        mid_hl_c = const_hl.generateHeadlands(cells_c, 1.5 * self.robot.getWidth());
        no_hl = const_hl.generateHeadlands(cells_c, 3.0 * self.robot.getWidth());

        bf = f2c.SG_BruteForce();
        swaths = bf.generateSwaths(math.pi/2.0, self.robot.getCovWidth(), no_hl);


        # Generate swaths by vrute force
        bf = f2c.SG_BruteForce();
        swaths = bf.generateSwaths(math.pi/2.0, self.robot.getCovWidth(), no_hl);

        # Set robot movement constraints
        self.robot.setMinTurningRadius(3)  # m
        self.robot.setMaxDiffCurv(0.2);  # 1/m^2
        path_planner = f2c.PP_PathPlanning()

        # Conection of paths with Dubin Curves
        # Done with continous survature to avoid instant changes of direction
        route_planner = f2c.RP_RoutePlannerBase();
        route = route_planner.genRoute(mid_hl_c, swaths);

        # Visualise
        f2c.Visualizer.figure();
        f2c.Visualizer.plot(cells_c);
        f2c.Visualizer.plot(no_hl);
        f2c.Visualizer.plot(route);
        f2c.Visualizer.xlim(-5,65);
        f2c.Visualizer.ylim(-5,65);
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