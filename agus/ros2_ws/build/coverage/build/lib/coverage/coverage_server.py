import rclpy #ROS Client Library for the Python language.
from rclpy.node import Node 
from std_msgs.msg import String
import fields2cover as f2c
from osgeo import ogr
import math

class CoverageServer(Node):
    robot = None;
    field = None;

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

        # Order swaths using the Boustrophedon Order
        boustrophedon_sorter = f2c.RP_Boustrophedon();
        swaths = boustrophedon_sorter.genSortedSwaths(swaths);

        f2c.Visualizer.figure();
        f2c.Visualizer.plot(self.field);
        f2c.Visualizer.plot(no_hl);
        f2c.Visualizer.plot(swaths);
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