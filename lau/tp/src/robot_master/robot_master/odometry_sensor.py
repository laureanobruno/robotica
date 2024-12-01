from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math


class AbsolutePosition:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __repr__(self):
        return f"AbsolutePosition(x={self.x}, y={self.y}, z={self.z}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"

    def __rotation__(self):
        return f"roll: {self.roll:.3f}, pitch: {self.pitch:.3f}, yaw: {self.yaw:.3f}"


class OdometrySensor:
    def __init__(self):
        pass

    def procesar(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = position.x
        y = position.y
        z = position.z  # Convertir quaternion a ángulos de Euler
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        r = R.from_quat(quaternion)
        roll, pitch, yaw = r.as_euler("xyz", degrees=True)
        return AbsolutePosition(x, y, z, roll, pitch, yaw)
