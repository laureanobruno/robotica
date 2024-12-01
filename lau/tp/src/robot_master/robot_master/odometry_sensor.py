from nav_msgs.msg import Odometry


class AbsolutePosition:
    def __init__(self, x, y, z, roll, pitch, yaw):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    def __str__(self):
        return f"x: {self.x}, y: {self.y}, z: {self.z}, roll: {self.roll}, pitch: {self.pitch}, yaw: {self.yaw}"

    def __repr__(self):
        return f"AbsolutePosition(x={self.x}, y={self.y}, z={self.z}, roll={self.roll}, pitch={self.pitch}, yaw={self.yaw})"


class OdometrySensor:
    def __init__(self):
        pass

    def procesar(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = position.x
        y = position.y
        z = position.z
        roll, pitch, yaw = self.quaternion_to_euler(
            orientation.x, orientation.y, orientation.z, orientation.w
        )
        return AbsolutePosition(x, y, z, roll, pitch, yaw)
