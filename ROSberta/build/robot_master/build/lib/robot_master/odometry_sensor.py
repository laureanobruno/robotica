from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from collections import deque
import numpy as np

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

    def rotation(self):
        return f"roll: {self.roll:.3f}, pitch: {self.pitch:.3f}, yaw: {self.yaw:.3f}"
    
    def distanceTo(self, x, y):
        return np.sqrt((self.x - x)**2 + (self.y - y)**2)


class OdometrySensor:
    def __init__(self):
        pass

    def procesar(self, msg):
        """
        Procesa un mensaje de Odometry y guarda los valores en la estructura.
        """
        # Obtener posición y orientación
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        x = position.x
        y = position.y
        z = position.z

        # Convertir quaternion a ángulos de Euler
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        r = R.from_quat(quaternion)
        roll, pitch, yaw = r.as_euler("xyz", degrees=True)

        # Guardar las mediciones en las estructuras

        return AbsolutePosition(x, y, z, roll, pitch, yaw)