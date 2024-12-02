from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
import math
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


class OdometrySensor:
    def __init__(self, max_samples=10):
        """
        max_samples: Número máximo de mediciones a almacenar
        """
        self.max_samples = max_samples
        self.positions = deque(maxlen=max_samples)  # Almacena las últimas N posiciones
        self.orientations = deque(
            maxlen=max_samples
        )  # Almacena las últimas N orientaciones (roll, pitch, yaw)

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
        self.positions.append((x, y, z))
        self.orientations.append((roll, pitch, yaw))

        # Calcular resultados lógicos
        return self._calcular_promedio()

    def _calcular_promedio(self):
        """
        Calcula el promedio de las posiciones y orientaciones, descartando los valores extremos.
        """
        # Procesar posiciones
        if (
            len(self.positions) < self.max_samples
        ):  # No hay suficientes mediciones para calcular un promedio lógico
            return None

        positions_array = np.array(self.positions)
        x = self._promedio_logico(positions_array[:, 0])
        y = self._promedio_logico(positions_array[:, 1])
        z = self._promedio_logico(positions_array[:, 2])

        # Procesar orientaciones
        orientations_array = np.array(self.orientations)
        roll = self._promedio_logico(orientations_array[:, 0])
        pitch = self._promedio_logico(orientations_array[:, 1])
        yaw = self._promedio_logico(orientations_array[:, 2])

        return AbsolutePosition(x, y, z, roll, pitch, yaw)

    def _promedio_logico(self, values):
        """
        Descartar las mediciones más altas y bajas y calcular el promedio de las intermedias.
        """
        sorted_values = np.sort(values)  # Ordenar valores
        trimmed_values = sorted_values[1:-1]  # Eliminar el valor más alto y el más bajo
        return np.mean(trimmed_values)  # Promedio de los valores restantes
