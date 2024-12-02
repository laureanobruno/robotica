from sensor_msgs.msg import LaserScan


class WallDistances:

    global dictionary
    dictionary = {0: "LEFT", 1: "RIGHT", 2: "FRONT", -1: "NONE"}

    def __init__(self, front, left, right, closest_wall):
        self.front = front
        self.left = left
        self.right = right
        self.closest_wall = closest_wall
        self.closest = min(front, left, right)

    def __repr__(self):
        return (
            f"WallDistances(front={self.front}, left={self.left}, "
            f"right={self.right}, closest_wall='{dictionary[self.closest_wall]}')"
        )


class LidarSensor:
    global samples
    samples = 640

    global LEFT
    global RIGHT
    global FRONT
    global NONE
    LEFT = 0
    RIGHT = 1
    FRONT = 2
    NONE = -1

    def __init__(
        self,
        samples,
        wall_threshold,
        min_angle,
        max_angle,
        left_range,
        center_range,
        right_range,
    ):
        self.samples = samples
        self.wall_threshold = wall_threshold
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.left_range = left_range
        self.right_range = right_range
        self.center_range = center_range
        self.definirRangos()

    def definirRangos(self):

        def calcular_indice(angulo):
            return int(
                (angulo - self.min_angle)
                / (self.max_angle - self.min_angle)
                * (self.samples - 1)
            )

        self.index_start_left = calcular_indice(self.left_range[0])
        self.index_end_left = calcular_indice(self.left_range[1])
        self.index_start_right = calcular_indice(self.right_range[0])
        self.index_end_right = calcular_indice(self.right_range[1])
        self.index_start_center = calcular_indice(self.center_range[0])
        self.index_end_center = calcular_indice(self.center_range[1])

        # print(
        #     f"Left: {self.index_start_left}-{self.index_end_left}, "
        #     f"Right: {self.index_start_right}-{self.index_end_right}, "
        #     f"Center: {self.index_start_center}-{self.index_end_center}"
        # )

    def procesar(self, msg):
        front_ranges = [
            r
            for r in msg.ranges[self.index_start_center : self.index_end_center]
            if r < float("inf")
        ]
        left_ranges = [
            r
            for r in msg.ranges[self.index_start_left : self.index_end_left]
            if r < float("inf")
        ]
        right_ranges = [
            r
            for r in msg.ranges[self.index_start_right : self.index_end_right]
            if r < float("inf")
        ]

        closest_front = min(front_ranges, default=float("inf"))
        closest_left = min(left_ranges, default=float("inf"))
        closest_right = min(right_ranges, default=float("inf"))

        if (
            closest_front <= self.wall_threshold
            and closest_front < closest_left
            and closest_front < closest_right
        ):
            closest_wall = FRONT
        elif (
            closest_left <= self.wall_threshold
            and closest_left < closest_front
            and closest_left < closest_right
        ):
            closest_wall = LEFT
        elif (
            closest_right <= self.wall_threshold
            and closest_right < closest_front
            and closest_right < closest_left
        ):
            closest_wall = RIGHT
        else:
            closest_wall = NONE

        return WallDistances(closest_front, closest_left, closest_right, closest_wall)
