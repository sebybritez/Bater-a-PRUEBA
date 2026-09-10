from controller import Robot as WebotsRobot # type: ignore
import math
import utils
import numpy as np
from vector import Vector
from side import Side
from filter import FrontLidarPositioner, EncoderPositioner
import traceback
import inspect

MAX_VEL = 6.28
TIME_STEP = 16
LIMIT_STEPS = 100
WHEEL_RADIUS = 0.026
DIST_BETWEEN_WHEELS = 0.104
MIN_POINTS_ICP = 4
MAX_LIDAR_DIST = 1
LIMIT_AVERAGE = 25

DEBUGGING = False
DEBUGGING_3_CAMS = True
DEBUGGPSOK = False   # True: activa y usa gps_ok para comparar posición; False: no lo habilita

class Robot:
    def __init__(self):
        self.robot = WebotsRobot()

        self.wheel_left = self.robot.getDevice("wheel2 motor")
        self.wheel_right = self.robot.getDevice("wheel1 motor")
        self.wheel_left.setPosition(float('inf'))
        self.wheel_right.setPosition(float('inf'))
        self.stop()

        self.enc_left = self.wheel_left.getPositionSensor()
        self.enc_right = self.wheel_right.getPositionSensor()
        self.enc_left.enable(TIME_STEP)
        self.enc_right.enable(TIME_STEP)
        
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(TIME_STEP)

        self.gps_ok = None
        if DEBUGGPSOK:
            self.gps_ok = self.robot.getDevice("gps_ok")
            self.gps_ok.enable(TIME_STEP)

        self.imu = self.robot.getDevice("inertial_unit")
        self.imu.enable(TIME_STEP)

        self.lidar = self.robot.getDevice("lidar")
        self.lidar.enable(TIME_STEP)

        self.front_camera = self.robot.getDevice("camaraFrontal")
        self.front_camera.enable(TIME_STEP)

        self.left_camera = self.robot.getDevice("camaraIzquierda")
        self.left_camera.enable(TIME_STEP)

        self.right_camera = self.robot.getDevice("camaraDerecha")
        self.right_camera.enable(TIME_STEP)

        self.distance_sensor = self.robot.getDevice("distance sensor1")
        self.distance_sensor.enable(TIME_STEP)

        self.receiver = self.robot.getDevice("receiver")
        self.receiver.enable(TIME_STEP)
        self.emitter = self.robot.getDevice("emitter")

        self.is_moving = False
        self.is_lop = False
        self.previous_position = Vector.ZERO
        self.previous_direction = Vector.ZERO
        self.start_pos = None
        self.stuck_counter = 0
        self.step_callbacks = []

        self.start_pos = None
        self._last_time = None
        self.first_dists = {}
        self._lidar_image_cache = None

        self.is_the_same_position = True
        self.same_position_counter = 0
        self.is_saving_gps_samples = False
        self.max_diff_x = 0
        self.max_diff_y = 0
        
        # utils.log_debug("Robot initialized")
        self.raw_step()

        self.gps_pos = np.array([Vector.ZERO for _ in range(LIMIT_AVERAGE)], dtype=object)

        self.first_update = True
        self.step_counter = 0
        self.is_rotating = True
        self.pos = Vector(0,0)

        # self.odometry = FrontLidarPositioner(init_pos = Vector(0, 0))
        self.odometry = EncoderPositioner(init_pos = Vector(0, 0))

        self.step()
        # utils.log_debug("First step completed")
        raw_start_pos = self.get_absolute_position()
        self.start_pos = utils.get_corrected_position(raw_start_pos, 0.06)
        # print(f"Robot start position set to: {self.start_pos}") 

        self.last_absolute_position = self.start_pos

    def set_corrected_position_by_lop(self, pos, margin):
        """Sets the robot's perfect position if a LoP is executed."""
        # NOTE (Martu): When making a LoP, the robot changes its position to the exact center of a tile, so we correct the GPS given position
        # to make it a multiple of 0.06 (the half tile size) and set it as the robot's position.
        pos = utils.get_corrected_position(pos, margin)
        self.odometry.set_position(pos - self.start_pos)
        self.odometry.set_encoders(0, 0)
        self.odometry.set_front_distance(None)
        self.pos = self.odometry.get_position()
        self.is_saving_gps_samples = False
        self.same_position_counter = 0

    def raw_step(self):
        """Raw step without odometry update. Used only for initialization."""
        self.update_stuck_counter()
        for callback in self.step_callbacks:
            callback()
        result = self.robot.step(TIME_STEP) != -1
        self._lidar_image_cache = None
        return result

    def to_dict(self):
        """Returns the robot's current state as a dictionary."""
        return {
            "position": self.get_position(),
            "rotation": self.get_rotation(),
        }

    def on_step(self, callback):
        """Registers a callback to be called on each step."""
        self.step_callbacks.append(callback)

    def extract_stack(self):
        """Extracts the current call stack."""
        call_stack = traceback.extract_stack()
        functions = ""
        idx = 0
        for frame_summary in call_stack[2:-2]:
            if idx > 0:
                functions += "/"
            functions += (frame_summary.name)
            idx += 1

        with open("steps.txt", "a", encoding="utf-8") as f:
            f.write(functions + "\n")

    def step(self):
        """Performs a single step of the robot's operation."""
        self.step_counter += 1

        self.is_lop = False
        if self.receiver.getQueueLength() > 0:
            message = self.receiver.getBytes()
            if message[0] == ord('L'):
                print(f"Received LoP message in robot step!!")
                # utils.log_debug(f"Received LoP message in robot step!!")
                self.is_lop = True
                self.set_corrected_position_by_lop(self.get_absolute_position(), 0.06)

        self.update_odometry()
        self.update_stuck_counter()
        
        if not self.is_saving_gps_samples and self.is_the_same_position:
            self.is_saving_gps_samples = True
        elif self.is_saving_gps_samples and (not self.is_the_same_position) or (self.same_position_counter >= LIMIT_AVERAGE):
            x = [pos.x for pos in self.gps_pos[:self.same_position_counter]]
            y = [pos.y for pos in self.gps_pos[:self.same_position_counter]]
            self.odometry.set_position(Vector(np.mean(x), np.mean(y)))
            # utils.log_debug(f"Calculated average GPS position: {self.odometry.get_position()} based on {self.same_position_counter} samples")
            self.is_saving_gps_samples = False
            self.same_position_counter = 0
        
        if self.is_saving_gps_samples and self.same_position_counter < LIMIT_AVERAGE:
            self.gps_pos[self.same_position_counter] = self.gps_relative_position()
            self.same_position_counter += 1

        # print(f"Is robot in LoP? {self.is_lop}")
        if DEBUGGPSOK:
            est = self.get_position()
            real = self.gps_ok_relative_position()
            diff_x = abs(est.x - real.x)
            diff_y = abs(est.y - real.y)
            self.max_diff_x = max(self.max_diff_x, diff_x)
            self.max_diff_y = max(self.max_diff_y, diff_y)
            # print(f"Dif X: {diff_x:.4f}  Y: {diff_y:.4f}  |  Max X: {self.max_diff_x:.4f}  Max Y: {self.max_diff_y:.4f}")
        for callback in self.step_callbacks:
            callback()
        result = self.robot.step(TIME_STEP) != -1
        self._lidar_image_cache = None
        return result

    def update_stuck_counter(self):
        """Updates the counter that tracks how many consecutive steps the robot has been stuck (not moving)."""
        if self.start_pos == None: return
        if not self.is_moving:
            self.stuck_counter = 0
            return

        current_pos = self.get_position()
        vel = current_pos - self.previous_position

        current_dir = self.get_forward_vector()
        vang = current_dir.angle_to(self.previous_direction)

        # print(f"Vector length: {vel.length()}")
        if vel.length() < 0.0005 and vang < 0.001:
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0

        self.previous_position = current_pos
        self.previous_direction = current_dir

    def is_stuck(self, step_threshold=LIMIT_STEPS):
        """Checks if the robot is stuck based on the stuck counter and a step threshold."""
        return self.stuck_counter > step_threshold

    def delay(self, ms):
        """Delays the robot's operation for a given number of milliseconds."""
        init_time = self.robot.getTime()
        while self.step():
            if (self.robot.getTime() - init_time) * 1000.0 >= ms:
                break

    def set_velocity(self, l, r):
        """Sets the velocity of the robot's wheels and updates the robot's movement state."""
        self.wheel_left.setVelocity(utils.clamp(l, -MAX_VEL, MAX_VEL))
        self.wheel_right.setVelocity(utils.clamp(r, -MAX_VEL, MAX_VEL))
        if l == 0 and r == 0:
            self.stuck_counter = 0
            self.is_moving = False
        else:
            self.is_moving = True

        if l == r:
            self.is_rotating = False
        else:
            self.is_rotating = True

        if (l==-r):
            self.is_the_same_position=True
        else:
            self.is_the_same_position=False
        
    def stop(self):
        """Stops the robot's movement by setting both wheel velocities to zero."""
        self.set_velocity(0, 0)

    def get_lidar_image(self):
        """Returns the cached lidar image if available; otherwise, retrieves it from the lidar sensor."""
        if self._lidar_image_cache is None:
            image = self.lidar.getRangeImage()
            self._lidar_image_cache = image[1024:1536]
        return self._lidar_image_cache

    def get_lidar_points(self):
        """Returns the lidar's distance collision points as world coordinates (Vectors)."""
        roll, pitch, _ = self.imu.getRollPitchYaw()
        if abs(roll) > 0.01 or abs(pitch) > 0.01:
            return []

        distances = np.array(self.get_lidar_image(), dtype=np.float32)

        # Ray 0 points opposite to forward; rays increase clockwise (delta = -2π/512)
        base_angle = self.get_forward_vector().angle() + math.pi
        angles = base_angle + np.arange(512, dtype=np.float32) * (-2.0 * math.pi / 512)

        valid = distances != np.inf
        d = distances[valid]
        a = angles[valid]

        xs = np.sin(-a) * d   # Vector.from_angle uses sin(-angle) for x
        ys = -np.cos(a) * d   # and -cos(angle) for y

        return [Vector(float(xs[i]), float(ys[i])) for i in range(len(d))]

    def update_odometry(self):
        """Updates the robot's odometry based on lidar and encoders readings."""
        # print(f"Updating odometry with pos: {self.get_position()}")
        yaw = self.get_rotation()
        front_ray = Side.FRONT.value
        front_dist = min(min(self.get_lidar_image()[front_ray - 64: front_ray + 64]), 1.1) # NOTE: Used min function in case lidar dist = inf
        # (max lidar dist: 1).

        if self.first_update:
            self.odometry.set_base_distances(front_dist)
            print(f"First update - setting base front distance: {front_dist}")
            self.first_update = False
        # print(f"Updating odometry with front distance: {front_dist} and yaw: {yaw}")    
        self.odometry.set_encoders(self.enc_left.getValue(), self.enc_right.getValue())
        self.odometry.set_gps(self.gps_relative_position())
        self.odometry.update(front_dist, yaw, self.get_position(), self.is_rotating)
        self.pos = self.odometry.get_position()

    def get_position(self):
        """Returns the robot's current position as a Vector, relative to the starting position."""
        return self.odometry.get_position() # pos
    
    def gps_relative_position(self):
        """Returns the robot's GPS (noisy) position as a Vector, relative to the starting position."""
        abs_pos = self.get_absolute_position()
        if abs_pos is None or self.start_pos is None:
            return Vector.ZERO
        return abs_pos - self.start_pos
    
    def get_absolute_position(self):
        """Returns the robot's GPS (noisy) absolute position as a Vector."""
        x, z, y = self.gps.getValues()
        return Vector(x, y)

    def gps_ok_relative_position(self): # TODO (Martu): Delete...
        if not DEBUGGPSOK or self.gps_ok is None:
            return Vector.ZERO
        x, z, y = self.gps_ok.getValues()
        abs_pos = Vector(x, y)
        if self.start_pos is None:
            return Vector.ZERO
        return abs_pos - self.start_pos

    def get_rotation(self):
        """Returns the robot's current rotation (yaw) in radians."""
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        return yaw

    def convert_camera(self, img, height, width):
        """Converts the camera image from a byte buffer to a numpy array with shape (height, width, 4)."""
        converted_image = np.array(np.frombuffer(img, np.uint8).reshape((height, width, 4)))
        return converted_image

    def get_camera_from_side(self, side):
        """Returns the camera image from the specified side (LEFT, RIGHT, FRONT) as a numpy array."""
        if side == Side.LEFT:
            camera = self.left_camera
        else:
            if side == Side.RIGHT:
                camera = self.right_camera
            else:
                camera = self.front_camera
        return self.convert_camera(camera.getImage(), 40, 40)

    def get_forward_vector(self):
        """Returns the robot's forward direction as a unit vector."""
        return Vector.from_angle(self.get_rotation())

    def look_at(self, direction, margin=0.001, *, blocking_steps=-1):
        """Rotates the robot to look at a specific direction (Vector) within a given margin."""
        # print(f"Looking at {direction} with margin {margin} and blocking_steps {blocking_steps}")
        is_done = False
        steps = 0
        while not self.is_stuck() and not self.is_lop:
            angle = self.get_forward_vector().angle_to(direction)
            if abs(angle) < margin:
                is_done = True
                self.stop()
                break

            vel = MAX_VEL / 0.25 * abs(angle)
            if angle < 0:
                self.set_velocity(vel, -vel)
            else:
                self.set_velocity(-vel, vel)

            if blocking_steps == steps:
                break

            if not self.step():
                break

            steps += 1

        return is_done

    def turn(self, angle, margin=0.001, *, blocking_steps=-1):
        """ Rotates the robot by a specific angle (in radians) within a given margin."""
        # print(f"Robot state: turning!")
        target = self.get_forward_vector().rotated(angle)
        return self.look_at(target, margin, blocking_steps=blocking_steps)

    def advance(self, distance, margin=0.001, *, blocking_steps=-1):
        """Moves the robot forward or backward by a specific distance (in meters) within a given margin."""
        initial_pos = self.get_position()
        # print(f"Initial pos before advance loop: {initial_pos}")
        is_done = False
        steps = 0
        while not self.is_stuck() and not self.is_lop:
            traveled_dist = self.get_position().distance_to(initial_pos)
            diff = abs(distance) - traveled_dist
            if diff <= margin:
                is_done = True
                self.stop()
                break

            vel = min(max(diff/0.01, 0.1), 1) * MAX_VEL
            if distance < 0: vel *= -1

            if vel > 0:
                front_rays = self.get_lidar_image()[Side.FRONT.value-45:Side.FRONT.value+45]
                if min(front_rays) < 0.044:  # If we are very close to the wall, exit the while loop to avoid collision
                    self.stop()
                    print("Stopping advance to avoid collision with wall in front")
                    break
            else:
                back_rays = self.get_lidar_image()[Side.BACK.value:Side.BACK.value+45] + self.get_lidar_image()[Side.BACK.value+467:Side.BACK.value+512]
                if min(back_rays) < 0.044:  # If we are very close to the wall, exit the while loop to avoid collision
                    self.stop()
                    print("Stopping advance to avoid collision with wall in back")
                    break

            self.set_velocity(vel, vel)

            if blocking_steps == steps:
                break
            if not self.step():
                break

            steps += 1

        return is_done
    
    def move_to_point(self, target, margin=0.001, *, blocking_steps=-1):
        """Moves the robot to a specific point (Vector) within a given margin without making curved movements."""
        ANGLE_MIN = math.pi / 42

        is_done = False
        steps = 0
        while not self.is_stuck() and not self.is_lop:
            position = self.get_position()
            dist = position.distance_to(target)

            if dist < margin:
                is_done = True
                self.stop()
                break

            angle = self.get_forward_vector().angle_to(target - position)
            # print(f"Angle to target: {math.degrees(angle)} degrees")

            if abs(angle) < ANGLE_MIN:
                self.set_velocity(MAX_VEL, MAX_VEL)
                self.is_rotating = False
            else:
                # Proportional control for turning speed based on the angle to the target
                turn_vel = utils.clamp(abs(angle) / (math.pi / 4) * MAX_VEL, MAX_VEL * 0.3, MAX_VEL)
                if angle < 0:
                    self.set_velocity(turn_vel, -turn_vel)
                else:
                    self.set_velocity(-turn_vel, turn_vel)
                self.is_rotating = True
            # print(f"Is robot rotating? {self.is_rotating}")
            if blocking_steps == steps:
                break
            if not self.step():
                break

            steps += 1

        return is_done
    