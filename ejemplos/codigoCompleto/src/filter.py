import math
import utils
from vector import Vector

ROT_MARGIN = math.pi/20
MAX_STEP_T = 0.04
MAX_TRASLATION_PER_STEP = 0.002
ROBOT_RADIUS = 0.035

class LidarPositioner:
    def __init__(self, init_pos = Vector(0, 0)):
        # self.starting_point = Vector(0, 0)
        
        self.base_fw_dist = -1
        self.base_bw_dist = -1
        self.base_rt_dist = -1
        self.base_lt_dist = -1

        self._pos = init_pos
        self.last_rot = None

    def set_base_distances(self, fw_dist, bw_dist, rt_dist, lt_dist):
        self.base_fw_dist = fw_dist
        self.base_bw_dist = bw_dist
        self.base_rt_dist = rt_dist
        self.base_lt_dist = lt_dist

    def update_starting_point(self, point):
        return
        self.starting_point = point

    def get_base_distances(self):
        return self.base_fw_dist, self.base_bw_dist, self.base_rt_dist, self.base_lt_dist

    def calculate_y_translation(self, fw_dist, bw_dist):
        if self.base_fw_dist == -1 or self.base_bw_dist == -1:
            # print(f"Base distances not set for Y translation: FW: {self.base_fw_dist}, BW: {self.base_bw_dist}")
            return 0
        # print(f"Base distances for Y translation: FW: {self.base_fw_dist}, BW: {self.base_bw_dist}")
        # print(f"Current distances for Y translation: FW: {fw_dist}, BW: {bw_dist}")
        # print(f"FW distance: {fw_dist} - BW distance: {bw_dist} - Base FW distance: {self.base_fw_dist} - Base BW distance: {self.base_bw_dist}")
        fw_diff = fw_dist - self.base_fw_dist
        bw_diff = bw_dist - self.base_bw_dist
        # print(f"Distance differences for Y translation: FW diff: {fw_diff}, BW diff: {bw_diff}")
        if abs(fw_diff) > MAX_STEP_T and abs(bw_diff) > MAX_STEP_T:
            print(f"Enormous distance differences for Y translation: FW diff: {fw_diff}, BW diff: {bw_diff}")
            print(f"Suppoused to return 0 for Y translation")
            return 0

        if abs(bw_diff) > MAX_STEP_T:
            print(f"Significant distance difference for Y translation: FW diff: {fw_diff}, BW diff: {bw_diff}")
            print(f"Suppoused to return {fw_diff} for Y translation")
            return -fw_diff
        
        if abs(bw_dist) == 1:
            print(f"BW distance is 1, using FW diff for Y translation: {fw_diff}")
            return -fw_diff
        
        return bw_diff
    
    def calculate_x_translation(self, rt_dist, lt_dist):
        if self.base_rt_dist == -1 or self.base_lt_dist == -1:
            return 0
        rt_diff = rt_dist - self.base_rt_dist
        # print(f"Base distances for X translation: RT: {self.base_rt_dist}, LT: {self.base_lt_dist}")
        # print(f"Current distances for X translation: RT: {rt_dist}, LT: {lt_dist}")
        lt_diff = lt_dist - self.base_lt_dist
        # print(f"Distance differences for X translation: RT diff: {rt_diff:.5f}, LT diff: {lt_diff:.5f}")
        if abs(rt_diff) > MAX_STEP_T and abs(lt_diff) > MAX_STEP_T:
            print(f"Enormous distance differences for X translation: RT diff: {rt_diff:.5f}, LT diff: {lt_diff:.5f}")
            print(f"Suppoused to return 0 for X translation")
            return 0
        
        if abs(rt_diff) > MAX_STEP_T:
            print(f"Significant distance difference for X translation: RT diff: {rt_diff:.5f}, LT diff: {lt_diff:.5f}")
            print(f"Suppoused to return {lt_diff:.5f} for X translation")
            return -lt_diff
        
        if abs(rt_diff) == 1:
            print(f"RT distance is 1, using LT diff for X translation: {lt_diff:.5f}")
            return -lt_diff

        return rt_diff

    def update(self, fw_dist, bw_dist, rt_dist, lt_dist, rot, pos):
        if self.last_rot is None:
            self.last_rot = rot
        rot_diff = rot - self.last_rot
        if True:
            # print(f"Rot diff: {rot_diff:.5f} within margin, updating position")
            x_translation = self.calculate_x_translation(rt_dist, lt_dist)
            y_translation = self.calculate_y_translation(fw_dist, bw_dist)
            # print(f"Calculated translations: X: {x_translation:.5f}, Y: {y_translation:.5f}")
            self._pos = Vector(pos.x - x_translation, pos.y - y_translation)
            self.last_rot = rot
            self.set_base_distances(fw_dist, bw_dist, rt_dist, lt_dist)
            return True
        return False
    
    def get_position(self):
        return self._pos

class FrontLidarPositioner:
    def __init__(self, init_pos = Vector(0, 0)):
        # self.starting_point = Vector(0, 0)
        
        self.base_front_dist = -1
        

        self._pos = init_pos
        self.last_rot = None

    def set_base_distances(self, front_dist):
        self.base_front_dist = front_dist

    def set_encoders(self, left, right):
        return
    
    def set_gps(self, gps_pos):
        return
    
    def update_starting_point(self, point):
        return
        self.starting_point = point

    def get_base_distances(self):
        return self.base_front_dist

    def calculate_translation(self, front_dist, actual_rot):
        # print(f"Calculating translation with front_dist: {front_dist}, actual_rot: {actual_rot}")
        if self.base_front_dist == -1:
            # print(f"Base distances not set for Y translation: FW: {self.base_fw_dist}, BW: {self.base_bw_dist}")
            return Vector(0, 0)
        # print(f"Base distances for Y translation: FW: {self.base_fw_dist}, BW: {self.base_bw_dist}")
        # print(f"Current distances for Y translation: FW: {fw_dist}, BW: {bw_dist}")
        # print(f"FW distance: {fw_dist} - BW distance: {bw_dist} - Base FW distance: {self.base_fw_dist} - Base BW distance: {self.base_bw_dist}")
        front_diff = front_dist - self.base_front_dist
  
        # print(f"Distance differences for Y translation: FW diff: {fw_diff}, BW diff: {bw_diff}")
        # print(f"Distance difference for Y translation: Front diff: {front_diff}")
        if front_diff == None:
            print(f"Front distance is None, returning 0 translation")
        if abs(front_diff) > MAX_STEP_T:
            print(f"Enormous distance differences for Y translation: FW diff: {front_diff}")
            print(f"Suppoused to return 0 for translation")
            return Vector(0, 0)

        # Teniendo el ángulo actual, podemos calcular la componente en X e Y del movimiento
        x_translation = -front_diff * math.sin(actual_rot)
        y_translation = -front_diff * math.cos(actual_rot)
        # print(f"Calculated translation components before clamping: X: {x_translation}, Y: {y_translation}")
        
        # clamped_x_translation = utils.clamp(x_translation, -MAX_TRASLATION_PER_STEP * math.sin(actual_rot), MAX_TRASLATION_PER_STEP * math.sin(actual_rot))
        # clamped_y_translation = utils.clamp(y_translation, MAX_TRASLATION_PER_STEP * math.cos(actual_rot), -MAX_TRASLATION_PER_STEP * math.cos(actual_rot))

        x_limit = -MAX_TRASLATION_PER_STEP * math.sin(actual_rot)
        y_limit = -MAX_TRASLATION_PER_STEP * math.cos(actual_rot)
        # print(f"Limits -> X: {x_limit}, Y: {y_limit}")
        # utils.log_debug(f"X limit: {x_limit}, Y limit: {y_limit}")

        clamped_x_translation = utils.clamp(abs(x_translation), 0, abs(x_limit), False)
        clamped_y_translation = utils.clamp(abs(y_translation), 0, abs(y_limit), False)

        if x_translation < 0:
            clamped_x_translation = -clamped_x_translation
        if y_translation < 0:
            clamped_y_translation = -clamped_y_translation
        # print(f"Clamped X translation: {clamped_x_translation}, Clamped Y translation: {clamped_y_translation}")

        # print(f"Clamped translation components: X: {clamped_x_translation}, Y: {clamped_y_translation}")
        # utils.log_debug(f"Final X traslation: {clamped_x_translation}, Final Y translation: {clamped_y_translation}")

        # print(f"X limit: {x_limit}, Y limit: {y_limit}")
        # print(f"Calculated translation components before clamping: X: {x_translation}, Y: {y_translation} based on front diff: {front_diff} and actual rotation: {actual_rot}")
        # print(f"Calculated translation components after clamping: X: {clamped_x_translation}, Y: {clamped_y_translation} based on front diff: {front_diff} and actual rotation: {actual_rot}")
        # print(f"Calculated translation components: X: {x_translation:.10f}, Y: {y_translation:.5f} based on front diff: {front_diff:.5f} and actual rotation: {actual_rot:.10f}")
        # if x_translation == 

        return Vector(clamped_x_translation, clamped_y_translation)

    def update(self, front_dist, rot, ant_pos, is_rotating):
        if is_rotating:
            # print(f"Es la primera vez, distancia frontal: {front_dist:.5f}, rotación: {rot:.5f}, posición anterior: X: {ant_pos.x:.5f}, Y: {ant_pos.y:.5f}")
            # print(f"Rotating, updating base distances with front distance: {front_dist}")
            self.set_base_distances(front_dist)
            # print(f"Robot is rotating, not updating position but setting base front distance: {front_dist}")
            return True
        else:
            # print(f"Rot diff: {rot_diff:.5f} within margin, updating position")
            translation = self.calculate_translation(front_dist, rot)
            # print(f"Calculated translation: X: {translation.x:.5f}, Y: {translation.y:.5f}")
            self._pos = Vector(ant_pos.x - translation.x, ant_pos.y - translation.y)
            # print(f"Ant position: X: {ant_pos.x:.5f}, Y: {ant_pos.y:.5f} - New position: X: {self._pos.x:.5f}, Y: {self._pos.y:.5f}")
           
            self.last_rot = rot
            self.set_base_distances(front_dist)
            return True
    
    def get_position(self):
        return self._pos
    
    def set_position(self, pos):
        self._pos = pos


_ENC_WHEEL_RADIUS = 0.026
_ENC_MAX_DISP     = 6.28 * _ENC_WHEEL_RADIUS * 0.016 * 1.1   # max desplazamiento por step (~3.1 mm)
_ENC_GPS_ALPHA    = 0.12   # peso del GPS en la fusión (12 % por step)
_ENC_GPS_SMOOTH   = 0.30   # EMA alpha para suavizado GPS (ventana efectiva ~3 steps, 53 ms)
_ENC_GPS_DEADBAND = 0.002  # error mínimo para aplicar corrección GPS (2 mm)
_ENC_STUCK_MIN_DISP = 0.0010          # avance encoder mínimo para considerar movimiento (1 mm)
_ENC_STUCK_LIDAR_EPS = 0.00035        # cambio mínimo de lidar frontal para validar avance (0.35 mm)
_ENC_STUCK_CONSECUTIVE_STEPS = 4      # steps seguidos de mismatch para declarar trabado
_ENC_UNSTUCK_CONSECUTIVE_STEPS = 2    # steps válidos para salir de estado trabado

class EncoderPositioner:
    def __init__(self, init_pos=Vector(0, 0)):
        self._pos        = init_pos
        self._prev_left  = None
        self._prev_right = None
        self._gps_smooth = None   # EMA del GPS
        self._prev_front_dist = None
        self._left_enc   = None
        self._right_enc  = None
        self._gps_pos    = None
        self.last_rot    = None
        self._stuck_counter = 0
        self._unstuck_counter = 0
        self._is_stuck_by_lidar = False

    # --- setters de datos externos (llamar antes de update) ---

    def set_encoders(self, left, right):
        self._left_enc  = left
        self._right_enc = right

    def set_front_distance(self, front_dist):
        self._prev_front_dist = front_dist

    def set_gps(self, gps_pos):
        self._gps_pos = gps_pos

    # --- interfaz polimórfica con FrontLidarPositioner ---

    def set_base_distances(self, front_dist):
        pass  # no aplica a encoders

    def update_starting_point(self, point):
        return

    def get_base_distances(self):
        return None  # no aplica a encoders

    def calculate_translation(self, front_dist, actual_rot):
        return Vector(0, 0)  # no aplica a encoders

    def update(self, front_dist, rot, ant_pos, is_rotating):
       
        # utils.log_debug(f"Así comienzo el update de EncoderPositioner: {self._pos}")
        left_enc  = self._left_enc
        right_enc = self._right_enc
        gps_pos   = self._gps_pos
        yaw       = rot

        if left_enc is None or right_enc is None:
            return True

        # Primera llamada: solo guardar valores de referencia
        if self._prev_left is None:
            self._prev_left  = left_enc
            self._prev_right = right_enc
            self._prev_front_dist = front_dist
            return True

        delta_left  = (left_enc  - self._prev_left)  * _ENC_WHEEL_RADIUS
        delta_right = (right_enc - self._prev_right) * _ENC_WHEEL_RADIUS
        self._prev_left  = left_enc
        self._prev_right = right_enc

        if is_rotating:
            self._prev_front_dist = front_dist
            return True

        d_center = (delta_left + delta_right) / 2.0
        d_center = max(-_ENC_MAX_DISP, min(_ENC_MAX_DISP, d_center))

        lidar_delta = 0.0
        if front_dist is not None and self._prev_front_dist is not None:
            lidar_delta = self._prev_front_dist - front_dist

        # Si encoder indica avance pero lidar frontal no cambia acorde, marcamos posible trabado.
        if abs(d_center) >= _ENC_STUCK_MIN_DISP and front_dist == 1.1:
            self._stuck_counter += 1
            self._unstuck_counter = 0
        else:
            self._unstuck_counter += 1
            if self._stuck_counter > 0:
                self._stuck_counter -= 1

        if self._stuck_counter >= _ENC_STUCK_CONSECUTIVE_STEPS:
            self._is_stuck_by_lidar = True

        if self._is_stuck_by_lidar:
            if self._unstuck_counter >= _ENC_UNSTUCK_CONSECUTIVE_STEPS:
                self._is_stuck_by_lidar = False
                self._stuck_counter = 0
            else:
                self._prev_front_dist = front_dist
                self.last_rot = rot
                return True

        dx = -d_center * math.sin(yaw)
        dy = -d_center * math.cos(yaw)
        encoder_pos = Vector(self._pos.x + dx, self._pos.y + dy)

        if gps_pos is not None:
            if self._gps_smooth is None:
                self._gps_smooth = gps_pos
            else:
                self._gps_smooth = Vector(
                    _ENC_GPS_SMOOTH * gps_pos.x + (1 - _ENC_GPS_SMOOTH) * self._gps_smooth.x,
                    _ENC_GPS_SMOOTH * gps_pos.y + (1 - _ENC_GPS_SMOOTH) * self._gps_smooth.y,
                )
            error = math.hypot(self._gps_smooth.x - encoder_pos.x, self._gps_smooth.y - encoder_pos.y)
            if error > _ENC_GPS_DEADBAND:
                self._pos = Vector(
                    (1 - _ENC_GPS_ALPHA) * encoder_pos.x + _ENC_GPS_ALPHA * self._gps_smooth.x,
                    (1 - _ENC_GPS_ALPHA) * encoder_pos.y + _ENC_GPS_ALPHA * self._gps_smooth.y,
                )
            else:
                self._pos = encoder_pos
        else:
            self._pos = encoder_pos

        self._prev_front_dist = front_dist
        self.last_rot = rot
        # utils.log_debug(f"Así termino el update de EncoderPositioner: {self._pos}")
        return True

    def get_position(self):
        return self._pos

    def set_position(self, pos):
        self._pos = pos
        self._gps_smooth = None