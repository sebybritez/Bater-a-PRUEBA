import math
import numpy as np
from typing import Dict
from vector import Vector

import logging
# logger = logging.getLogger("willow")
# filename = 'willow.log'

# logging.basicConfig(filename=filename, encoding='utf-8', level=logging.DEBUG)

logger = logging.getLogger("willow")
logger.setLevel(logging.DEBUG)

if not logger.handlers:
    handler = logging.FileHandler("willow.log", encoding="utf-8")
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)

logger.propagate = False

def create_csv(filename, headers):
    with open(filename, 'w') as f:
        f.write(headers + '\n')

def write_csv(filename, data):
    with open(filename, 'a') as f:
        f.write(';'.join(str(x) for x in data) + '\n')

def log_debug(msg):
    # print(msg)
    # return
    logger.debug(msg)

def clamp(val, min_val, max_val, print_val=False):
    if print_val:
        print(f"Clamping value: {val} to range ({min_val}, {max_val})")
    if val < min_val: 
        if print_val:
            print(f"Value {val} is less than min_val {min_val}, clamping to min_val")
        return min_val
    if val > max_val: 
        if print_val:
            print(f"Value {val} is greater than max_val {max_val}, clamping to max_val")
        return max_val
    return val

def point_on_horizontal_border(robot_position, tolerance = 0.009):
    aux = robot_position.y - 0.06
    # print(f"Checking horizontal border: robot_position.y={robot_position.y}, aux={aux}, tolerance={tolerance}")
    if near_multiple(aux, 0.12, tolerance):
        # print(f"Point {robot_position} is on horizontal border")
        return True
    return False
 
def point_on_vertical_border(robot_position, tolerance = 0.009):
    aux = robot_position.x - 0.06
    if near_multiple(aux, 0.12, tolerance):
        # print(f"Point {robot_position} is on vertical border")
        return True
    return False

def near_multiple(num, base = 0.12, tolerance = 1e-6):
    # Encuentra el múltiplo más cercano de la base
    closest_multiple = round(num / base) * base
    # Verifica si la diferencia es menor que la tolerancia
    # print(f"Checking if {num} is near multiple of {base}, difference={abs(num - closest_multiple)}, tolerance={tolerance}")
    # print("-"*20)
    return abs(num - closest_multiple) < tolerance

def sort_cw(list_points):
    list_aux = list_points.copy()
    # Ordena los puntos en sentido horario
    aux=[]
    sum_min = 1000
    first = 0
    for i in range(4):
        sum_points = list_aux[i][0] + list_aux[i][1]
        if sum_points < sum_min:
            sum_min = sum_points
            first = i
    # remove an element of numpy array
    pos_1 = list_aux[first]
    list_aux = np.delete(list_aux, first, axis=0)

    sum_max =- 10000
    third = 0
    for i in range(3):
        sum_points = list_aux[i][0] + list_aux[i][1]
        if sum_points > sum_max:
            sum_max = sum_points
            third = i
    pos_3 = list_aux[third]
    list_aux = np.delete(list_aux, third, axis=0)

    if list_aux[0][0] > list_aux[1][0]:
        pos_2 = list_aux[0]
        pos_4 = list_aux[1]
    else:
        pos_2 = list_aux[1]
        pos_4 = list_aux[0]
    return np.array([pos_1, pos_2, pos_3, pos_4], dtype = np.float32)

def get_target_contour(contours, hierarchy, is_letter_contour = False):
    hierarchy = hierarchy[0]
    for component in zip(contours, hierarchy):
        current_contour = component[0]
        current_hierarchy = component[1]            
        if not is_letter_contour:
            if current_hierarchy[2] >= 0:
                contour = current_contour
                return contour
        else:
            if current_hierarchy[2] < 0 and current_hierarchy[3] >= 0:
                contour = current_contour
                return contour
    return None

def get_percentage(a, b):
    total = a + b
    if total == 0: return None, None
    a_percentage = round((a / total) * 100)
    b_percentage = round((b / total) * 100)
    return a_percentage, b_percentage

def get_vectors_average(vectors):
    if len(vectors) == 0: return None
    sum_vector = Vector(0, 0)
    for vector in vectors:
        sum_vector += vector
    average_vector = sum_vector / len(vectors)
    return average_vector

def get_nearest_multiple(num, mult = 0.06):
    return round(num / mult) * mult

def wrap_to_pi(angle: float) -> float:
    """Envuelve un ángulo a (-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi

def cardinal_ray_indices(theta: float,
                         N: int = 512,
                         lidar_offset: float = 0.0):
    """
    Devuelve los índices de rayo (0..N-1) que apuntan a los ejes cardinales del mundo
    (arriba, abajo, izquierda, derecha), dados:
      - theta: orientación del robot en rad (0 hacia arriba, +CCW, -CW),
      - N: cantidad de rayos del LiDAR (por defecto 512),
      - lidar_offset: desfase del LiDAR respecto al chasis (rad). Por defecto 0,
        asumiendo que el rayo 0 está exactamente hacia atrás del robot.

    Convenciones del LiDAR:
      - Índices 0..N-1, crecen en sentido horario (CW).
      - El rayo 0 (y N-1) apuntan hacia atrás del robot.

    Retorna: dict con claves 'arriba', 'abajo', 'izquierda', 'derecha'.
    """
    delta = 2.0 * math.pi / N

    def angle_to_index_world(alpha_world: float) -> int:
        # Pasar el ángulo del mundo al marco del robot
        alpha_body = wrap_to_pi(alpha_world - theta)
        # Invertir la parametrización del LiDAR para hallar el índice más cercano
        idx_float = (math.pi + lidar_offset - alpha_body) / delta
        # print(f"Idx not rounded: {idx_float}")
        # idx = int(round(idx_float)) % N
        return idx_float % N

    arriba = angle_to_index_world(0.0)
    abajo  = angle_to_index_world(math.pi)      # (π ≡ -π)
    derecha = angle_to_index_world(-math.pi/2)  # CW
    izquierda = angle_to_index_world( math.pi/2) # CCW

    return arriba, abajo, izquierda, derecha

def order_nearest_points(points, reference):
    points.sort(key=lambda p: abs(reference - p))
    return points

def get_prev_follow_index(current, total = 512):
    if current - int(current) > 0.001:
        prev = int(current)
        following = (int(current) + 1) % total
    else:
        prev = following = int(current)
    return prev, following

def get_corrected_position(pos, margin):
    if pos is None: return
    pos.x = get_nearest_multiple(pos.x, margin)
    pos.y = get_nearest_multiple(pos.y, margin)
    return Vector(pos.x, pos.y)