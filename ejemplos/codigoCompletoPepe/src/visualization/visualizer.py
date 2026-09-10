import numpy as np
from visualization.json_utils import JSON, sanitize_keys, to_json_dict
from visualization.socket_connection import SocketConnection
from visualization.file_connection import FileConnection

SKIP_EVERY = 25

class Visualizer:
    def __init__(self, config):
        self.connections = []
        if config.get("visualizer_enabled", False):
            if config.get("visualizer.socket_enabled", False):
                self.connections.append(SocketConnection())
            if config.get("visualizer.file_enabled", False):
                self.connections.append(FileConnection())

        self.step_counter = 0
        self.update_counter = 0
        self.dont_skip = True

        self.previous_imgs = {}
        self.previous_json = {}

    def step(self):
        self.step_counter += 1
        self.send_step()

    def stop(self):
        for conn in self.connections:
            conn.stop()

    def get_active_connections(self):
        return [conn for conn in self.connections if conn.is_connected()]

    def update(self, director):
        connections = self.get_active_connections()
        if len(connections) == 0: return

        self.dont_skip = self.update_counter % SKIP_EVERY == 0
        self.update_counter += 1
        self.send_update(connections)
        self.send_navigator_map(director.navigator.map, connections)
        self.send_robot(director.robot, connections)
        self.send_mapper_map(director.mapper.map, connections)
        self.send_path_finding(director.navigator, connections)
        self.send_visited_minitiles(director.navigator.map.minitiles, connections)
        self.send_signs(director.robot.start_pos, director.mapper.signs, connections)
        self.send_visited_map(director.navigator.map, connections)
        self.send_pixel_paths(director.navigator.pixel_paths, connections)
        self.dont_skip = False

    def send_navigator_map(self, navigator_map, active_connections = None):
        self.send_image(0, navigator_map.get_image(), navigator_map.get_scale(), active_connections)

    def send_robot(self, robot, active_connections = None):
        self.send_json(1, robot.to_dict(), active_connections)

    def send_mapper_map(self, mapper_map, active_connections = None):
        self.send_image(2, mapper_map.get_image(), mapper_map.get_scale(), active_connections)

    def send_path_finding(self, navigator, active_connections = None):
        self.send_json(3, {
            "start": navigator.start,
            "came_from": sanitize_keys(navigator.came_from),
            "cost_so_far": sanitize_keys(navigator.cost_so_far),
            "walls_around": sanitize_keys(navigator.walls_around),
            "penalty_scores": sanitize_keys(navigator.penalty_scores),
            "priority_scores": sanitize_keys(navigator.priority_scores),
            "path": navigator.path,
            "tsp": navigator.tsp,
        }, active_connections)

    def send_visited_minitiles(self, minitiles, active_connections = None):
        self.send_json(4, sanitize_keys(minitiles), active_connections)

    def send_signs(self, start_pos, signs, active_connections = None):
        sign_data = []
        for key, val in signs.items():
            t = ((key.x, key.y), val[1])
            sign_data.append(t)

        self.send_json(5, {
            "origin": start_pos,
            "signs": sign_data
        }, active_connections)

    def send_visited_map(self, navigator_map, active_connections = None):
        self.send_image(6, navigator_map.get_visited_image(), navigator_map.get_scale(), active_connections)
    
    def send_debug(self, debug_data, active_connections = None):
        self.send_json(7, debug_data, active_connections)

    def send_pixel_paths(self, pixel_paths, active_connections = None):
        self.send_json(8, to_json_dict(pixel_paths), active_connections)

    def send_step(self, active_connections = None):
        self.send_binary(255, bytes(), active_connections)

    def send_update(self, active_connections = None):
        self.send_binary(254, bytes(), active_connections)

    def send_json(self, msg_type, data, active_connections = None):
        old_json = self.previous_json.get(msg_type)
        new_json = JSON.stringify(data).encode("utf8")
        if self.dont_skip or new_json != old_json:
            self.previous_json[msg_type] = new_json
            
            self.send_binary(msg_type, new_json, active_connections)

    def send_image(self, msg_type, image, scale, active_connections = None):
        previous_img = self.previous_imgs.get(msg_type)
        self.previous_imgs[msg_type] = image.copy()

        data = bytearray()
        data.extend(scale.to_bytes(4, "little"))

        if self.dont_skip or previous_img is None or np.shape(image) != np.shape(previous_img):
            # print(f"Sending FULL frame {msg_type} at update {self.update_counter} (step {self.step_counter})!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            data.append(0) # Full frame
            data.extend(image.tobytes())
        else:
            data.append(1) # Delta frame
            diff_mask = previous_img != image
            if np.count_nonzero(diff_mask) == 0:
                # print(f"Skipping {msg_type} at update {self.update_counter} because delta frame is empty")
                return
            
            # print(f"Sending delta frame {msg_type} at update {self.update_counter}")
            diff_image = np.ones_like(previous_img) * 255
            diff_image[diff_mask] = image[diff_mask]
            data.extend(diff_image.tobytes())
        
        self.send_binary(msg_type, bytes(data), active_connections)

    def send_binary(self, msg_type, data, active_connections = None):
        if active_connections == None:
            active_connections = self.get_active_connections()
        for conn in active_connections:
            conn.send_binary(msg_type, data)