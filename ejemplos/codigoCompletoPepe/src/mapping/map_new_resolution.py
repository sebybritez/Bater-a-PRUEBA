import numpy as np
import cv2
from vector import Vector

TILE_SIZE = 0.12
MINITILE_SIZE = TILE_SIZE / 2
SCALE = 200  # doubled from 100 → each tile is now 24 px instead of 12 px
MINITILE_SIZE_PX = int(SCALE * MINITILE_SIZE)
INVISIBILIZER_STEPS = 1

class Map:
    def __init__(self):
        initial_size = 24
        self.img = np.ones((initial_size, initial_size), dtype=np.uint8) * 255
        self.overlay_img = np.ones((initial_size, initial_size, 3), dtype=np.uint8) * 255
        self.origin = Vector.ONE * (initial_size // 2)
        self.update_steps = 0

    def get_scale(self):
        return SCALE

    def get_image(self):
        return self.img

    def world_to_pixel(self, point):
        """Transforms point from world coordinates to pixel coordinates."""
        return (self.origin + point * SCALE).floor()

    def pixel_to_world(self, point):
        """Transforms point from pixel coordinates to world coordinates."""
        return (point - self.origin) / SCALE

    def needs_expansion(self, lidar_points):
        """Checks if the map needs expansion based on the lidar_points."""
        for v in lidar_points:
            point = self.world_to_pixel(v)
            if not (0 <= point.x < self.img.shape[1] and 0 <= point.y < self.img.shape[0]):
                return True
        return False

    def expand_map(self):
        """Expands the map if necessary."""
        new_width = self.img.shape[1] * 2
        new_height = self.img.shape[0] * 2
        new_map = np.ones((new_height, new_width), dtype=np.uint8) * 255
        new_overlay = np.ones((new_height, new_width, 3), dtype=np.uint8) * 255

        new_map[new_height // 4:new_height // 4 + self.img.shape[0],
                new_width // 4:new_width // 4 + self.img.shape[1]] = self.img
        new_overlay[new_height // 4:new_height // 4 + self.overlay_img.shape[0],
                    new_width // 4:new_width // 4 + self.overlay_img.shape[1]] = self.overlay_img

        self.img = new_map
        self.overlay_img = new_overlay
        self.origin = Vector.ONE * (new_width // 2)

    NOT_WALL_INCREMENT = 1
    WALL_DECREMENT = 15

    def update_walls(self, robot_position, lidar_points):
        """Plots lidar_points on the map texture, incrementing free pixels and decrementing collision pixels."""
        robot_pixel = self.world_to_pixel(robot_position)
        rx, ry = int(robot_pixel.x), int(robot_pixel.y)
        h, w = self.img.shape

        do_free_space = (self.update_steps % INVISIBILIZER_STEPS == 0)

        if do_free_space:
            ray_mask = np.zeros((h, w), dtype=np.uint8)

        collision_ys = []
        collision_xs = []

        for v in lidar_points:
            point = self.world_to_pixel(v)
            if 0 <= point.x < w and 0 <= point.y < h:
                px, py = int(point.x), int(point.y)

                if do_free_space:
                    cv2.line(ray_mask, (rx, ry), (px, py), 1, 1)

                collision_ys.append(py)
                collision_xs.append(px)

            self.update_steps += 1

        if do_free_space:
            self.img = np.minimum(255, self.img.astype(np.uint16) + ray_mask).astype(np.uint8)

        if collision_xs:
            cys = np.array(collision_ys, dtype=np.intp)
            cxs = np.array(collision_xs, dtype=np.intp)
            self.img[cys, cxs] = np.maximum(
                0, self.img[cys, cxs].astype(np.int16) - self.WALL_DECREMENT
            ).astype(np.uint8)

    def display_map(self):
        """Displays the environment map."""
        display_img = cv2.cvtColor(self.img, cv2.COLOR_GRAY2BGR)
        mask = np.any(self.overlay_img != 255, axis=2)
        display_img[mask] = self.overlay_img[mask]
        cv2.circle(display_img, (self.origin.x, self.origin.y), 2, (0, 140, 255), -1)

        window_name = "Environment Map"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, display_img)
        _, _, width, height = cv2.getWindowImageRect(window_name)
        if width != height:
            size = max(width, height)
            cv2.resizeWindow(window_name, size, size)
        cv2.waitKey(1)

    def update(self, robot_position, lidar_points, surrounding_tiles):
        lidar_points = [robot_position + p for p in lidar_points if p.length() < TILE_SIZE * 3.5]

        while self.needs_expansion(lidar_points):
            self.expand_map()

        self.update_walls(robot_position, lidar_points)
        # self.display_map()  # DEBUG
