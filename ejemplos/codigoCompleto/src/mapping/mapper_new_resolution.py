import numpy as np
from vector import Vector
from tile_classifier import TileType
from mapping.map_new_resolution import Map, TILE_SIZE, SCALE
import math
from enum import Enum
import cv2
import utils

# Each tile is now 24 px × 24 px (SCALE doubled to 200).
TILE_SIZE_PX = int(TILE_SIZE * SCALE)   # 24
HALF = TILE_SIZE_PX // 2                # 12  – pixels per quarter-tile side
LAST = HALF - 1                         # 11  – last index inside a quarter-tile


class Tile:
    def __init__(self, tile_type, pos):
        self.tile_type = tile_type
        self.pos = pos
        self.rep = np.zeros((5, 5), dtype=int)
        self.area = None

    def __repr__(self):
        return f"Tile({self.tile_type}, {self.pos}, Area {self.area})"

    def __str__(self):
        return f"Tile({self.tile_type}, {self.pos}, Area {self.area})"


class Curve(Enum):
    LEFT_TOP = 6
    RIGHT_TOP = 7
    RIGHT_BOTTOM = 8
    LEFT_BOTTOM = 9
    VALUES_REPLACED_BY_1 = 10


class Mapper:

    def __init__(self):
        self.signs = {}
        self.tiles = {}
        self.visited_tiles = []
        self.map = Map()
        self.obstacles = set()

    def set_area(self, tile, area):
        """Sets the area of a tile, unless it's a colored passage tile."""
        if self.is_color_passage(tile): return None
        tile.area = area

    def is_color_passage(self, tile):
        """Checks if a tile is a colored passage tile."""
        colored_types = [TileType.BLUE, TileType.YELLOW, TileType.GREEN,
                         TileType.PURPLE, TileType.ORANGE, TileType.RED]
        return tile.tile_type in colored_types

    def get_new_area(self, area, tile):
        """Determines the new area for a tile based on its current area and type."""
        color = tile.tile_type
        possible_areas = {
            (1, TileType.BLUE): 2,
            (1, TileType.YELLOW): 3,
            (1, TileType.GREEN): 4,
            (2, TileType.BLUE): 1,
            (2, TileType.PURPLE): 3,
            (2, TileType.ORANGE): 4,
            (3, TileType.YELLOW): 1,
            (3, TileType.PURPLE): 2,
            (3, TileType.RED): 4,
            (4, TileType.GREEN): 1,
            (4, TileType.ORANGE): 2,
            (4, TileType.RED): 3,
        }
        change_of_area = (area, color)
        if change_of_area in possible_areas:
            return possible_areas[change_of_area]
        return None

    def get_tile_at_pos(self, pos):
        """Returns the tile object at a given position, or None if it doesn't exist."""
        return self.tiles.get(pos, None)

    def get_visited_tile_at_pos(self, pos):
        """Returns the visited tile object at a given position, or None if it doesn't exist."""
        for visited_tiles_dict in self.visited_tiles:
            if pos in visited_tiles_dict:
                return visited_tiles_dict[pos]
        return None

    def is_tile_in_visited_tiles(self, pos):
        """Checks if a tile is in the visited tiles list."""
        for visited_tiles_dict in self.visited_tiles:
            if pos in visited_tiles_dict:
                return True
        return False

    def add_sign(self, letter, pos, tile_pos):
        """Adds a sign to the signs dictionary."""
        self.signs[pos] = (tile_pos, letter)

    def add_tile(self, tile):
        """Adds a tile to the tiles dictionary if it doesn't already exist."""
        if tile.pos not in self.tiles:
            self.tiles[tile.pos] = tile

    def add_visited_tile(self, tile):
        """Adds a tile to the visited tiles list if it doesn't already exist."""
        for visited_tiles_dict in self.visited_tiles:
            if tile.pos in visited_tiles_dict:
                return
        self.visited_tiles.append({tile.pos: tile})

    def add_tiles(self, tile_list):
        """Adds multiple tiles to the tiles dictionary."""
        for pos, tile_type in tile_list.items():
            tile = Tile(tile_type, pos)
            self.add_tile(tile)

    def update(self, robot_pos, lidar_points, surrounding_tiles):
        """Updates the map with the robot's position, lidar points, and surrounding tiles."""
        self.add_tiles(surrounding_tiles)
        self.map.update(robot_pos, lidar_points, surrounding_tiles)

    def add_obstacles(self, obstacle_pos):
        """Adds an obstacle to the obstacles set."""
        self.obstacles.add(obstacle_pos)
        
    def pixel_to_tile(self, pixel):
        """Converts pixel coordinates to tile coordinates."""
        col = int(pixel[0] / TILE_SIZE_PX)
        row = int(pixel[1] / TILE_SIZE_PX)
        return (col, row)

    # ── Wall setters (unchanged – they write into the 5×5 tile rep) ─────────

    def set_left_straight_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the left straight wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map] = 1

    def set_right_straight_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the right straight wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 2] = 1

    def set_top_straight_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the top straight wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map] = 1
        tile_rep[row_quarter_map, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map, col_quarter_map + 2] = 1

    def set_bottom_straight_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the bottom straight wall in the tile representation."""
        tile_rep[row_quarter_map + 2, col_quarter_map] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 2] = 1

    def set_left_top_curve_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the left top curve wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 2] = Curve.LEFT_TOP.value

    def set_right_top_curve_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the right top curve wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map] = Curve.RIGHT_TOP.value

    def set_right_bottom_curve_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the right bottom curve wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map] = 1
        tile_rep[row_quarter_map, col_quarter_map] = Curve.RIGHT_BOTTOM.value

    def set_left_bottom_curve_wall(self, tile_rep, row_quarter_map, col_quarter_map):
        """Sets the left bottom curve wall in the tile representation."""
        tile_rep[row_quarter_map, col_quarter_map] = 1
        tile_rep[row_quarter_map, col_quarter_map + 1] = 1
        tile_rep[row_quarter_map + 1, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map + 2, col_quarter_map + 2] = 1
        tile_rep[row_quarter_map, col_quarter_map + 2] = Curve.LEFT_BOTTOM.value

    # ── Curve detection – indices scaled ×2 for the 12×12 quarter tile ──────
    #
    # The quarter tile is now HALF×HALF = 12×12 (indices 0..11).
    # Original patterns were designed for a 6×6 quarter (indices 0..5).
    # Each original index i → 2*i, except the last edge (5 → 11 = LAST).

    def set_curve_walls_old(self, quarter_tile, tile_rep, row_quarter_map, col_quarter_map):
        """Sets curve walls in the tile representation based on the quarter tile."""
        L = LAST  # 11
        self.set_curve_walls_new(quarter_tile, tile_rep, row_quarter_map, col_quarter_map)

    def set_curve_walls_new(self, quarter_tile, tile_rep, row_quarter_map, col_quarter_map):
        """The quarter is taken into a slice of 8 x 8, the quarters of the first quarter are taken to analysis. A curve wall is
        setted basing on the amount of 1s of each quarter-quarter and its diagonal."""
        inner_quarter = quarter_tile[1:LAST, 1:LAST]  # 1:10, the inner 8×8 region of the quarter tile
        top_left = inner_quarter[0:inner_quarter.shape[0]//2, 0:inner_quarter.shape[1]//2]      # 0:6, upper-left 4×4 of the inner quarter
        top_right = inner_quarter[0:inner_quarter.shape[0]//2, inner_quarter.shape[1]//2:]     # 0:6, upper-right 4×4 of the inner quarter
        bottom_left = inner_quarter[inner_quarter.shape[0]//2:, 0:inner_quarter.shape[1]//2]   # 6:10, lower-left 4×4 of the inner quarter
        bottom_right = inner_quarter[inner_quarter.shape[0]//2:, inner_quarter.shape[1]//2:]  # 6:10, lower-right 4×4 of the inner quarter

        if top_left[0, 0] == 0: # If the vertex is 0, it is a curve wall, probably.
            diagonal = top_left.diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            # print(f"Top left diagonal: {diagonal}, Ones in diagonal: {ones_in_diag}")
            if len(ones_in_diag) > 0:
                # print(f"Setting right_bottom curve")
                self.set_right_bottom_curve_wall(tile_rep, row_quarter_map, col_quarter_map)
        else:
            diagonal = top_left.diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            ones_in_quarter = np.where(top_left == 1)[0]
            # print(f"Number of ones in top left quarter: {len(ones_in_quarter)}, Quarter size: {top_left.shape[0] * top_left.shape[1]}")
            if len(ones_in_diag) >= 2 and len(ones_in_quarter) >= (top_left.shape[0] * top_left.shape[1]) * 0.5:
                # print(f"Setting right_bottom curve (alternative detection)")
                self.set_right_bottom_curve_wall(tile_rep, row_quarter_map, col_quarter_map)

        if top_right[0, -1] == 0:
            diagonal = top_right[:, ::-1].diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            # print(f"Top right diagonal: {diagonal}, Ones in diagonal: {ones_in_diag}")
            if len(ones_in_diag) > 0:
                # print(f"Setting left_bottom curve")
                self.set_left_bottom_curve_wall(tile_rep, row_quarter_map, col_quarter_map)
        else:
            diagonal = top_right[:, ::-1].diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            ones_in_quarter = np.where(top_right == 1)[0]
            # print(f"Number of ones in top right quarter: {len(ones_in_quarter)}, Quarter size: {top_right.shape[0] * top_right.shape[1]}")
            if len(ones_in_diag) >= 2 and len(ones_in_quarter) >= (top_right.shape[0] * top_right.shape[1]) * 0.5:
                # print(f"Setting left_bottom curve (alternative detection)")
                self.set_left_bottom_curve_wall(tile_rep, row_quarter_map, col_quarter_map)

        if bottom_left[-1, 0] == 0:
            diagonal = bottom_left[:, ::-1].diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            # print(f"Bottom left diagonal: {diagonal}, Ones in diagonal: {ones_in_diag}")
            if len(ones_in_diag) > 0:
                # print(f"Setting right_top curve")
                self.set_right_top_curve_wall(tile_rep, row_quarter_map, col_quarter_map)
        else:
            diagonal = bottom_left[:, ::-1].diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            ones_in_quarter = np.where(bottom_left == 1)[0]
            # print(f"Number of ones in bottom left quarter: {len(ones_in_quarter)}, Quarter size: {bottom_left.shape[0] * bottom_left.shape[1]}")
            if len(ones_in_diag) >= 2 and len(ones_in_quarter) >= (bottom_left.shape[0] * bottom_left.shape[1]) * 0.5:
                # print(f"Setting right_top curve (alternative detection)")
                self.set_right_top_curve_wall(tile_rep, row_quarter_map, col_quarter_map)

        if bottom_right[-1, -1] == 0:
            diagonal = bottom_right.diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            # print(f"Bottom right diagonal: {diagonal}, Ones in diagonal: {ones_in_diag}")
            if len(ones_in_diag) > 0:
                # print(f"Setting left_top curve")
                self.set_left_top_curve_wall(tile_rep, row_quarter_map, col_quarter_map)
        else:
            diagonal = bottom_right.diagonal()
            ones_in_diag = np.where(diagonal == 1)[0]
            ones_in_quarter = np.where(bottom_right == 1)[0]
            # print(f"Number of ones in bottom right quarter: {len(ones_in_quarter)}, Quarter size: {bottom_right.shape[0] * bottom_right.shape[1]}")
            if len(ones_in_diag) >= 2 and len(ones_in_quarter) >= (bottom_right.shape[0] * bottom_right.shape[1]) * 0.5:
                # print(f"Setting left_top curve (alternative detection)")
                self.set_left_top_curve_wall(tile_rep, row_quarter_map, col_quarter_map)

    def set_curve_walls(self, quarter_tile, tile_rep, row_quarter_map, col_quarter_map):
        """Sets curve walls in quarter if the quarter tile has a curve wall pattern."""
        if not self.is_curve_wall(quarter_tile):
            return

        self.set_curve_walls_old(quarter_tile, tile_rep, row_quarter_map, col_quarter_map)

    def is_curve_wall(self, tile_rep):
        """Determines if a quarter tile has a curve wall pattern if the inner region of it has at least two 1s."""
        center_row = (tile_rep.shape[0] // 2) - 1
        center_col = (tile_rep.shape[1] // 2) - 1
        inner_rep = tile_rep[max(center_row - 4, 0):center_row + 5, max(center_col - 4, 0):center_col + 5]  # 4 pixels around the center in every direction
        ones_in_inner = np.where(inner_rep == 1)[0]
        if len(ones_in_inner) == 0:
            return False
        if len(ones_in_inner) >= 2:
            return True

    # ── Straight wall detection ───────────────────────────────────────────────
    def set_straight_walls(self, quarter_tile, tile_rep, row_quarter_map, col_quarter_map):
        """Sets straight walls in the tile rep after making an 'or' between adjacent rows and columns of the quarter tile."""
        
        # NOTE: This cleaning operation is done since the noisy GPS can result in a quarter tile having a 'cropped' wall.
        top_row = quarter_tile[0, :]
        second_top_row = quarter_tile[1, :]
        for col in range(len(top_row)):
            if top_row[col] == 0 and second_top_row[col] == 1:
                top_row[col] = 1
        quarter_tile[0, :] = top_row

        bottom_row = quarter_tile[-1, :]
        second_bottom_row = quarter_tile[-2, :]
        for col in range(len(bottom_row)):
            if bottom_row[col] == 0 and second_bottom_row[col] == 1:
                bottom_row[col] = 1
        quarter_tile[-1, :] = bottom_row

        left_col = quarter_tile[:, 0]
        second_left_col = quarter_tile[:, 1]
        for row in range(len(left_col)):
            if left_col[row] == 0 and second_left_col[row] == 1:
                left_col[row] = 1
        quarter_tile[:, 0] = left_col

        right_col = quarter_tile[:, -1]
        second_right_col = quarter_tile[:, -2]
        for row in range(len(right_col)):
            if right_col[row] == 0 and second_right_col[row] == 1:
                right_col[row] = 1
        quarter_tile[:, -1] = right_col

        if np.all(quarter_tile[:, 0] == 1):
            self.set_left_straight_wall(tile_rep, row_quarter_map, col_quarter_map)

        if np.all(quarter_tile[:, -1] == 1):
            self.set_right_straight_wall(tile_rep, row_quarter_map, col_quarter_map)

        if np.all(quarter_tile[0, :] == 1):
            self.set_top_straight_wall(tile_rep, row_quarter_map, col_quarter_map)

        if np.all(quarter_tile[-1, :] == 1):
            self.set_bottom_straight_wall(tile_rep, row_quarter_map, col_quarter_map)

    def custom_round(self, value):
        """Rounds a value to the nearest integer, rounding up for positive values and down for negative values."""
        if value > 0:
            return math.ceil(value)
        else:
            return math.floor(value)

    def crop_to_grid(self, gray, tile=TILE_SIZE_PX, bg=255, threshold=None, init_pixel=None):
        """Crops the map to the grid of tiles, ensuring the dimensions are multiples of the tile size."""
        if threshold is not None:
            work = np.where(gray <= threshold, 0, 255).astype(np.uint8)
        else:
            work = gray
        cv2.imwrite("Mapa que recortamos en map_rect, después de aplicar el threshold.png", work)

        h, w = work.shape
        has_pixel = work != bg
        col_counts = has_pixel.sum(axis=0)
        row_counts = has_pixel.sum(axis=1)

        best_col = int(np.argmax(col_counts))
        best_col = int(utils.get_nearest_multiple(best_col, tile/2))
        best_row = int(np.argmax(row_counts))
        best_row = int(utils.get_nearest_multiple(best_row, tile/2))

        cols_with_pixels = np.where(col_counts > 0)[0]
        rows_with_pixels = np.where(row_counts > 0)[0]

        min_col = int(utils.get_nearest_multiple(int(cols_with_pixels[0]), tile/2))
        max_col = int(utils.get_nearest_multiple(int(cols_with_pixels[-1]), tile/2))
        min_row = int(utils.get_nearest_multiple(int(rows_with_pixels[0]), tile/2))
        max_row = int(utils.get_nearest_multiple(int(rows_with_pixels[-1]), tile/2))

        diff_left = best_col - min_col
        diff_right = max_col  - best_col
        diff_top = best_row - min_row
        diff_bottom = max_row  - best_row

        left_remaining = tile - (diff_left % tile) if diff_left % tile != 0 else 0
        right_remaining = tile - (diff_right % tile) if diff_right % tile != 0 else 0
        top_remaining = tile - (diff_top % tile) if diff_top % tile != 0 else 0
        bottom_remaining = tile - (diff_bottom % tile) if diff_bottom % tile != 0 else 0

        left = max(min_col - left_remaining, 0)
        right = min(max_col + right_remaining, w)
        top = max(min_row - top_remaining, 0)
        bottom = min(max_row + bottom_remaining, h)

        cropped = work[top:bottom, left:right]

        out_h = bottom - top
        out_w = right  - left

        if cropped.shape != (out_h, out_w):
            padded = np.full((out_h, out_w), bg, dtype=work.dtype)
            padded[:cropped.shape[0], :cropped.shape[1]] = cropped
            result = padded
        else:
            result = cropped

        if init_pixel is not None:
            init_pixel.x = init_pixel.x - left
            init_pixel.y = init_pixel.y - top
            return result, init_pixel

        return result

    def mapping_pixels(self, robot_start_pos):
        """Whole mapping logic."""

        img = self.map.img
        cv2.imwrite("Mapa que nos mandaron.png", img)

        init_pixel = self.map.world_to_pixel(Vector(0, 0))

        # Offset by half a tile (HALF = 12) to land on the tile's top-left corner.
        init_pixel.x = init_pixel.x - HALF
        init_pixel.y = init_pixel.y - HALF

        map_rect, init_pixel = self.crop_to_grid(img, tile=TILE_SIZE_PX, bg=255, threshold=190, init_pixel=init_pixel)
        cv2.imwrite("Mapa que recortamos en map_rect, después de recortar.png", map_rect)

        init_tile = self.pixel_to_tile((init_pixel.x, init_pixel.y))
        print(f"Init tile: {init_tile}")

        map_rect[map_rect == 0] = 1
        map_rect[map_rect == 255] = 0
        np.savetxt("Mapa que recortamos, después de cambiar 0 por 1 y 255 por 0.txt", map_rect, fmt='%d', delimiter=';')

        # ── Straight walls ────────────────────────────────────────────────────
        for col_tile_pixel in range(0, map_rect.shape[1], TILE_SIZE_PX):
            for row_tile_pixel in range(0, map_rect.shape[0], TILE_SIZE_PX):
                tile_pos = self.pixel_to_tile((col_tile_pixel + HALF, row_tile_pixel + HALF))
                tile_pos = (tile_pos[0] - init_tile[0], tile_pos[1] - init_tile[1])
                # print(f"Tile pos: {tile_pos}")

                if tile_pos not in self.tiles:
                    self.add_tiles({tile_pos: TileType.UNKNOWN})

                tile_rep = self.tiles[tile_pos].rep
                tile_area = self.tiles[tile_pos].area

                for row_quarter_pixel in range(0, TILE_SIZE_PX, HALF):
                    for col_quarter_pixel in range(0, TILE_SIZE_PX, HALF):
                        quarter_tile = map_rect[
                            row_tile_pixel + row_quarter_pixel: row_tile_pixel + row_quarter_pixel + HALF,
                            col_tile_pixel + col_quarter_pixel: col_tile_pixel + col_quarter_pixel + HALF
                        ]

                        col_quarter_map = 0 if col_quarter_pixel == 0 else 2
                        row_quarter_map = 0 if row_quarter_pixel == 0 else 2

                        # Analyzing quarter tiles and setting walls in 3x3 quarter tile rep (final matrix will be 5x5 for each tile, 4x4 + 1 map).
                        self.set_straight_walls(quarter_tile, tile_rep, row_quarter_map, col_quarter_map)

        # ── Curved walls ──────────────────────────────────────────────────────
                if tile_area != 1 and tile_area != 2 and tile_area != 4: # Curve walls must only appear in area 3 tiles. However, if we see a curve
                    # wall in a tile we could not reach, its area is None, so we also check those cases.
                    for row_quarter_pixel in range(0, TILE_SIZE_PX, HALF):
                        for col_quarter_pixel in range(0, TILE_SIZE_PX, HALF):
                            quarter_tile = map_rect[
                                row_tile_pixel + row_quarter_pixel: row_tile_pixel + row_quarter_pixel + HALF,
                                col_tile_pixel + col_quarter_pixel: col_tile_pixel + col_quarter_pixel + HALF
                            ]

                            col_quarter_map = 0 if col_quarter_pixel == 0 else 2
                            row_quarter_map = 0 if row_quarter_pixel == 0 else 2

                            tile_type = self.tiles[tile_pos].tile_type
                            if tile_type != TileType.OBSTACLE: # To avoid obstacles/walls detection bug! (TODO for next year...)
                                # Analyzing quarter tiles and setting walls in 3x3 quarter tile rep (final matrix will be 5x5 for each tile, 4x4 + 1 map).
                                self.set_curve_walls(quarter_tile, tile_rep, row_quarter_map, col_quarter_map)

        # ── Build final matrix (4n+1) – unchanged from mapper.py ─────────────
        min_row = min(t[1] for t in self.tiles.keys())
        min_col = min(t[0] for t in self.tiles.keys())
        max_row = max(t[1] for t in self.tiles.keys())
        max_col = max(t[0] for t in self.tiles.keys())
        tile_rows = max_row - min_row + 1
        tile_cols = max_col - min_col + 1

        final_matrix = np.zeros((tile_rows * 4 + 1, tile_cols * 4 + 1), dtype=int)
        matrix_col = 0
        for tile_col in range(min_col, max_col + 1):
            matrix_row = 0
            for tile_row in range(min_row, max_row + 1):
                tile_pos = (tile_col, tile_row)
                if tile_pos not in self.tiles:
                    matrix_row += 4
                    continue
                tile_rep = self.tiles[tile_pos].rep
                for col in range(5):
                    for row in range(5):
                        new = tile_rep[row, col]
                        old = final_matrix[matrix_row + row, matrix_col + col]
                        """For the supervisor, the tiles only have right and bottom walls (except for the corner col, rows) 
                        and always replace old values for the new ones. This is a problem for curve walls,
                        thats why we have to evaluate the new value of the quarter and the type of curve."""
                        if (new == 1 and old == 0) or (old == 1 and new == 0):
                            final_matrix[matrix_row + row, matrix_col + col] = 1
                        elif new in [Curve.LEFT_TOP.value, Curve.RIGHT_TOP.value,
                                     Curve.RIGHT_BOTTOM.value, Curve.LEFT_BOTTOM.value]:
                            final_matrix[matrix_row + row, matrix_col + col] = new
                            """We made a special decodification for each curve wall vertex to recognise them while placing signs,
                            even if the wall's vertex should be replaced by a 1 in the final matrix."""

                        elif old == Curve.LEFT_BOTTOM.value:
                            if col == 0 and row == 2 and new == 1:
                                if tile_rep[2, 1] == 1 and tile_rep[2, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            elif col == 0 and row == 0 and new == 1:
                                if tile_rep[0, 1] == 1 and tile_rep[0, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            else:
                                final_matrix[matrix_row + row, matrix_col + col] = Curve.LEFT_BOTTOM.value

                        elif old == Curve.LEFT_TOP.value:
                            if col == 4 and row == 0 and new == 1:
                                if tile_rep[1, 4] == tile_rep[2, 4] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            elif col == 0 and row == 4 and new == 1:
                                if tile_rep[4, 1] == 1 and tile_rep[4, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            elif col == 0 and row == 2 and new == 1:
                                if tile_rep[2, 1] == 1 and tile_rep[2, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            elif col == 2 and row == 0 and new == 1:
                                if tile_rep[1, 2] == 1 and tile_rep[2, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value

                        elif old == Curve.RIGHT_TOP.value:
                            if col == 2 and row == 0 and new == 1:
                                if tile_rep[1, 2] == 1 and tile_rep[2, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value
                            elif col == 0 and row == 4 and new == 1:
                                if tile_rep[4, 1] == 1 and tile_rep[4, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.RIGHT_TOP.value
                            elif col == 0 and row == 0 and new == 1:
                                if tile_rep[0, 1] == 1 and tile_rep[0, 2] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.RIGHT_TOP.value
                                elif tile_rep[1, 0] == tile_rep[2, 0] == 1:
                                    final_matrix[matrix_row + row, matrix_col + col] = Curve.RIGHT_TOP.value

                        elif old == Curve.VALUES_REPLACED_BY_1.value and new == 1:
                            final_matrix[matrix_row + row, matrix_col + col] = Curve.VALUES_REPLACED_BY_1.value

                matrix_row += 4
            matrix_col += 4

        final_matrix = final_matrix.astype(str)
        np.savetxt("Matriz final después de poner las paredes.txt", final_matrix, fmt='%s', delimiter=';')

        # ── Signs ─────────────────────────────────────────────────────────────
        top_left_pos = Vector(round(robot_start_pos.x, 2) - 0.12 / 2 + 0.12 * min_col,
                              round(robot_start_pos.y, 2) - 0.12 / 2 + 0.12 * min_row)
        bottom_right_pos = Vector(round(robot_start_pos.x, 2) + 0.12 / 2 + 0.12 * max_col,
                                  round(robot_start_pos.y, 2) + 0.12 / 2 + 0.12 * max_row)
        signs = []
        for pos, tile_pos_and_let in self.signs.items():
            map_col = round(self.map_range(round(pos.x, 2), top_left_pos.x, bottom_right_pos.x, 0, final_matrix.shape[1] - 1), 0)
            map_row = round(self.map_range(round(pos.y, 2), top_left_pos.y, bottom_right_pos.y, 0, final_matrix.shape[0] - 1), 0)
            signs.append((tile_pos_and_let[0], (map_row, map_col), tile_pos_and_let[1], pos))
        signs.sort(key=lambda x: (x[0][1], x[0][0], x[3].y, x[3].x))

        for sign in signs:
            map_row = int(sign[1][0])
            map_col = int(sign[1][1])
            letter = sign[2]
            tile_row = sign[0][1]
            tile_col = sign[0][0]

            quarter_side = self.world_to_quarter_side(sign[3], tile_cols, tile_rows)
            type_of_wall = self.type_of_wall(map_row, map_col, final_matrix)

            if quarter_side == 0:
                row_off, col_off = 0, 0
            elif quarter_side == 1:
                row_off, col_off = 0, 2
            elif quarter_side == 2:
                row_off, col_off = 2, 0
            elif quarter_side == 3:
                row_off, col_off = 2, 2

            if type_of_wall != "c":
                quarter_mat_debug = final_matrix[
                    (tile_row - min_row) * 4 + row_off: (tile_row - min_row) * 4 + row_off + 3,
                    (tile_col - min_col) * 4 + col_off: (tile_col - min_col) * 4 + col_off + 3
                ]
                self.place_letter(letter, map_row, map_col, final_matrix)
                continue

            quarter_mat = final_matrix[
                (tile_row - min_row) * 4 + row_off: (tile_row - min_row) * 4 + row_off + 3,
                (tile_col - min_col) * 4 + col_off: (tile_col - min_col) * 4 + col_off + 3
            ]

            if quarter_mat[0, 2] != '0' and quarter_mat[1, 2] != '0' and quarter_mat[2, 0] != '0' and \
               quarter_mat[2, 1] != '0' and quarter_mat[2, 2] in ('9', '8', '7', '6', '10'):
                self.place_letter(letter, 2, 1, quarter_mat)
            elif quarter_mat[0, 0] != '0' and quarter_mat[1, 0] != '0' and quarter_mat[2, 1] != '0' and \
                 quarter_mat[2, 2] != '0' and (quarter_mat[2, 0] in ('9', '8', '7', '6', '10')):
                self.place_letter(letter, 2, 1, quarter_mat)
            elif quarter_mat[0, 0] != '0' and quarter_mat[0, 1] != '0' and quarter_mat[1, 2] != '0' and \
                 quarter_mat[2, 2] != '0' and (quarter_mat[0, 2] in ('9', '8', '7', '6', '10')):
                self.place_letter(letter, 0, 1, quarter_mat)
            elif quarter_mat[0, 1] != '0' and quarter_mat[0, 2] != '0' and quarter_mat[1, 0] != '0' and \
                 quarter_mat[2, 0] != '0' and (quarter_mat[0, 0] in ('9', '8', '7', '6', '10')):
                self.place_letter(letter, 0, 1, quarter_mat)
            else:
                print("The letter position in CURVE WALL is not defined, this should be IMPOSSIBLE!!")
                print(f"Quarter mat: \n {quarter_mat}")
                print(f"letter: {letter}")
            print("-" * 20)
                
        final_matrix[final_matrix == str(Curve.LEFT_TOP.value)] = '0'
        final_matrix[final_matrix == str(Curve.RIGHT_TOP.value)] = '0'
        final_matrix[final_matrix == str(Curve.RIGHT_BOTTOM.value)] = '0'
        final_matrix[final_matrix == str(Curve.LEFT_BOTTOM.value)] = '0'
        final_matrix[final_matrix == str(Curve.VALUES_REPLACED_BY_1.value)] = '1'

        # ── Tile types ────────────────────────────────────────────────────────
        matrix_col = 0
        for tile_col in range(min_col, max_col + 1):
            matrix_row = 0
            for tile_row in range(min_row, max_row + 1):
                tile_pos = (tile_col, tile_row)
                if tile_pos not in self.tiles:
                    matrix_row += 4
                    continue
                tile_type = self.tiles[tile_pos].tile_type
                tile_area = self.tiles[tile_pos].area
                if tile_type == TileType.STANDARD or tile_type == TileType.UNKNOWN:
                    matrix_row += 4
                    continue
                else:
                    matrix_row = (tile_row - min_row) * 4
                    matrix_col = (tile_col - min_col) * 4
                    final_matrix[matrix_row + 1, matrix_col + 1] = tile_type.value
                    final_matrix[matrix_row + 1, matrix_col + 3] = tile_type.value
                    final_matrix[matrix_row + 3, matrix_col + 1] = tile_type.value
                    final_matrix[matrix_row + 3, matrix_col + 3] = tile_type.value
                matrix_row += 4
            matrix_col += 4


        # ── Obstacles ─────────────────────────────────────────────────────────
        for tile_row in range(min_row, max_row + 1):
            for tile_col in range(min_col, max_col + 1):
                tile_pos = (tile_col, tile_row)
                if tile_pos not in self.tiles:
                    continue
                # print(f"TILE POS {tile_pos} ---------- OBSTACLES {self.obstacles} ")

                if tile_pos in self.obstacles:
                    matrix_row = (tile_row - min_row) * 4
                    matrix_col = (tile_col - min_col) * 4
                    # Limpiar TODO a '0'
                    final_matrix[matrix_row + 1: matrix_row + 4, matrix_col + 1: matrix_col + 4] = '0'
                    
                    # vecinos
                    top_pos = (tile_col, tile_row - 1)
                    bottom_pos = (tile_col, tile_row + 1)
                    left_pos = (tile_col - 1, tile_row)
                    right_pos = (tile_col + 1, tile_row)

                    top_neighbor = self.tiles.get(top_pos)
                    bottom_neighbor = self.tiles.get(bottom_pos)
                    left_neighbor = self.tiles.get(left_pos)
                    right_neighbor = self.tiles.get(right_pos)

                    # TOP row
                    if top_neighbor and top_pos not in self.obstacles:
                        top_neighbour_bottom_row = final_matrix[matrix_row - 1, matrix_col: matrix_col + 5]
                        tile_top_row = final_matrix[matrix_row, matrix_col: matrix_col + 5]

                        inner_bottom_neighbour = top_neighbour_bottom_row[1: 4]
                        inner_top_row = tile_top_row[1: 4]

                        ones_in_inner_top_bottom_row = np.char.count(inner_bottom_neighbour, sub='1').sum()
                        ones_in_inner_top_row = np.char.count(inner_top_row, sub='1').sum()

                        if ones_in_inner_top_row == 1:
                            index = np.where(inner_bottom_neighbour == '1')[0]
                            if len(index) == 0:
                                final_matrix[matrix_row, matrix_col + 1 : matrix_col + 4] = '0'

                                tile_top_row = final_matrix[matrix_row, matrix_col + 1 : matrix_col + 4]

                    #Bottom row
                    if bottom_neighbor and bottom_pos not in self.obstacles:
                        bottom_neighbour_top_row = final_matrix[matrix_row + 5, matrix_col: matrix_col + 5]
                        # print(f"Top row of the bottom neighbour tile: {bottom_neighbour_top_row}")
                        tile_bottom_row = final_matrix[matrix_row + 4, matrix_col: matrix_col + 5]
                        # print(f"Actual tile bottom row: {tile_bottom_row}")

                        inner_top_neighbour = bottom_neighbour_top_row[1 : 4]
                        inner_bottom_row = tile_bottom_row[1 : 4]

                        ones_in_inner_bottom_top_row = np.char.count(inner_top_neighbour, sub='1').sum()
                        ones_in_inner_bottom_row = np.char.count(inner_bottom_row, sub='1').sum()

                        if ones_in_inner_bottom_row == 1:
                            index = np.where(inner_top_neighbour == '1')[0]
                            print(f"Index : {index}")
                            if len(index) == 0:
                                final_matrix[matrix_row + 4 , matrix_col + 1 : matrix_col + 4] = '0'

                                tile_bottom_row = final_matrix[matrix_row + 4 , matrix_col + 1 : matrix_col + 4]

                    #LEFT col
                    if left_neighbor and left_pos not in self.obstacles:
                        left_neighbour_right_col = final_matrix[matrix_row: matrix_row + 5, matrix_col - 1]
                        tile_left_col = final_matrix[matrix_row: matrix_row + 5, matrix_col]
                        inner_right_neighbour = left_neighbour_right_col[1 : 4]
                        inner_left_col = tile_left_col[1 : 4]

                        ones_in_inner_left_right_col = np.char.count(inner_right_neighbour, sub='1').sum()
                        ones_in_inner_left_col = np.char.count(inner_left_col, sub='1').sum()

                        if ones_in_inner_left_col == 1:
                            index = np.where(inner_right_neighbour == '1')[0]
                            print(f"Index : {index}")
                            if len(index) == 0:
                                final_matrix[matrix_row + 1 : matrix_row + 4, matrix_col] = '0'

                                tile_left_col = final_matrix[matrix_row + 1 : matrix_row + 4, matrix_col]
                                # print(f"Tile col after: {tile_right_col}")

                    #RIGHT col
                    if right_neighbor and right_pos not in self.obstacles:
                        right_neighbour_left_col = final_matrix[matrix_row: matrix_row + 5, matrix_col + 5]
                        tile_right_col = final_matrix[matrix_row: matrix_row + 5, matrix_col + 4]
                        inner_left_neighbour = right_neighbour_left_col[1 : 4]
                        inner_right_col = tile_right_col[1 : 4]

                        ones_in_inner_right_neighbour = np.char.count(inner_left_neighbour, sub='1').sum()
                        ones_in_inner_right_col = np.char.count(inner_right_col, sub='1').sum()

                        if ones_in_inner_right_col == 1:
                            index = np.where(inner_left_neighbour == '1')[0]
                            print(f"Index : {index}")
                            if len(index) == 0:
                                final_matrix[matrix_row + 1 : matrix_row + 4, matrix_col + 4] = '0'

                                tile_right_col = final_matrix[matrix_row + 1 : matrix_row + 4, matrix_col + 4]
                                # print(f"Tile col after: {tile_right_col}")
                            

                 
                    final_matrix[matrix_row + 1: matrix_row + 4, matrix_col + 1: matrix_col + 4] = '0'
                    final_matrix[matrix_row + 1, matrix_col + 1] = TileType.OBSTACLE.value
                    final_matrix[matrix_row + 1, matrix_col + 3] = TileType.OBSTACLE.value
                    final_matrix[matrix_row + 3, matrix_col + 1] = TileType.OBSTACLE.value
                    final_matrix[matrix_row + 3, matrix_col + 3] = TileType.OBSTACLE.value

        # ── Area 4 flood-fill ─────────────────────────────────────────────────
        print(f"Tiles: {len(self.tiles)} Dict: {self.tiles}")
        np.savetxt("Matriz final antes de flood fill.txt", final_matrix, fmt='%s', delimiter=';')
        print("-"*20)
        self.flood_fill_area_4()

        for tile_row in range(min_row, max_row + 1):
            for tile_col in range(min_col, max_col + 1):
                tile_pos = (tile_col, tile_row)
                if tile_pos not in self.tiles:
                    continue
                tile = self.tiles[tile_pos]
                if tile.area == 4:
                    matrix_row = (tile_row - min_row) * 4
                    matrix_col = (tile_col - min_col) * 4
                    final_matrix[matrix_row: matrix_row + 5, matrix_col: matrix_col + 5] = '*'

        print("Almost finishing, replacing curve values...")
        np.savetxt("Mapa final.txt", final_matrix, fmt='%s', delimiter=';')
        print(f"Saved final matrix: \n {final_matrix}")

        return final_matrix

    # ── Flood-fill and neighbours (unchanged) ─────────────────────────────────

    def flood_fill_area_4(self):
        """Flood fill algorithm to mark all reachable tiles from area 4 tiles as area 4."""
        a4_tiles = [tile for tile in self.tiles.values() if tile.area == 4]
        if len(a4_tiles) == 0: return
        print(f"Flood fill: Starting flood fill for area 4 with {len(a4_tiles)} tiles")
        start = a4_tiles[0]
        frontier = [start]
        reached = set()
        reached.add(start)

        while not len(a4_tiles) == 0:
            current = a4_tiles.pop(0)
            if current.area is None:
                current.area = 4
            if current.tile_type == TileType.UNKNOWN:
                continue
            for next in self.get_valid_neighbours(current):
                if next.area in (1, 2, 3):
                    continue
                if self.is_color_passage(next):
                    continue
                if next not in reached:
                    reached.add(next)

        print(f"Flood fill: Finished. Reached {len(reached)} tiles.")
        for tile in reached:
            tile.area = 4

    def get_valid_neighbours(self, tile):
        """Returns a list of valid neighbouring tiles that can be reached from the given tile."""
        row = tile.pos[1]
        col = tile.pos[0]
        result = []

        checks = [
            ((col, row - 1), lambda n, t: not (n.rep[4, 1] != 0 and n.rep[4, 2] != 0 and n.rep[4, 3] != 0) and
                                          not (t.rep[0, 1] != 0 and t.rep[0, 2] != 0 and t.rep[0, 3] != 0)),
            ((col, row + 1), lambda n, t: not (n.rep[0, 1] != 0 and n.rep[0, 2] != 0 and n.rep[0, 3] != 0) and
                                          not (t.rep[4, 1] != 0 and t.rep[4, 2] != 0 and t.rep[4, 3] != 0)),
            ((col - 1, row), lambda n, t: not (n.rep[1, 4] != 0 and n.rep[2, 4] != 0 and n.rep[3, 4] != 0) and
                                          not (t.rep[1, 0] != 0 and t.rep[2, 0] != 0 and t.rep[3, 0] != 0)),
            ((col + 1, row), lambda n, t: not (n.rep[1, 0] != 0 and n.rep[2, 0] != 0 and n.rep[3, 0] != 0) and
                                          not (t.rep[1, 4] != 0 and t.rep[2, 4] != 0 and t.rep[3, 4] != 0)),
        ]

        for pos, passable in checks:
            neighbour = self.tiles.get(pos)
            if neighbour is None:
                continue
            if passable(neighbour, tile):
                inside = neighbour.rep[1:4, 1:4]
                if np.sum(inside != 0) >= 2:
                    result.append(neighbour)

        return result

    # def flood_fill_area_4(self):
    #     a4_tiles = [tile for tile in self.tiles.values() if tile.area==4]
    #     if len(a4_tiles) == 0: return
    #     start = a4_tiles[0]
    #     frontier = [start]
    #     reached = set()
    #     reached.add(start)

    #     while not len(frontier) == 0:
    #         current = frontier.pop(0)
    #         # print(f"Flood fill: Current tile {current.pos} with area {current.area}")

    #         if current.area == None:
    #             # print(f"Flood fill: Setting tile {current.pos} to area 4")
    #             current.area = 4

    #         if current.tile_type == TileType.UNKNOWN:
    #             continue

    #         for next in self.get_valid_neighbours(current):
    #             # if current.pos == (7, -3):
    #             #     print(f"ACAACA {next}")
    #             if next.area == 1 or next.area == 2 or next.area == 3:
    #                 continue

    #             if self.is_color_passage(next):
    #                 continue                

    #             if next not in reached:
    #                 frontier.append(next)
    #                 reached.add(next)

    # def get_valid_neighbours(self, tile):
    #     row = tile.pos[1]
    #     col = tile.pos[0]
    #     result = []

    #     # Upper tile
    #     row_to_check = row - 1
    #     col_to_check = col
    #     neighbour_tile = self.tiles.get((col_to_check, row_to_check))
    #     if neighbour_tile is not None:
    #         if not (neighbour_tile.rep[4,1] != 0 and neighbour_tile.rep[4,2] != 0 and neighbour_tile.rep[4,3] != 0) and \
    #             not (tile.rep[0,1] != 0 and tile.rep[0,2] != 0 and tile.rep[0,3] != 0):
    #             result.append(neighbour_tile)

    #     # Lower tile
    #     row_to_check = row + 1
    #     col_to_check = col
    #     neighbour_tile = self.tiles.get((col_to_check, row_to_check))
    #     if neighbour_tile is not None:
    #         if not(neighbour_tile.rep[0,1] != 0 and neighbour_tile.rep[0,2] != 0 and neighbour_tile.rep[0,3] != 0) and \
    #             not (tile.rep[4,1] != 0 and tile.rep[4,2] != 0 and tile.rep[4,3] != 0):
    #             result.append(neighbour_tile)
 
    #     # Left tile
    #     row_to_check = row
    #     col_to_check = col - 1
    #     neighbour_tile = self.tiles.get((col_to_check, row_to_check))
    #     if neighbour_tile is not None:
    #         if not(neighbour_tile.rep[1,4] != 0 and neighbour_tile.rep[2,4] != 0 and neighbour_tile.rep[3,4] != 0) and \
    #             not (tile.rep[1,0] != 0 and tile.rep[2,0] != 0 and tile.rep[3,0] != 0):
    #             result.append(neighbour_tile)
        
    #     # Right tile
    #     row_to_check = row
    #     col_to_check = col + 1
    #     neighbour_tile = self.tiles.get((col_to_check, row_to_check))
    #     if neighbour_tile is not None:
    #         if not(neighbour_tile.rep[1,0] != 0 and neighbour_tile.rep[2,0] != 0 and neighbour_tile.rep[3,0] != 0) and \
    #             not (tile.rep[1,4] != 0 and tile.rep[2,4] != 0 and tile.rep[3,4] != 0):
    #             result.append(neighbour_tile)

    #     return result

    def place_letter(self, letter, row, col, map_matrix):
        """Places a letter in the map matrix at the specified row and column, handling concatenation if necessary."""
        if map_matrix[row][col] == '0' or map_matrix[row][col] == '1' or map_matrix[row][col] in ['6', '7', '8', '9', '10']:
            map_matrix[row][col] = letter
        else:
            map_matrix[row][col] += letter

    def map_range(self, value, ori_min, ori_max, dest_min, dest_max):
        """Maps a value from one range to another."""
        return dest_min + (dest_max - dest_min) * (value - ori_min) / (ori_max - ori_min)

    def type_of_wall(self, row, col, final_matrix):
        """Determines the type of wall at a given position in the final matrix."""
        if col % 4 == 0 and final_matrix[row - 1][col] == '1' and final_matrix[row + 1][col] == '1':
            return "ev"
        if row % 4 == 0 and final_matrix[row][col - 1] == '1' and final_matrix[row][col + 1] == '1':
            return "eh"
        if col % 4 == 2 and (row % 4 == 1 or row % 4 == 3) and final_matrix[row - 1][col] == final_matrix[row + 1][col] == '1':
            return "iv"
        if row % 4 == 2 and (col % 4 == 1 or col % 4 == 3) and final_matrix[row][col - 1] == final_matrix[row][col + 1] == '1':
            return "ih"
        return "c"

    def world_to_quarter_side(self, pos, total_col, total_row):
        """Determines the quarter side of a position in the world."""
        left = False
        sup = False
        abs_x = pos.x % 0.12
        abs_y = pos.y % 0.12
        if total_col % 2 == 0:
            left = abs_x > 0.06 and abs_x < 0.12
        if total_col % 2 == 1:
            left = abs_x > 0 and abs_x < 0.06
        if total_row % 2 == 0:
            sup = abs_y > 0.06 and abs_y < 0.12
        if total_row % 2 == 1:
            sup = abs_y > 0 and abs_y < 0.06

        if left and sup: return 0
        elif not left and sup: return 1
        elif left and not sup: return 2
        else: return 3 
