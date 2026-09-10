from vector import Vector
from collections import deque
from heapq import heappush, heappop
import math
import utils
from navigation.map import Map, MINITILE_SIZE_PX

class Navigator:
    def __init__(self):
        self.map = Map()

        self.pixel_paths = {}

        # These variables are just to send the visualizer
        self.start = None
        self.came_from = {}
        self.cost_so_far = {}
        self.penalty_scores = {}
        self.priority_scores = {}
        self.walls_around = {}
        self.path = []
    
    def update(self, robot_position, lidar_points, surrounding_tiles):
        self.map.update(robot_position, lidar_points, surrounding_tiles)

    def find_closest_traversable_point(self, target, robot_pos, special_case=False):
        """Finds the closest traversable point of a target world pos."""
        return self.map.find_closest_traversable_point(target, robot_pos, special_case)
    
    def is_traversable(self, point):
        """Checks if a point is traversable in the map."""
        return self.map.is_traversable(point)

    def mark_lop(self, position):
        """Marks a lop point and invalidates pixel paths related to it."""
        self.map.mark_lop(position)
        self.invalidate_pixel_paths(position)

    def mark_position_as_visited(self, position):
        """Marks a position as visited in the map."""
        self.map.mark_position_as_visited(position)

    def mark_position_as_obstructed(self, position):
        """Marks a position as obstructed in the map and invalidates pixel paths related to it."""
        self.map.mark_as_obstructed(position)

        # Invalidate all cached paths to the minitile at position
        self.invalidate_pixel_paths(position)

    def invalidate_pixel_paths(self, position):
        """Invalidates all cached pixel paths that start or end in the minitile at position."""
        minitile = self.map.world_to_minitile(position)
        history_paths = self.pixel_paths.get((minitile.x, minitile.y))
        if history_paths == None: return
        history_paths.clear()
    
    def find_cached_pixel_path(self, src, dst):
        """Checks if theres an already calculated pixel path from src to dst."""
        history_paths = self.pixel_paths.get(dst)
        if history_paths == None: return None
        src_path = history_paths.get(src)
        if src_path == None: return None
        for p in src_path:
            if not self.map.is_pixel_traversable(p):
                history_paths[src] = None
                return None
        return src_path
    
    def store_pixel_path(self, src, dst, path):
        """Stores a pixel path from src to dst in the cache."""
        history_paths = self.pixel_paths.get(dst)
        if history_paths == None:
            self.pixel_paths[dst] = {src: path}
        else: # Updates the path if it already exists a way of reaching dst
            history_paths[src] = path

    def calculate_cost_pixel_path(self, pixels):
        """Calculates the cost of a pixel path based on distance."""
        cost = 0
        adicional_cost = 8
        # consecutive_swamps = 0
        for i in range(len(pixels) - 1): #NOTE: If we do not subtract, the b item will be out of range
            a = pixels[i]
            b = pixels[i+1]
            cost += a.distance_to(b)
            if self.map.is_pixel_swamp(b):
                # consecutive_swamps += 1
                # print(f"sumo 1")
                cost += 8.0  # Penalty for swamp pixels
                world_pixel = self.map.pixel_to_world(b)
                minitile_pixel = self.map.world_to_minitile(world_pixel)
                _, visits = self.map.minitiles.get((minitile_pixel.x, minitile_pixel.y), (False, 0))
                # print(f"VISITAS {visits}")
                if visits != 0:
                    cost += visits * adicional_cost   # Additional penalty for previously visited swamp pixels
                    # print(f"MINITILE YA VISITADO COSTO: {cost}")
                    # print(f"VISITS: {visits}")
            # else:
            #     consecutive_swamps = 0
        return cost
    
    def is_boring(self, minitile_tuple):
        """Checks if a minitile is boring, meaning it has one or no walls around it."""
        # TODO(Richo): Is this okay???
        return self.count_walls_around(minitile_tuple) <= 1

    def find_path(self, position, use_cache=True):
        """Finds a path from the robot's position (terms of minitle) to a minitile not visited yet."""
        self.start = position

        """Performs a Dijkstra on the map's minitiles starting from the robot's position and finishing
        in a minitile not visited yet. If no minitile is found, it returns the path to the start.
        A very good explanation of the algorithm can be found here:
        https://www.redblobgames.com/pathfinding/a-star/introduction.html#breadth-first-search"""
        robot_pos = self.map.world_to_minitile(position)
        start = (robot_pos.x, robot_pos.y)
        
        # The order in which we check the neighbour minitiles is important, we want to prioritize
        # horizontal and vertical movements vs diagonals, so we put them first in the list
        cardinals = [(-1, 0), (0, -1), ( 1, 0), (0,  1)]
        diagonals = [(-1, -1), (1, -1), ( 1,  1), (-1, 1)]
        directions = cardinals + diagonals

        frontier = [] # Priority queue of points to explore
        # Now, each each tuple has these elements: [priority_score, counter, minitile_tuple]
        heappush(frontier, [0.0, 0, start]) # Counter used to break ties in the priority queue
        came_from = {} # path A->B is stored as came_from[B] == A
        came_from[start] = None
        cost_so_far = {start: 0}
        # consecutive_swamps = {start:0}
        self.penalty_scores = {}
        self.priority_scores = {}
        pixel_paths = {}
        pixel_pos = {start: self.map.world_to_pixel(position)}

        candidates = []
        counter = 0
        while len(frontier) > 0:
            _, _, current = heappop(frontier)

            # If we find 15 candidates (not visited), we stop searching
            is_visited, _ = self.map.is_visited(current)
            if current != start and not is_visited:
                if not self.is_boring(current) and not current in candidates:
                    candidates.append(current)
                    if len(candidates) > 15:
                        break
            
            # Verifying the neighbours of the current minitile.
            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.map.is_valid(neighbor):
                    continue
                
                pixel_path = None # We will try to find a cached pixel path first, if not found, we will calculate it.
                if use_cache:
                    pixel_path = self.find_cached_pixel_path(current, neighbor)
                if pixel_path == None:
                    current_px = pixel_pos.get(current)
                    if current_px == None:
                        current_px = self.map.world_to_pixel(self.map.minitile_to_world(Vector.from_tuple(current)))

                    neighbor_px = pixel_pos.get(neighbor)
                    if neighbor_px == None:
                        neighbor_px = self.map.world_to_pixel(self.map.minitile_to_world(Vector.from_tuple(neighbor)))

                    pixel_path = self.find_pixel_path(current_px, neighbor_px)
                    pixel_path = self.smooth_path(pixel_path) # We smooth the path to avoid unnecessary zig-zagging.
                    self.store_pixel_path(current, neighbor, pixel_path)
                    self.store_pixel_path(neighbor, current, pixel_path[::-1]) # To avoid redundant calculations, we store the reverse path as well.

                if len(pixel_path) == 0:
                    continue 
                
                def get_penalty_score(wall_count):
                    return 0 # -25.0/7 * wall_count + 20 
                
                # The neighbour cost is the acumulated cost of the current minitile plus the cost calculated from the current minitile to the neighbour
                neighbor_cost = cost_so_far[current] + self.calculate_cost_pixel_path(pixel_path)
                walls_around = self.count_walls_around(neighbor)
                penalty_score = get_penalty_score(walls_around)

                # We combine interest and cost into a single score
                priority_score = neighbor_cost + penalty_score

                #If the robot finds a cheaper path than the previous one, we take that path
                if neighbor not in cost_so_far or neighbor_cost < cost_so_far[neighbor]:
                    # consecutive_swamps[neighbor] = new_consecutive
                    pixel_paths[(current, neighbor)] = pixel_path
                    pixel_pos[neighbor] = pixel_path[-1]
                    cost_so_far[neighbor] = neighbor_cost
                    self.penalty_scores[neighbor] = penalty_score
                    self.priority_scores[neighbor] = priority_score
                    self.walls_around[neighbor] = walls_around
                    came_from[neighbor] = current
                    counter += 1

                    # Push the neighbour into the priority queue with its priority score and a counter to break ties
                    heappush(frontier, [priority_score, counter, neighbor])  

        self.cost_so_far = cost_so_far
        self.came_from = came_from
        self.tsp = {}

        # If we find a candidate target
        if len(candidates) > 0:
            clusters = self.paint_clusters(candidates)
            dead_ends = self.find_dead_ends(clusters)
            self.tsp = {
                "candidates": candidates,
                "clusters": [list(c) for c in clusters],
                "dead_ends": list(dead_ends)
            }

            #We prioritize dead ends if the cost of the dead end is closer/cheaper than the closest_target + 6px of margin.
            #The idea is to visit first dead ends because it will cost much more later.
            closest_target = candidates[0]
            dead_end_target = next((candidate for candidate in candidates if candidate in dead_ends), None)
            if dead_end_target != None and cost_so_far[dead_end_target] < cost_so_far[closest_target] + MINITILE_SIZE_PX/2:
                target = dead_end_target
            else:
                target = closest_target

            path = self.reconstruct_path(came_from, pixel_paths, start, target)
            self.path = path
            return path
        
        # If we didn't find a minitile, we try go back to the start
        # If (0,0) not in came_from, it means that the robot is stuck and cannot find a path to any minitile, including the start.
        if (0,0) not in came_from: # This bug should never happen, but just in case
            print(f"ACAACA!!!")
            # utils.log_debug(f"ACAACA: came_from {came_from}, pixel_paths: {pixel_paths}")
            if use_cache == True:
                new_path = self.find_path(position, False)
                self.path = new_path
                return new_path
            else:
                self.path = []
                return []
        else:
            path = self.reconstruct_path(came_from, pixel_paths, start, (0, 0))
            self.path = path
            return path

    def count_walls_around(self, minitile):
        x, y = minitile 
        wall_count = 0
        # Counting how many walls are around the minitile, including diagonals
        cardinals = [(-1, 0), (0, -1), ( 1, 0), (0,  1)]
        diagonals = [(-1, -1), (1, -1), ( 1,  1), (-1, 1)]
        directions = cardinals + diagonals
        for dx, dy in directions:
            neighbour = (x + dx, y + dy)
            if self.map.has_walls(neighbour):
                wall_count += 1
        return wall_count

    def reconstruct_path(self, came_from, pixel_paths, start, target):
        """Turns the came_from table into a LIST of points from start to target.
        came_from should contain the path from start to any valid minitile (including target)"""
        current = target
        path = []
        
        while current != start:
            prev = came_from[current]
            pixel_path = pixel_paths[(prev, current)]
            
            for p in pixel_path[::-1]:
                path.append(self.map.pixel_to_world(p))

            current = prev

        path.reverse()
        return self.smooth_path(path)
    

    def are_colinear(self, a, b, c, margin=1e-9):
        """Checks if vectors are aligned."""
        v1x, v1y=b.x-a.x,b.y-a.y
        v2x, v2y=c.x-b.x,c.y-b.y
        dot = v1x*v2x+v1y*v2y
        mag1_sq = v1x**2 + v1y**2
        mag2_sq = v2x**2 + v2y**2
        return abs(dot**2 - mag1_sq * mag2_sq) < margin

    def smooth_path(self, path):
        """Simplifies the path lenght by removing colinear points."""
        filtered = []
        for p in path:
            while (len(filtered) >= 2) and (self.are_colinear(filtered[-2], filtered[-1], p)) and (filtered[-1].distance_to(filtered[-2]) < (MINITILE_SIZE_PX / 2)):
                filtered.pop()
            filtered.append(p)
        return filtered

    def paint_clusters(self, minitiles):
        """Agrupates a minitile cluster."""
        groups = {m: -1 for m in minitiles}
        idx = 0
        for minitile in minitiles:
            group = groups.get(minitile)
            if group != -1: continue
            
            self.flood_fill(minitile, groups, idx)
            idx += 1
        
        clusters = {}
        for minitile, group_id in groups.items():
            if group_id not in clusters:
                clusters[group_id] = set([minitile])
            else:
                clusters[group_id].add(minitile)
        return list(clusters.values())
    
    def flood_fill(self, minitile, groups, group_id):
        """Flood fill algorithm to group minitiles into clusters."""
        frontier = deque()
        frontier.append(minitile)
        came_from = {}
        came_from[minitile] = None
        cost_so_far = {minitile: 0}
        
        directions = [(-1, 0), (0, -1), ( 1, 0), (0,  1)] # Only using cardinal directions for do not selecting extra minitiles to clusters

        while len(frontier) > 0:
            current = frontier.popleft()

            if groups.get(current) == -1:
                groups[current] = group_id

            for dx, dy in directions:
                neighbor = (current[0] + dx, current[1] + dy)
                if not self.map.is_valid(neighbor):
                    continue
                is_visited, _ = self.map.is_visited(neighbor)
                if is_visited:
                    continue
                if groups.get(neighbor) != -1:
                    continue
                
                pixel_path = self.find_cached_pixel_path(current, neighbor)
                if pixel_path == None or len(pixel_path) == 0:
                    continue

                neighbor_cost = cost_so_far[current] + 1
                if neighbor not in cost_so_far:
                    cost_so_far[neighbor] = neighbor_cost
                    came_from[neighbor] = current
                    frontier.append(neighbor)
        
    def find_dead_ends(self, clusters):
        """Finds dead ends in the clusters of minitiles."""
        dead_ends = set()
        for cluster in clusters:
            if self.is_dead_end(cluster):
                for minitile in cluster:
                    dead_ends.add(minitile)
        return dead_ends

    def is_dead_end(self, cluster):
        """Checks if a cluster of minitiles is a dead end."""
        if len(cluster) > 3: 
            return False
        
        directions = [(-1, 0), (0, -1), ( 1, 0), (0,  1)]
        for minitile in cluster:
            neighbour_paths = self.pixel_paths.get(minitile) 
            for dx, dy in directions:
                """Discarding clusters where their minitiles are potential 'open map regions'"""
                neighbour = (minitile[0] + dx, minitile[1] + dy)
                if neighbour not in neighbour_paths: continue
                is_visited, _ = self.map.is_visited(neighbour)
                if is_visited: continue
                if self.map.has_walls(neighbour): continue
                if neighbour in cluster: continue
                if self.is_boring(neighbour): continue
                # If theres no minitile that follows the condition, the whole cluster is not a dead end
                return False
        return True

    def find_pixel_path(self, start, target):
        """As we are using greedy algorythm, we are now using a priority queue to explore the points.
        The first element is the distance to the target, the second is a counter to break ties in the priority queue.
        Ref: https://www.redblobgames.com/pathfinding/a-star/introduction.html#breadth-first-search"""

        start_mintile = self.map.world_to_minitile(self.map.pixel_to_world(start))
        target_minitle = self.map.world_to_minitile(self.map.pixel_to_world(target))
        if start_mintile == target_minitle:
            return []
        
        if not self.map.is_pixel_traversable(start):
            print(f"Start {start} is not traversable!")

        frontier = [] # Priority queue of points to explore
        heappush(frontier, [0.0, 0, start]) # Counter used to break ties in the priority queue
        came_from = {}
        came_from[start] = None
        cost_so_far = {start: 0}

        closest = start
        closest_dist = math.inf

        counter = 0
        path = None
        while len(frontier) > 0:
            _, _, current = heappop(frontier)

            if current == target:
                path = self.reconstruct_pixel_path(came_from, start, current)
                break
            
            # TODO(Richo): Early exit if there is no path after some distance!
            if cost_so_far[current] > MINITILE_SIZE_PX*1.42: # 1.42 is the diagonal of a minitile (sqrt(2))
                break
            
            # Looking for the closest neighbour.
            for neighbour in self.map.get_traversable_pixel_neighbours(current):
                if neighbour not in came_from:
                    dist_to_target = target.distance_to(neighbour)
                    counter += 1
                    heappush(frontier, [dist_to_target, counter, neighbour])
                    came_from[neighbour] = current
                    cost_so_far[neighbour] = cost_so_far[current] + 1

                    if dist_to_target < closest_dist:
                        closest = neighbour
                        closest_dist = dist_to_target

        if path is not None:
            # print(f"Found valid path! {path}")
            return path
        
        closest_minitile = self.map.world_to_minitile(self.map.pixel_to_world(closest))
        if closest_minitile != target_minitle:
            return []
        
        # Minimum bounding box of the target minitile, to check if the closest pixel is inside it. (1/8 of a tile is the margin)
        left = target.x - MINITILE_SIZE_PX//2
        right = target.x + MINITILE_SIZE_PX//2
        top = target.y - MINITILE_SIZE_PX//2
        bottom = target.y + MINITILE_SIZE_PX//2
        if closest.x > left and closest.x < right and closest.y > top and closest.y < bottom:
            path = self.reconstruct_pixel_path(came_from, start, closest)
            return path
        else:
            # print(f"NO path found!")
            return []       
        
    def reconstruct_pixel_path(self, came_from, start, target):
        """Turns the came_from table into a LIST of points from start to target.
        came_from should contain the path from start to any valid minitile (including target)"""
        current = target
        path = [target]
        
        while current != start:
            current = came_from[current]
            path.append(current)
        
        path.append(start) # Make sure start is also in the path
        path.reverse()
        return path
