from __future__ import annotations

from abc import ABC
from abc import abstractmethod
from controller import Supervisor
from controller import Node

from typing import TYPE_CHECKING, Type

from ConsoleLog import Console
from ErebusObject import ErebusObject

if TYPE_CHECKING:
    from MainSupervisor import Erebus
    from Robot import Robot


class Tile(ABC):
    """Abstract Tile object holding boundary data"""
    
    NAME = ""

    @abstractmethod
    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        center: tuple[float, float, float]
    ) -> None:
        """WARNING: This is an abstract class. Use `Checkpoint`, `Swamp` or
        `StartTile`

        Initialize min/max bounds of the tile, along with it's center 
        position

        Args:
            min (tuple[float, float]): Minimum x,y position
            max (tuple[float, float]): Maximum x,y position
            center (tuple[float, float, float]): Center x,y,z position
        """

        self.min: tuple[float, float] = min
        self.max: tuple[float, float] = max
        self.center: tuple[float, float, float] = center

    def check_position(self, pos: list[float]) -> bool:
        """Check if a 3D position within the bounds of this tile

        Args:
            pos (list[float]): x,y,z position

        Returns:
            bool: True if within the tile bounds, False otherwise
        """
        # If the x position is within the bounds
        if pos[0] >= self.min[0] and pos[0] <= self.max[0]:
            # if the z position is within the bounds
            if pos[2] >= self.min[1] and pos[2] <= self.max[1]:
                # It is in this checkpoint
                return True

        # It is not in this checkpoint
        return False


class Checkpoint(Tile):
    """Checkpoint Tile object holding boundary data"""

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, center)


class Swamp(Tile):
    """Swamp Tile object holding boundary data"""

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, center)

class ElementTile(Tile):

    NAME = ""

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        wb_node: Node,
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, center)
        
        self._wb_node: Node = wb_node
        self._is_active: bool = True
        
        self.wb_node_def: str = wb_node.getDef()
    
    def is_active(self):
        return self._is_active
      
    def deactivate(self):
        self._is_active = False
        #self._wb_node.getField("tileColor").setSFColor([0.635, 0.635, 0.635])

class LinkedTile(ElementTile):
    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        wb_node: Node,
        linked_wb_node: Node,
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, wb_node, center)
        
        self._linked_wb_node: Node | None = linked_wb_node
        
    def deactivate(self):
        super().deactivate()
        
        if self._linked_wb_node == None:
            return
        Console.log_debug(f"Editing linked node: {self._linked_wb_node.getDef()}")
        
        
        wall_indexes = ["right", "bottom", "top", "left"]
        
        wall_to_rmv: int = self._linked_wb_node.getField("wallToRemove").getSFInt32()
        Console.log_debug(f"Wall to remove index: {wall_to_rmv}")
        if wall_to_rmv < 0 or wall_to_rmv > 3:
            return
        wall_field: str = f"{wall_indexes[wall_to_rmv]}Wall"
        Console.log_debug(f"Removing wall: {wall_field}")
        self._linked_wb_node.getField(wall_field).setSFInt32(0)


class EndSectionTile(ElementTile):
    def __init__(
        self,
        wb_node: Node,
    ) -> None:
        super().__init__((0,0), (0,0), wb_node, (0,0,0))
        
    def deactivate(self):
        super().deactivate()
        
        wall_indexes = ["right", "bottom", "top", "left"]
        
        wall_to_rmv: int = self._wb_node.getField("wallToRemove").getSFInt32()
        Console.log_debug(f"Wall to remove index: {wall_to_rmv}")
        if wall_to_rmv < 0 or wall_to_rmv > 3:
            return
        wall_field: str = f"{wall_indexes[wall_to_rmv]}Wall"
        Console.log_debug(f"Removing wall: {wall_field}")
        self._wb_node.getField(wall_field).setSFInt32(0)
    
    def check_position(self, pos: list[float]) -> bool:
        return False

class WaterTile(ElementTile):
    """Water Tile object holding boundary data"""

    NAME = "water"

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        wb_node: Node,
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, wb_node, center)
        

class FireTile(ElementTile):
    """Fire Tile object holding boundary data"""

    NAME = "fire"

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        wb_node: Node,
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, wb_node, center)
        

class CoordinationPointPair():
    def __init__(
        self, 
        tiles: tuple[LinkedTile,LinkedTile], 
    ):
        self.tiles: tuple[LinkedTile,LinkedTile] = tiles
        
        self._completed: bool = False
        
    def check_completed_coord(self, robots: list[Robot]):
        if self._completed:
            return
        
        robot_tiles = list(self.tiles)
        
        for i in range(2):
            if robots[i].time_stopped() >= 3:
                if (self.tiles[0].check_position(robots[i].position) and
                            self.tiles[0].is_active()):
                    robot_tiles[i] = self.tiles[0]
                elif (self.tiles[1].check_position(robots[i].position) and
                            self.tiles[1].is_active()):
                    robot_tiles[i] = self.tiles[1]
                else:
                    return
            else:
                return

        if robot_tiles[0] == robot_tiles[1]:
            return

        Console.log_debug("Completed coordination challenge")
        for i in range(2):
            robots[i].increase_score("Completed coordination challenge", 5)
            self.tiles[i].deactivate()
        self._completed = True


class StartTile(Tile):
    """StartTile Tile object holding boundary data"""

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        wb_node: Node,
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, center)
        self._wb_node: Node = wb_node
        
    def set_visible(self, visible: bool) -> None:
        """Sets the visibility of the start tile

        Args:
            visible (bool): True to show green start tile color, False to
            disable
        """
        self._wb_node.getField("start").setSFBool(visible)
        
    def is_wall_present(self, wall_name: str) -> bool:
        """Returns whether a wall on a specified side is present on the start 
        tile

        Args:
            wall_name (str): Wall name to check. The valid strings for this are:
            `topWall`, `rightWall`, `bottomWall`, `leftWall`
        Returns:
            bool: True if a wall is present on the specified side, False 
            otherwise
        """
        if wall_name not in ["topWall", "rightWall", "bottomWall", "leftWall"]:
            Console.log_err(f"Invalid is_wall_present parameter: {wall_name}")
            return False
        return self._wb_node.getField(wall_name).getSFInt32() != 0

class EndTile(ElementTile):
        
    def activate(self):
        wall_indexes = ["right", "bottom", "top", "left"]
        
        wall_to_rmv: int = self._wb_node.getField("wallToRemove").getSFInt32()
        Console.log_debug(f"Wall to add index: {wall_to_rmv}")
        if wall_to_rmv < 0 or wall_to_rmv > 3:
            return
        wall_field: str = f"{wall_indexes[wall_to_rmv]}Wall"
        Console.log_debug(f"Adding wall: {wall_field}")
        self._wb_node.getField(wall_field).setSFInt32(1)
    

class StartSectionTile(Tile):
    '''EndTile object holding the boundaries'''

    def __init__(
        self,
        min: tuple[float, float],
        max: tuple[float, float],
        center: tuple[float, float, float]
    ) -> None:
        super().__init__(min, max, center)




class TileManager(ErebusObject):
    """Manages swamp and checkpoint tiles for performing checks on entry
    """
    
    # Room multipliers
    ROOM_MULT: list[float] = [1, 1.25, 1.5, 2]
    SWAMP_TIME_MULT: float = 5.0

    def __init__(self, erebus: Erebus):
        """Creates a new TileManager object. Initialises start tile, checkpoint
        and swamp objects from the Webots world.

        Args:
            erebus (Erebus): Erebus supervisor game object
        """
        super().__init__(erebus)
        self.num_swamps: int = 0
        self.num_checkpoints: int = 0

        self.start_tiles: list[StartTile] = [self._get_start_tile(0), 
                                             self._get_start_tile(1)]
        
        self.end_tiles: list[EndTile] = [self._get_end_tile(0), 
                                         self._get_end_tile(1)]
        
        self.checkpoints: list[Checkpoint] = self._get_checkpoints()
        self.swamps: list[Swamp] = self._get_swamps()
        self.section_ends = self._get_section_tiles()
        # self.water_tiles: list[WaterTile] = self._get_element_tiles(WaterTile)
        # self.fire_tiles: list[FireTile] = self._get_element_tiles(FireTile)
        # self.coord_pairs: list[CoordinationPointPair] = self._get_coord_pairs()

    def _get_swamps(self) -> list[Swamp]:
        """Get all swamps in simulation. Stores boundary information
        within a list of Swamp objects

        Returns:
            list[Swamp]: List of swamp objects
        """
        
        swamps: list[Swamp] = []

        self.num_swamps = (
            self._erebus.getFromDef('SWAMPBOUNDS')
            .getField('children')
            .getCount()
        )

        # Iterate for each swamp
        for i in range(self.num_swamps):
            # Get swamp min and max bounds positions
            min_pos: list[float] = (
                self._erebus.getFromDef(f"swamp{i}min")
                .getField("translation")
                .getSFVec3f()
            )
            max_pos: list[float] = (
                self._erebus.getFromDef(f"swamp{i}max")
                .getField("translation")
                .getSFVec3f()
            )

            center_pos: tuple = ((max_pos[0]+min_pos[0])/2,
                                 max_pos[1],
                                 (max_pos[2]+min_pos[2])/2)

            # Create a swamp object using the min and max (x,z)
            swamp: Swamp = Swamp((min_pos[0], min_pos[2]),
                                 (max_pos[0], max_pos[2]),
                                 center_pos)

            swamps.append(swamp)
            
        return swamps

    def _get_section_tiles(self) -> list[StartSectionTile]:
        # TODO could change this for swamp as well
        
        end_tiles = []

        self.num_tiles = (
            self._erebus.getFromDef('METADATA').getField('numSection').getSFInt32()
        ) 

        # Iterate for each swamp
        for i in range(self.num_tiles):
            sub_group = []
            for r in range(2):
            
                wb_node: Node = self._erebus.getFromDef(f'SECTION_TILE_{i}{r}')

                end_tile: EndSectionTile = EndSectionTile(wb_node) 
                sub_group.append(end_tile)
            end_tiles.append(sub_group)
            
            
        return end_tiles


    def _get_element_tiles(self, object: Type[ElementTile]) -> list[Type[ElementTile]]:
        # TODO could change this for swamp as well
        
        tiles = []

        self.num_tiles = (
            self._erebus.getFromDef(f'{object.NAME.upper()}BOUNDS')
            .getField('children')
            .getCount()
        )

        # Iterate for each swamp
        for i in range(self.num_tiles):
            # Get swamp min and max bounds positions
            min_pos: list[float] = (
                self._erebus.getFromDef(f"{object.NAME.lower()}{i}min")
                .getField("translation")
                .getSFVec3f()
            )
            max_pos: list[float] = (
                self._erebus.getFromDef(f"{object.NAME.lower()}{i}max")
                .getField("translation")
                .getSFVec3f()
            )

            center_pos: tuple = ((max_pos[0]+min_pos[0])/2,
                                 max_pos[1],
                                 (max_pos[2]+min_pos[2])/2)
            
            wb_node = self._erebus.getFromDef(f'{object.NAME.upper()}_TILE_{i}')

            # Create a swamp object using the min and max (x,z)
            tile: object = object((min_pos[0], min_pos[2]),
                                 (max_pos[0], max_pos[2]),
                                 wb_node,
                                 center_pos)

            tiles.append(tile)
            
        return tiles
    
    def _get_coord_pairs(self) -> list[CoordinationPointPair]:        
        pairs = []

        self.num_tiles = (
            self._erebus.getFromDef('COORDBOUNDS')
            .getField('children')
            .getCount()
        ) // 2

        # Iterate for each swamp
        for i in range(self.num_tiles):
            tiles = []
            for j in range(0, 1 + 1):
                # Get swamp min and max bounds positions
                min_pos: list[float] = (
                    self._erebus.getFromDef(f"coord{i}{j}min")
                    .getField("translation")
                    .getSFVec3f()
                )
                max_pos: list[float] = (
                    self._erebus.getFromDef(f"coord{i}{j}max")
                    .getField("translation")
                    .getSFVec3f()
                )

                center_pos: tuple = ((max_pos[0]+min_pos[0])/2,
                                    max_pos[1],
                                    (max_pos[2]+min_pos[2])/2)
                
                wb_node: Node = self._erebus.getFromDef(f'COORD_TILE_{i}{j}')
                linked_wb_node_def: str = wb_node.getField("linkedTileDef").getSFString()
                if linked_wb_node_def == "":
                    linked_wb_node = None
                else:
                    linked_wb_node: Node = self._erebus.getFromDef(
                        wb_node.getField("linkedTileDef").getSFString()
                    )

                # Create a swamp object using the min and max (x,z)
                tile: object = LinkedTile((min_pos[0], min_pos[2]),
                                            (max_pos[0], max_pos[2]),
                                            wb_node,
                                            linked_wb_node,
                                            center_pos)

                tiles.append(tile)
            
            pairs.append(CoordinationPointPair(tuple(tiles)))
        return pairs
                

    def _get_checkpoints(self) -> list[Checkpoint]:
        """Get all checkpoints in simulation. Stores boundary information
        within a list of Checkpoint objects

        Returns:
            list[Checkpoint]: List of checkpoint objects
        """

        checkpoints: list[Checkpoint] = []

        self.num_checkpoints = (
            self._erebus.getFromDef('CHECKPOINTBOUNDS')
            .getField('children')
            .getCount()
        )

        # Iterate for each checkpoint
        for i in range(self.num_checkpoints):
            # Get the checkpoint min and max bounds positions
            min_pos: list[float] = (
                self._erebus.getFromDef(f"checkpoint{i}min")
                .getField("translation")
                .getSFVec3f()
            )
            max_pos: list[float] = (
                self._erebus.getFromDef(f"checkpoint{i}max")
                .getField("translation")
                .getSFVec3f()
            )

            centerPos = ((max_pos[0]+min_pos[0])/2,
                         max_pos[1],
                         (max_pos[2]+min_pos[2])/2)

            # Create a checkpoint object using the min and max (x,z)
            checkpoint = Checkpoint((min_pos[0], min_pos[2]),
                                    (max_pos[0], max_pos[2]),
                                    centerPos)

            checkpoints.append(checkpoint)
            
        return checkpoints
            
    def _get_start_tile(self, num: int) -> StartTile:
        """Gets the world's start tile as a StartTile object, holding boundary
        information

        Returns:
            StartTile: StartTile object
        """
        start_tile_node = self._erebus.getFromDef(f"START_TILE_{num}")

        # Get the vector positions
        start_min_pos: list[float] = (
            self._erebus.getFromDef(f"start{num}min")
            .getField("translation")
            .getSFVec3f()
        )
        start_max_pos: list[float] = (
            self._erebus.getFromDef(f"start{num}max")
            .getField("translation")
            .getSFVec3f()
        )
        
        start_center_pos: tuple = ((start_max_pos[0]+start_min_pos[0])/2,
                                   start_max_pos[1],
                                   (start_max_pos[2]+start_min_pos[2])/2)

        return StartTile((start_min_pos[0], start_min_pos[2]),
                         (start_max_pos[0], start_max_pos[2]),
                         start_tile_node, 
                         center=start_center_pos,)
    
    def _get_end_tile(self, num: int):

        end_tile_node = self._erebus.getFromDef(f"END_TILE_{num}")
        
        end_min_pos: list[float] = (
            self._erebus.getFromDef(f"end{num}min")
            .getField("translation")
            .getSFVec3f()
        )
        end_max_pos: list[float] = (
            self._erebus.getFromDef(f"end{num}max")
            .getField("translation")
            .getSFVec3f()
        )
        
        end_center_pos: tuple = ((end_max_pos[0]+end_min_pos[0])/2,
                                   end_max_pos[1],
                                   (end_max_pos[2]+end_min_pos[2])/2)

        return EndTile((end_min_pos[0], end_min_pos[2]),
                         (end_max_pos[0], end_max_pos[2]),
                         end_tile_node,
                         center=end_center_pos,)

    @staticmethod
    def coord2grid(
        coord: list[float] | tuple[float, float, float],
        supervisor: Supervisor
    ) -> int:
        """Converts a world coordinate to the corresponding world tile node 
        index (only uses x,z components) 

        Args:
            coord (list[float] | tuple[float, float, float]): Webots world 
            coordinate 
            supervisor (Supervisor): Erebus supervisor object

        Returns:
            int: Index of world tile within Webots node hierarchy
        """
        # TODO assuming this is okay to just use 0?
        side: float = 0.3 * (supervisor.getFromDef("START_TILE_0")
                             .getField("xScale")
                             .getSFFloat())
        height: float = (
            supervisor.getFromDef("START_TILE_0")
            .getField("height")
            .getSFFloat()
        )
        width: float = (
            supervisor.getFromDef("START_TILE_0")
            .getField("width")
            .getSFFloat()
        )
        return int(
            round((coord[0] + (width / 2 * side)) / side, 0) * height +
            round((coord[2] + (height / 2 * side)) / side, 0)
        )
        
    def check_swamps(self, num: int) -> None: 
        """Check if the simulation robot is in any swamps. Slows down the robot
        accordingly
        """
        # Check if the robot is in a swamps
        in_swamp: bool = any([s.check_position(self._erebus.robots[num].position) 
                              for s in self.swamps])
        self._erebus.robots[num].update_in_swamp(in_swamp, 
                                               self._erebus.DEFAULT_MAX_MULT)
        
    def check_water_tiles(self, num: int) -> None: 
        # Check if the robot is in a swamps
        in_water: bool = any([s.check_position(self._erebus.robots[num].position) and
                              s.is_active()
                              for s in self.water_tiles])
        self._erebus.robots[num].update_in_water(in_water)
    
    def check_fire_tiles(self, num: int) -> None: 
        # Check if the robot is in a swamps
        in_fire: bool = any([s.check_position(self._erebus.robots[num].position) and
                             s.is_active()
                              for s in self.fire_tiles])
        self._erebus.robots[num].update_in_fire(in_fire)
    
    def check_coord_tiles(self) -> None: 
        for pair in self.coord_pairs:
            pair.check_completed_coord(self._erebus.robots)

    def check_checkpoints(self, num: int) -> None: 
        """Check if the simulation robot is in any checkpoints. Awards points
        accordingly
        """
        # Test if the robots are in checkpoints
        checkpoint = [c for c in self.checkpoints 
                      if c.check_position(self._erebus.robots[num].position)]
        # If any checkpoints
        if len(checkpoint) > 0:
            self._erebus.robots[num].update_checkpoints(checkpoint[0])

    def remove_walls(self, section_counter: int) -> None:
        for t in self.section_ends[section_counter]:
            t.deactivate()
