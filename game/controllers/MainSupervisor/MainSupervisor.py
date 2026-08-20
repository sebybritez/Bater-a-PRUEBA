# Auto install any pip modules used throughout the code base
from hashlib import new

import AutoInstall
AutoInstall._import("np", "numpy")
AutoInstall._import("cl", "termcolor")
AutoInstall._import("req", "requests")
AutoInstall._import("overrides", "overrides")
AutoInstall._import("PIL", "PIL", "pillow")

import os
import shutil
import struct
from threading import Thread
import shutil
import json
import time
import subprocess
import random
import requests as req

from controller import Supervisor
from controller import Emitter
from controller import Receiver
from controller import Node
import MapScorer
import ControllerUploader

from Tools import *
from ConsoleLog import Console
from Logger import Logger
from ProtoGenerator import generate_robot_proto
from MapAnswer import MapAnswer, pretty_print_map
from Config import Config
from Camera import *
from Tile import *
from Victim import *
from Robot import *
from Recorder import Recorder
from Test import TestRunner
from RobotWindowSender import RWSender
from ThumbnailWriter import export_map_to_img
from DockerHelper import run_docker_container

from typing import Sequence, cast

from controller.wb import wb
CANTIDADCAJAS=6
ESTRUCTURAS = "OBSTACLE"
class GameState(Enum):
    MATCH_NOT_STARTED = 1
    MATCH_RUNNING = 2
    MATCH_FINISHED = 3
    MATCH_PAUSED = 4


class Erebus(Supervisor):

    ROBOT_NAME = "Erebus_Bot"
    TIME_STEP = 16
    DEFAULT_MAX_MULT = 1.0

    def __init__(self):
        super().__init__()

        # Version info
        self._stream = 25
        self.version = "25.SUPER"

        # Start controller uploader
        uploader: Thread = Thread(target=ControllerUploader.start, daemon=True)
        uploader.start()

        # Robot window send text wrapper
        self.rws: RWSender = RWSender(self)
        
        # Get the config data from config.txt
        config_file_path = get_file_path(
            "controllers/MainSupervisor/config.txt",
            "config.txt"
        )
        self.config: Config = self._get_config(config_file_path)
        
        self.simulation_mode = self.SIMULATION_MODE_REAL_TIME
        
        # Send message to robot window to perform setup
        self.rws.send("startup")
        self._get_erebus_version()

        # Subprocess for running controllers in docker containers
        self._docker_process: Optional[subprocess.Popen] = None

        self._game_state: GameState = GameState.MATCH_NOT_STARTED
        self._last_frame: Optional[bool] = False
        self._first_frame: bool = True
        self._robot_initialised: bool = False

        # How long the game has been running for
        self.time_elapsed: float = 0.0
        self._last_time: float = -1.0
        self._real_time_elapsed: float = 0.0
        self._last_real_time: float = -1.0
        self._first_real_time: bool = True
        self._time_muliplier: float = 1.0
        # Maximum time for a match
        self.max_time: int = 8 * 60

        self._last_sent_score: float = 0.0
        self._last_sent_time: float = 0.0
        self._last_sent_real_time: float = 0.0
        
        self._num_in_swamp: int = 0

        self._section_count: int = 0

        # Get custom world data, to get max game time
        custom_world_data: list[str] = []
        if self.getCustomData() != '':
            custom_world_data = self.getCustomData().split(',')
            self.max_time = int(custom_world_data[0])

        # Max real world time is max_time + 1 min or 125% of max_time
        # which ever is greater
        self._max_real_world_time: int = int(max(self.max_time + 60,
                                                self.max_time * 1.25))

        # Init tile and victim managers
        self.tile_manager: TileManager = TileManager(self)
        self.victim_manager: VictimManager = VictimManager(self)

        cam_side: FollowSide = FollowSide.BOTTOM
        if len(custom_world_data) > 1:
            cam_side = FollowSide[custom_world_data[1].upper()]
        self._camera: Camera = Camera(self.getFromDef("Viewpoint"), cam_side)

        # Typing casts have to be used here to get proper type hints. Webots
        # returns Devices from `getDevice`, but e.g. an Emitter or Receiver
        # dont inherit from Device...
        self._receiver: Receiver = cast(Receiver, self.getDevice('receiver'))
        self._receiver.enable(Erebus.TIME_STEP)

        self.emitter: Emitter = cast(Emitter, self.getDevice('emitter'))

        # Init robots as objects to hold game data
        self.score_robot: Robot = Robot(self, -1, None)
        self.robots: list[Robot] = [Robot(self, 0, self.score_robot), Robot(self, 1, self.score_robot)]
        for i in range(len(self.robots)):
            self.robots[i] = Robot(self, i, self.score_robot)
            self.robots[i].update_config(self.config)
            self.robots[i].controller.reset_file()
            self.robots[i].reset_proto()

        # Calculate the solution arrays for the map layout
        self._map_ans = MapAnswer(self)
        map_ans: Optional[list[list]] = self._map_ans.generateAnswer()
        if map_ans is None:
            raise Exception("Critical error: Could not generate answer matrix")
        self._map_sol: list[list] = map_ans

        # Init test runner to run (unit) tests
        self._test_runner: TestRunner = TestRunner(self)
        self._run_tests: bool = False

        # Toggle for enabling remote webots controllers
        self._update_remote_enabled()

        # Export the answer map to an image used within the world selector UI
        export_map_to_img(self, self._map_sol)
        
        self.rws.send("currentWorld", self._get_current_world())

        self.rws.send("update", f"0,0,{self.max_time},0")

    def wwiReceiveText(self) -> Optional[str]:
        """
        Allow a robot controller to receive a message sent from a JavaScript
        function running in the HTML robot window

        This overrides Webot's Robot class implementation

        Returns:
            Optional[str]: Decoded robot window message
        """
        
        text: bytes = wb.wb_robot_wwi_receive_text()
        if text is None:
            return None
        else: 
            try:
                return text.decode()
            except:
                Console.log_debug(f"<wwiReceiveText> failed to decode {text}")
                pass

    def _game_init(self) -> None:
        """Initialises Erebus' initial game state. This should be run on the 
        first frame of simulation run time.
        """
        # If recording
        if self.config.recording:
            Recorder.start_recording(self)

        for index in range(len(self.robots)):
            # Get the robot node by DEF name
            robot_node: Optional[Node] = self.getFromDef(f"ROBOT{index}")
            # Add robot into world
            if robot_node == None:
                robot_node = self._add_robot(index)
            # Init robot as object to hold their info
            self.robots[index].set_node(robot_node)

            # Set robots starting position in world
            self.robots[index].set_start_pos(self.tile_manager.start_tiles[index])
            self.robots[index].set_end_pos(self.tile_manager.end_tiles[index])
            self.robots[index].in_simulation = True
            self.robots[index].set_max_velocity(self.DEFAULT_MAX_MULT)
            # Reset physics
            self.robots[index].reset_physics()

        if self.config.recording:
            Recorder.reset_countdown(self)
            
        # Enqueue warning if debug mode is on when game the starts
        if Console.DEBUG_MODE:
            self.robots[0].history.enqueue("WARNING: Debug mode is on. This "
                                           "should not be on during competitions.")
            self.robots[1].history.enqueue("WARNING: Debug mode is on. This "
                                           "should not be on during competitions.")
        
        # Inicializar posiciones de obstáculos al comenzar
        self.positionMisCajas = []
        self.obstacles = []
        self.destroyed_obstacles = set()  # Rastrear cajas destruidas
        self.obstacles_destroyed_by_robot = [0, 0]  # Contador por robot
        for i in range(CANTIDADCAJAS):
            obs = self.getFromDef(f"{ESTRUCTURAS}{i}")
            if obs:
                self.obstacles.append(obs)
                position = obs.getPosition()
                self.positionMisCajas.append(position)
        
        # Asignar colores aleatorios a las cajas
        self._randomize_box_colors()
        
        self._last_time = self.getTime()
        self._first_frame = False
        self._robot_initialised = True
        self._last_real_time = time.time()

    def _get_obstacle_color(self, obstacle: Node) -> tuple[float, float, float]:
        """Obtiene el color RGB de un obstáculo desde su material
        
        Args:
            obstacle (Node): Nodo del obstáculo
            
        Returns:
            tuple[float, float, float]: Color en formato RGB (valores entre 0 y 1)
        """
        try:
            # Acceder a la estructura: Solid -> children[0] -> Shape -> appearance -> material
            shape = obstacle.getField("children").getMFNode(0)
            if shape is None:
                return (0.45, 0.45, 0.45)  # Color por defecto (gris)
            
            appearance = shape.getField("appearance").getSFNode()
            material = appearance.getField("material").getSFNode()
            color = material.getField("diffuseColor").getSFColor()
            
            return tuple(color)
        except Exception as e:
            print(f"Error al obtener color del obstáculo: {e}")
            return (0.45, 0.45, 0.45)  # Color por defecto
    
    def _set_obstacle_color(self, obstacle_index: int, color: tuple[float, float, float]) -> None:
        """Cambia el color de un obstáculo
        
        Args:
            obstacle_index (int): Índice del obstáculo (0-5)
            color (tuple): Color en RGB (valores entre 0 y 1)
        """
        if obstacle_index >= len(self.obstacles) or self.obstacles[obstacle_index] is None:
            print(f"Obstáculo #{obstacle_index} no existe o ya fue destruido")
            return
        
        try:
            obstacle = self.obstacles[obstacle_index]
            shape = obstacle.getField("children").getMFNode(0)
            appearance = shape.getField("appearance").getSFNode()
            material = appearance.getField("material").getSFNode()
            material.getField("diffuseColor").setSFColor(list(color))
            print(f"Color del obstáculo #{obstacle_index} cambiado a RGB{color}")
        except Exception as e:
            print(f"Error al cambiar color del obstáculo #{obstacle_index}: {e}")
    
    def _randomize_box_colors(self) -> None:
        """Asigna colores aleatorios a las cajas: EXACTAMENTE 3 azules y 3 rojos
        
        Crea una lista con 3 azules y 3 rojos, la mezcla, y la asigna
        a las cajas para garantizar una distribución equilibrada.
        """
        if not hasattr(self, 'obstacles') or len(self.obstacles) == 0:
            print("ERROR: No hay obstáculos inicializados")
            return
        
        print("\n" + "="*70)
        print("ASIGNANDO COLORES A LAS CAJAS (3 AZULES + 3 ROJOS)".center(70))
        print("="*70)
        
        color_azul = (0.0, 0.0, 1.0)
        color_rojo = (1.0, 0.0, 0.0)
        
        # Crear lista: 3 azules y 3 rojos
        colores = [color_azul] * 3 + [color_rojo] * 3
        
        # Mezclar aleatoriamente
        random.shuffle(colores)
        
        # Asignar a las cajas
        for idx, color in enumerate(colores):
            if self.obstacles[idx] is not None:
                color_nombre = "AZUL" if color == color_azul else "ROJO"
                self._set_obstacle_color(idx, color)
                print(f"  Caja #{idx} → {color_nombre} {color}")
        
        print("="*70 + "\n")
    
    def _is_box_blue(self, color: tuple[float, float, float]) -> bool:
        """Determina si un color corresponde a azul puro
        
        Args:
            color (tuple): Color en formato RGB
            
        Returns:
            bool: True si es azul (0, 0, 1), False en caso contrario
        """
        r, g, b = color
        return b > 0.9 and r < 0.1 and g < 0.1
    
    def _is_box_red(self, color: tuple[float, float, float]) -> bool:
        """Determina si un color corresponde a rojo puro
        
        Args:
            color (tuple): Color en formato RGB
            
        Returns:
            bool: True si es rojo (1, 0, 0), False en caso contrario
        """
        r, g, b = color
        return r > 0.9 and g < 0.1 and b < 0.1
    
    def _calculate_box_score(self, robot_index: int, box_color: tuple[float, float, float]) -> tuple[int, str]:
        """Calcula los puntos asignados según el robot y el color de la caja
        
        Args:
            robot_index (int): Índice del robot (0 = azul, 1 = rojo)
            box_color (tuple): Color RGB de la caja
            
        Returns:
            tuple[int, str]: (puntos a asignar, razón/descripción)
        """
        is_blue = self._is_box_blue(box_color)
        is_red = self._is_box_red(box_color)
        
        if robot_index == 0:  # Robot azul
            if is_blue:
                return (20, "Color coincide")
            else:
                return (10, "No coincide el color")
        else:  # robot_index == 1, Robot rojo
            if is_red:
                return (20, "Color coincide")
            else:
                return (10, "No coincide el color")

    def _check_obstacle_collisions(self, robot_index: int) -> None:
        """Check if un obstáculo se ha movido y eliminarlo, sumando puntos al robot que lo está tocando
            Sistema cooperativo: Robot azul (0) y rojo (1) trabajan juntos.
            - Caja azul: 20 pts para robot azul, 10 pts para robot rojo
            - Caja roja: 20 pts para robot rojo, 10 pts para robot azul
            - Otras cajas: 10 puntos
        Args:
            robot_index (int): El índice del robot a verificar colisiones
        """
        if not self.robots[robot_index].in_simulation:
            return
        
        # Si no se han inicializado los obstáculos, salir
        if not hasattr(self, 'obstacles') or not hasattr(self, 'positionMisCajas'):
            return
        
        # Tolerancia para detectar movimiento real (ignorar cambios por física)
        MOVEMENT_THRESHOLD = 0.0001 
        
        # Comparar posición actual con posición inicial de cada obstáculo
        for idx, obs in enumerate(self.obstacles):
            # Si ya fue destruido, saltar
            if obs is None or idx in self.destroyed_obstacles:
                continue
            
            actualPosition = obs.getPosition()
            initialPosition = self.positionMisCajas[idx]
            
            # Calcular distancia en X y Z (ignorar Y porque la física puede afectarlo)
            distance_x = abs(actualPosition[0] - initialPosition[0])
            distance_z = abs(actualPosition[2] - initialPosition[2])
            
            # Verificar si el obstáculo se ha movido significativamente
            if distance_x > MOVEMENT_THRESHOLD or distance_z > MOVEMENT_THRESHOLD:
                # Calcular distancia del robot actual a la caja
                robot_pos = self.robots[robot_index].position
                robot_to_box_dist = ((robot_pos[0] - actualPosition[0])**2 + 
                                     (robot_pos[2] - actualPosition[2])**2)**0.5
                
                #print(f"DEBUG: Caja #{idx} movida. Robot {robot_index} está a {robot_to_box_dist:.4f}m")
                
                # Calcular distancia del otro robot a la caja
                other_robot_idx = 1 - robot_index
                if self.robots[other_robot_idx].in_simulation:
                    other_pos = self.robots[other_robot_idx].position
                    other_to_box_dist = ((other_pos[0] - actualPosition[0])**2 + 
                                        (other_pos[2] - actualPosition[2])**2)**0.5
                    #print(f"DEBUG: Robot {other_robot_idx} está a {other_to_box_dist:.4f}m")
                    
                    # El robot más cercano obtiene el punto
                    if other_to_box_dist < robot_to_box_dist:
                        actual_destroyer = other_robot_idx
                        closest_dist = other_to_box_dist
                    else:
                        actual_destroyer = robot_index
                        closest_dist = robot_to_box_dist
                else:
                    actual_destroyer = robot_index
                    closest_dist = robot_to_box_dist
                
                try:
                    robot_name = "robot0Controller" if actual_destroyer == 0 else "robot1Controller"
                    robot_color = "AZUL" if actual_destroyer == 0 else "ROJO"
                    print(f"\n>>> CAJA #{idx} DESTRUIDA POR ROBOT {actual_destroyer} ({robot_name} - {robot_color}) <<<")
                    
                    # Obtener el color de la caja
                    box_color = self._get_obstacle_color(obs)
                    #print(f"Color de la caja: RGB{box_color}")
                    
                    # Calcular puntos según el robot y el color
                    points, reason = self._calculate_box_score(actual_destroyer, box_color)
                    #print(f"Puntos asignados: {points} ({reason})")
                    
                    # Sumar puntos al robot
                    if points > 0:
                        self.robots[actual_destroyer].increase_score(f"Caja #{idx} eliminada - {reason}", points)
                        print(f"Puntos totales: {self.score_robot.get_score()}")
                    
                    # Eliminar obstáculo
                    obs.remove()
                    self.obstacles[idx] = None
                    self.destroyed_obstacles.add(idx)
                    
                    
                    self.print_scoreboard()
                except Exception as e:
                    print(f"Error al procesar destrucción de caja #{idx}: {e}")

    def print_scoreboard(self) -> None:
        """Imprime un tablero de puntuación con estadísticas de cajas destruidas"""
        print(f"Cajas destruidas: {len(self.destroyed_obstacles)}/{CANTIDADCAJAS}")
        print("\n  Cajas destruidas (ID):")
        if self.destroyed_obstacles:
            destroyed_list = sorted(list(self.destroyed_obstacles))
            print(f"    {destroyed_list}")
        else:
            print(f"    Ninguna aún")
    

    def relocate_robot(self, num: int, manual = False) -> None:
        """Relocate robot to last visited checkpoint

        Args:
            manual (bool, optional): Whether the robot relocate is manual (from
            the UI) or not (via robot packet info). Defaults to False.
        """
        # if not self.robots[num].in_simulation:
        #     Console.log_debug("Robot trying to relocate after pseudo-exit")
        #     return
        if self.robots[num].last_visited_checkpoint_pos is None:
            Console.log_err("Last visited checkpoint was None.")
            return

        # Get last checkpoint visited
        relocate_position: tuple = self.robots[num].last_visited_checkpoint_pos

        # Set position and rotation of robot
        self.robots[num].position = [relocate_position[0],
                                   -0.03,
                                   relocate_position[2]]
        self.robots[num].rotation = [0, 1, 0, 0]

        # Reset physics
        self.robots[num].reset_physics()
        # Notify robot
        self.emitter.send(struct.pack("c", bytes("L", "utf-8")))
        
        # Suffix for event history to log what causes a relocate
        suffix = "(via robot)"
        if manual:
            suffix = "(via UI)"
        
        # Update history with event
        self.robots[num].increase_score(f"Lack of Progress {suffix}", -5)

        # Update the camera position since the robot has now suddenly moved
        if self.config.automatic_camera and self._camera.wb_viewpoint_node:
            self._camera.set_view_point(self.robots[num])

    def _robot_quit(self, num: int, time_up: bool) -> None:
        """Quit robot from simulation

        Args:
            time_up (bool): Whether the cause of the robot quit is due to the 
            timer running out 
        """
        # Quit robot if present
        if self.robots[num].in_simulation:
            self.robots[num].position = [self.robots[num].end_tile.center[0], self.robots[num].position[1],
                                             self.robots[num].end_tile.center[2]]
            self.robots[num].end_tile.activate()
            # Remove webots node
            # self.robots[num].remove_node()
            self.robots[num].in_simulation = False
            # Send message to robot window to update quit button
            self.rws.send(f"robotNotInSimulation{num}")
            # Update history event whether its manual or via exit message
            if not time_up:
                self.robots[num].history.enqueue("Successful Exit")


    def _add_physicsless_robot_proto(self) -> None:
        """Copies a physics-less version of the default robot proto to 
        the custom_robot.proto proto location, forcing the use of this proto
        instead of any other that may have been loaded
        
        Note: This should really only be used for automated testing
        """

        # Copy physicsless proto file to be used as custom robot proto
        path: str = get_file_path("proto_defaults/E-puck-custom-default-FLU-physicsless.proto",
                                  "../../proto_defaults/E-puck-custom-default-FLU-physicsless.proto")
        dest: str = get_file_path("protos/custom_robot.proto",
                                  "../../protos/custom_robot.proto")
        shutil.copyfile(path, dest)

    def _add_robot(self, num: int) -> Node:
        """Add a robot Node to the root of the Webots scene tree.
        
        Sets the robot's controller to either point to the robot0Controller 
        file, or if the remote enabled setting is set, sets the robot to take
        "extern" controllers

        Returns:
            Node: Node reference to newly added robot
        """
        
        if self._run_tests:
            self._add_physicsless_robot_proto()

        controller: str = f"robot{num}Controller"
        if self.robots[num].remote_enabled:
            controller = "<extern>"

        # Get webots root
        root: Node = self.getRoot()
        root_children_field: Field = root.getField('children')

        node_string: str = f"""DEF ROBOT{num} custom_robot_{num} {{ 
                                    translation 1000 1000 1000 
                                    rotation 0 1 0 0 
                                    name "{Erebus.ROBOT_NAME}_{num}"
                                    controller "{controller}"
                                    camera_fieldOfView 1 
                                    camera_width 64 
                                    camera_height 40 
                                }}
                            """

        # Get robot to insert into world
        root_children_field.importMFNodeFromString(-1, node_string)
        # Update robot window to say robot is in simulation
        self.rws.send(f"robotInSimulation{num}")
        # Return the robot node
        return self.getFromDef(f"ROBOT{num}")

    def _process_robot_json(self, num: int, json_data: str) -> None:
        """Process custom robot json data to generate a new robot proto file.
        
        The custom robot proto file is used when importing the robot at game 
        start. Also detects if the robot has a Battery component and configures
        the supervisor's battery tracking accordingly.
        """
        robot_json: dict = json.loads(json_data)
        if generate_robot_proto(num, robot_json):
            self.rws.send(f"jsonLoaded,{num}")

            # ── Detectar componente Battery en el JSON ────────────────────────
            self.robots[num].battery.deactivate()  # reset por si se recarga
            for comp_val in robot_json.values():
                if comp_val.get("name") == "Battery":
                    max_energy = comp_val.get("maxEnergy", 100)
                    self.robots[num].battery.configure(max_energy=max_energy)
                    break

    def wait(self, sec: float) -> None:
        """Waits for x amount of seconds, while still stepping the Webots
        simulation to avoid simulation pauses

        Args:
            sec (float): Seconds to wait
        """
        first: float = self.getTime()
        while True:
            self.step(Erebus.TIME_STEP)
            if self.getTime() - first > sec:
                break
    
    def set_time_multiplier(self, multiplier: float) -> None:
        """Set time multiplier for game countdown timer

        Args:
            multiplier (float): Countdown time multiplier
        """
        if multiplier == Erebus.DEFAULT_MAX_MULT:
            self._num_in_swamp -= 1
            if self._num_in_swamp == 0:
                self._time_muliplier = Erebus.DEFAULT_MAX_MULT
        else:
            self._num_in_swamp += 1
            self._time_muliplier = multiplier
        Console.log_debug(f"# in swamp: {self._num_in_swamp}")
        Console.log_debug(f"Updating time multiplier: {self._time_muliplier}x")
            
    def _get_current_world(self) -> str:
        """Gets the current world name, with no file extension

        Returns:
            str: Current world name
        """
        return os.path.basename(self.getWorldPath())[:-4]

    def _get_worlds(self) -> str:
        """Gets all worlds from the `worlds` directory as a list of file names,
        separated by commas. File extensions are stripped and hidden files
        are ignored.

        Returns:
            str: List of worlds as a string. Example: `"world1,world2,room4"`
        """
        path: str = get_file_path("worlds", "../../worlds")
        files: list[str] = [file for file in os.listdir(path)
                            if file[-3:] == 'wbt' and file[0] != '.']
        return ','.join(files)

    def _load_world(self, world: str) -> None:
        """Loads a specified Webots world world file located in the worlds 
        directory. This will close the current running world.

        Args:
            world (str): World file name within the worlds directory
        """
        path: str = get_file_path("worlds", "../../worlds")
        path = os.path.join(path, world)
        self.worldLoad(path)

    def _load_test_script(self) -> None:
        """Loads the test controller script, used to run Erebus (unit) tests,
        as the robot0Controller. This effectively achieves what the load
        controller UI does, but directly.
        """
        path: str = get_file_path("controllers/MainSupervisor/tests.py",
                                  "tests.py")
        dest: str = get_file_path("controllers/robot0Controller/robot0Controller.py",
                                  "../robot0Controller/robot0Controller.py")
        shutil.copyfile(path, dest)

    def _get_erebus_version(self) -> None:
        """Updates the Erebus web UI with the version of the platform. Extra
        data is sent to specify if the version if up to date, or needs updating.
        """
        try:
            self.rws.send("version", f"{self.version}")
            # Check updates
            url = "https://gitlab.com/api/v4/projects/22054848/releases"
            response = req.get(url)
            releases = response.json()
            releases = list(filter(lambda release: release['tag_name'].startswith(
                f"v{self._stream}"), releases))
            if len(releases) > 0:
                if releases[0]['tag_name'].replace('_', ' ') == f'v{self.version}':
                    self.rws.send("latest", f"{self.version}")
                elif any([r['tag_name'].replace('_', ' ') == f'v{self.version}' for r in releases]):
                    self.rws.send(
                        "outdated", f"{self.version},{releases[0]['tag_name'].replace('v','').replace('_', ' ')}")
                else:
                    self.rws.send("unreleased", f"{self.version}")
            else:
                self.rws.send("version", f"{self.version}")
        except:
            self.rws.send("version", f"{self.version}")

    def _detect_victim(self, robot_message: list[Any]) -> None:
        """Runs victim detection to give points based on the victim's estimated
        type and location

        Args:
            robot_message (list[Any]): The competitor's robot message data
        """
        # NOW OF TYPE::: i i c i
        
        # Get estimated position and type values
        est_vic_pos = robot_message[0]
        est_vic_type = robot_message[1]
        num = int(robot_message[2]) # Get robot number

        iterator: Sequence[VictimObject] = self.victim_manager.victims
        name: str = 'Victim'
        correct_type_bonus: int = 10
        misidentification: bool = True

        if est_vic_type.lower() in list(map(to_lower, HazardMap.HAZARD_TYPES)):
            iterator = self.victim_manager.hazards
            name = 'Hazard'

        # Get nearby victim/hazards that are within range (as per the rules)
        nearby_map_issues: Sequence[VictimObject] = [
            h for h in iterator
            if h.check_position(self.robots[num].position) and
            h.check_position(est_vic_pos) and
            h.on_same_side(self.robots[num]) and
            not h.identified
        ]

        Console.log_debug(f"--- Victim Data ---")
        for h in iterator:
            Console.log_debug("===")
            Console.log_debug(
                f"Position {self.robots[num].position}")
            Console.log_debug(
                f"Distance {h.get_distance(self.robots[num].position)}/0.09")
            Console.log_debug(
                f"In range: ({h.check_position(self.robots[num].position)})")
            Console.log_debug(f"Est pos: {est_vic_pos}")
            Console.log_debug(
                f"Est distance {h.get_distance(est_vic_pos)}/0.09")
            Console.log_debug(
                f"Est distance in range: {h.check_position(est_vic_pos)}")
            Console.log_debug(
                f"On same side: {h.on_same_side(self.robots[num])}")
            Console.log_debug(f"Identified: {h.identified}")
            Console.log_debug("===")
        Console.log_debug(f"Nearby issues: {len(nearby_map_issues)}")
        Console.log_debug(f"--- ----------- ---")

        # Award points based on correct victim identifications etc.
        if len(nearby_map_issues) > 0:

            # TODO should it take the nearest, or perhaps also account
            # for which victim type was trying to be identified?

            # Take the nearest map issue by distance to the estimated coordinate
            distances: list[float] = [h.get_distance(est_vic_pos)
                                      for h in nearby_map_issues]

            nearby_issue: VictimObject = nearby_map_issues[np.argmin(
                distances)]

            # Get points scored depending on the type of victim
            grid: int = self.tile_manager.coord2grid(
                nearby_issue.wb_translation_field.getSFVec3f(),
                self)

            Console.log_debug(f"Victim type est. {est_vic_type.lower()} vs "
                              f"{nearby_issue.simple_victim_type.lower()}")

            # Update score and history for victim
            if est_vic_type.lower() == nearby_issue.simple_victim_type.lower() and name == "Victim":
                self.robots[num].increase_score(
                    f"Found victim: {nearby_issue.simple_victim_type.upper()}",
                    correct_type_bonus,
                    2**self._section_count,
                )
                misidentification: bool = False
                self.robots[num].victim_identified = True
                nearby_issue.identified = True
                if self.robots[num].update_detected_victims(nearby_issue, self.robots[int(not bool(num))]):
                    self.robots[0].reset_victims_counters()
                    self.robots[1].reset_victims_counters()
                    self.tile_manager.remove_walls(self._section_count)
                    self._section_count += 1

        if misidentification:
            self.robots[num].increase_score(f"Misidentification of {name}",
                                          -5)

    def _process_message(self, robot_message: list[Any]) -> None:
        """Processes the messages recieved from the competitor's robot's emitter
        as specified in the simulation rules

        Args:
            robot_message (list[Any]): The competitor's robot message data 
        """
        Console.log_debug(
            f"Robot 0 Stopped for {self.robots[0].time_stopped()}s")
        Console.log_debug(
            f"Robot 1 Stopped for {self.robots[1].time_stopped()}s")
        
        if robot_message[0] == 'L':
            if len(robot_message) > 0 and self.robots[robot_message[1]].in_simulation:
                self.relocate_robot(robot_message[1])
                self.robots[robot_message[1]].reset_time_stopped()
        # Process game info commands
        elif robot_message[0] == 'G':
            if self.robots[robot_message[1]].in_simulation:
                # Send game info in format:
                # (G, score, game time left, real time left)
                self.emitter.send(
                    struct.pack(
                        "c f i i",
                        bytes("G", "utf-8"),
                        round(self.score_robot.get_score(), 2),
                        self.max_time - int(self.time_elapsed),
                        self._max_real_world_time - int(self._real_time_elapsed)
                    )
                )

        # If robot stopped for 1 second, run victim detection 
        # TODO since messages to supervisor are stateless, either robot can technically send the victim ack message
        elif len(robot_message) == 3:
            if self.robots[int(robot_message[2])].time_stopped() >= 1.0 and self.robots[int(robot_message[2])].in_simulation:
                self._detect_victim(robot_message)

    def _process_rw_message(self, message: str) -> None:
        """Processes messages received from the MainSupervisor's robot window

        Args:
            message (str): Message to process
        """
        
        # Split the message to get extra command arguments if needed 
        parts: list[str] = message.split(",")

        if len(parts) > 0:
            command: str = parts[0]
            self.rws.update_received_history(command, str(parts[1:]))

            # Start running the match
            if command == "run":
                self._game_state = GameState.MATCH_RUNNING
                self.rws.update_history("runPressed")
                
            # Run tests
            if command == 'runTest':
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self._game_state = GameState.MATCH_RUNNING
                    self._run_tests = True
                    self.config.disable_lop = True
                    self.simulation_mode = self.SIMULATION_MODE_FAST

            # Pause the match
            if command == "pause":
                self._game_state = GameState.MATCH_PAUSED
                self.rws.update_history("pausedPressed")

            # Reset the simulation (reload the world)
            if command == "reset":
                self._robot_quit(0, False)
                self._robot_quit(1, False)
                self.victim_manager.reset_victim_textures()

                self.simulationReset()
                self._game_state = GameState.MATCH_FINISHED

                # Show start tile
                self.tile_manager.start_tiles[0].set_visible(True)
                self.tile_manager.start_tiles[1].set_visible(True)

                # Must restart world - to reload to .wbo file for the robot
                # which only seems to be read and interpreted once per game, so
                # if we load a new robot file, the new changes won't come into
                # place until the world is reset!
                self.worldReload()

            # Unload the robot controller
            if command == "robot0Unload":
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.robots[0].controller.reset()
                    
            if command == "robot1Unload":
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.robots[1].controller.reset()

            # Unload the custom robot json
            if command == "jsonUnload":
                data = message.split(",", 1)
                if len(data) > 1:
                    # Remove the robot proto
                    if self._game_state == GameState.MATCH_NOT_STARTED:
                        self.robots[int(data[1])].reset_proto(True)

            # Relocate the robot
            if command == 'relocate':
                data = message.split(",", 1)
                if len(data) > 1:
                    self.relocate_robot(int(data[1]), manual=True)

            # Quite the robot from the simulation
            if command == 'quit':
                data = message.split(",", 1)
                if len(data) > 1:
                    if int(data[1]) == 0:
                        if self._game_state == GameState.MATCH_RUNNING:
                            self.robots[0].history.enqueue("Manual give up!")
                            self.robots[1].history.enqueue("Manual give up!")
                            self._robot_quit(0, True)
                            self._robot_quit(1, True)
                            self._game_state = GameState.MATCH_FINISHED
                            self._last_frame = True
                            self.rws.send("ended")

            # If custom robot json is loaded
            if command == 'robotJson':
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    data = message.split(",", 2)
                    if len(data) > 1:
                        self._process_robot_json(int(data[1]), data[2])

            # If config is updated
            if command == 'config':
                configData = message.split(",")[1:]
                self.config = Config(configData, self.config.path)
                self.robots[0].update_config(self.config)
                self.robots[1].update_config(self.config)
                
                # Enqueue warning when config is updated when the game is running
                if self._game_state == GameState.MATCH_RUNNING:
                    self.robots[0].history.enqueue("WARNING: Erebus config updated")
                    self.robots[1].history.enqueue("WARNING: Erebus config updated")

                with open(self.config.path, 'w') as f:
                    f.write(','.join(message.split(",")[1:]))
            
            # Load a specific world file
            if command == 'loadWorld':
                self._load_world(parts[1])

            # Load test controller
            if command == 'loadTest':
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self._load_test_script()

            # The robot window was reloaded, commands must be re-sent to
            # achieve the same state it was previously in
            if command == 'rw_reload':
                self.rws.send_all()
                config_file_path = get_file_path(
                    "controllers/MainSupervisor/config.txt",
                    "config.txt"
                )
                self.config = self._get_config(config_file_path)

            # Robot controller ui button was pressed 
            if command == 'loadControllerPressed':
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.rws.update_history("loadControllerPressed,", parts[1])

            # Robot unload controller ui button was pressed
            if command == 'unloadControllerPressed':
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.rws.update_history("unloadControllerPressed,", parts[1])

            # Enable remote controller
            if command == 'remoteEnable':
                data = message.split(",", 1)
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.robots[int(data[1])].remote_enabled = True
                    self.rws.update_history("remoteEnabled", data[1])

            # Disable remote controller
            if command == 'remoteDisable':
                data = message.split(",", 1)
                if self._game_state == GameState.MATCH_NOT_STARTED:
                    self.robots[int(data[1])].remote_enabled = False
                    self.rws.update_history("remoteDisabled", data[1])

            # Send the list of Erebus worlds
            if command == 'getWorlds':
                self.rws.send('worlds', f'{str(self._get_worlds())}')

    def _get_config(self, config_file_path: str) -> Config:
        """Processes the simulation's `config.txt` csv file to create
        a config object to use during game runtime

        Args:
            config_file_path (str): Config file path location

        Returns:
            Config: Config data object
        """

        with open(config_file_path, 'r') as f:
            configData = f.read().replace('\\', '/').split(',')

        self.rws.send("config", ','.join(configData))

        return Config(configData, config_file_path)

    def _update_remote_enabled(self) -> None:
        """Updates internal variables to align with the settings from the
        current `Config` data (`self.config`).

        Updates the web UI's remote enabled button enable state to reflect the
        current state of the config setting 
        """
        for index in range(len(self.robots)):
            self.robots[index].remote_enabled = self.config.keep_remote
            if self.robots[index].remote_enabled:
                self.rws.update_history(f"remoteEnabled,{index}")
            else:
                self.rws.update_history(f"remoteDisabled,{index}")

    def update(self) -> None:
        """Main Erebus update loop, used to process anything needed during the
        runtime of the simulation
        """

        # If last frame
        if self._last_frame == True:
            self._last_frame = None
            self._game_state = GameState.MATCH_FINISHED
            if self.config.recording:
                Recorder.stop_recording(self)

        # The first frame of the game running only
        if self._first_frame and self._game_state == GameState.MATCH_RUNNING:
            self._game_init()

        if self._run_tests:
            self._test_runner.run()

        for r_index in range(len(self.robots)):
            # Main game loop
            if self.robots[r_index].in_simulation:
                self.robots[r_index].update_time_elapsed(self.time_elapsed)

                self.tile_manager.check_checkpoints(r_index)
                self.tile_manager.check_swamps(r_index)
                # self.tile_manager.check_section_start(r_index)
                
                # Check for obstacle collisions
                self._check_obstacle_collisions(r_index)

                # If receiver has got a message
                if self._receiver.getQueueLength() > 0:
                    # Get receiver data
                    received_data = self._receiver.getBytes()
                    
                    # Process robot messages
                    test_msg = False
                    if self._run_tests:
                        test_msg = self._test_runner.get_stage(received_data)
                        self._receiver.nextPacket()
                    if not test_msg:
                        self.robots[r_index].set_message(received_data)
                        self._receiver.nextPacket()

                    # If data received from competitor's robot
                    if self.robots[r_index].message != []:
                        robot_message: list[Any] = self.robots[r_index].message
                        Console.log_debug(f"Robot Message: {robot_message}")
                        self.robots[r_index].message = []
                        self._process_message(robot_message)

                if self._game_state == GameState.MATCH_RUNNING:
                    # Relocate robot if stationary for 20 sec
                    # IN SUPERTEAMS NOT USED

                    
                    # if self.robots[r_index].time_stopped() >= 20:
                    #     if not self.config.disable_lop:
                    #         self.relocate_robot(r_index)
                    #     self.robots[r_index].reset_time_stopped()
                        
                    # Relocate robot if fallen in black hole
                    if (self.robots[r_index].position[1] < -0.035 and
                            self._game_state == GameState.MATCH_RUNNING):
                        if not self.config.disable_lop:
                            self.relocate_robot(r_index)
                        self.robots[r_index].reset_time_stopped()

                if (self.robots[r_index].in_simulation and 
                    self.robots[r_index].end_tile.check_position(self.robots[r_index].position)):
                        self.robots[r_index].increase_score("Exit found!", 100)
                        self._robot_quit(r_index, False)


                    

        if self._robot_initialised:
            # Send the update information to the robot window, the current
            # simulation time and score etc.
            current_score: float = self.score_robot.get_score()

            self.time_elapsed = min(self.time_elapsed, self.max_time)
            self._real_time_elapsed = min(self._real_time_elapsed,
                                         self._max_real_world_time)

            if (self._last_sent_score != current_score or
                self._last_sent_time != int(self.time_elapsed) or
                    self._last_sent_real_time != int(self._real_time_elapsed)):

                self.rws.send("update", f"{round(current_score, 2)},"
                                        f"{int(self.time_elapsed)},"
                                        f"{self.max_time},"
                                        f"{int(self._real_time_elapsed)}")

                self._last_sent_score = current_score
                self._last_sent_time = int(self.time_elapsed)
                self._last_sent_real_time = int(self._real_time_elapsed)
                if self.config.recording:
                    Recorder.update(self)

            both_exited = all(not r.in_simulation for r in self.robots)
            
            # If the time is up
            if ((self.time_elapsed >= self.max_time or
                 self._real_time_elapsed >= self._max_real_world_time or
                 both_exited) and
                    self._last_frame != None):
                
                for r_idx in range(len(self.robots)):
                    if not self.robots[r_idx].in_simulation:
                        self._robot_quit(r_idx, True)

                self._game_state = GameState.MATCH_FINISHED
                self._last_frame = True

                self.rws.send("ended")

        # Get the message in from the robot window(if there is one)
        message: Optional[str] = self.wwiReceiveText()
        while message not in ['', None]:
            Console.log_debug(f"Received wwi message: {message}")
            self._process_rw_message(message)  # type: ignore
            message = self.wwiReceiveText()

        if self._game_state == GameState.MATCH_PAUSED:
            self.step(0)
            # Sleep the script every loop while paused to do "busy work", so
            # the loop doesn't use unnecessary amounts of CPU time
            time.sleep(0.01)
            self._last_real_time = time.time()

        # If the match is running
        if self._robot_initialised and self._game_state == GameState.MATCH_RUNNING:
            # If waiting for a remote controller, don't count time waiting
            if ((self.robots[0].remote_enabled or self.robots[1].remote_enabled) 
                and self._first_real_time and self._last_time != self.getTime()):
                self._last_real_time = time.time()
                self._first_real_time = False
            # Get real world time (for 9 min real world time elapsed rule)
            self._real_time_elapsed += (time.time() - self._last_real_time)
            self._last_real_time = time.time()
            # Get the time since the last frame
            frameTime = self.getTime() - self._last_time
            # Scale frame time by countdown time multiplier (used for swamps)
            frameTime *= self._time_muliplier
            # Add to the elapsed time
            self.time_elapsed += frameTime
            # Get the current time
            self._last_time = self.getTime()
            # Step the simulation on
            step = self.step(Erebus.TIME_STEP)
            # If the simulation is terminated or the time is up
            if step == -1:
                # Stop simulating
                self._game_state = GameState.MATCH_FINISHED

        elif (self._first_frame or
              self._last_frame == True or
              self._game_state == GameState.MATCH_FINISHED):
            # Step simulation
            self.step(Erebus.TIME_STEP)


if __name__ == '__main__':

    erebus: Erebus = Erebus()

    while True:  # Main loop
        try:
            erebus.update()
        except Exception as e:
            Console.log_err(f"Caught MainSupervisor main thread error: {e}")
