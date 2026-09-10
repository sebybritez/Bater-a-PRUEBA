from controller import Robot, Motor, DistanceSensor, TouchSensor, Camera, Emitter, Receiver
import time
import struct

class RobotRL:
    """
    Clase simplificada RobotRL para facilitar la programacion educativa de robots en Webots / Erebus.
    Utiliza directamente los nombres de hardware del robot 'Erebus_Bot' (custom_robot.proto)
    para evitar advertencias o errores en la consola de Webots.
    Sincroniza automaticamente la velocidad de los motores y la telemetria (bateria, tiempo y puntaje).
    Todo el codigo y variables siguen la notacion camelCase.
    """

    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())
        if self.timeStep <= 0:
            self.timeStep = 32

        # 1. Motores (wheel1 motor = Izquierda, wheel2 motor = Derecha)
        self.leftMotor = None
        self.rightMotor = None

        motorNamesLeft = ["wheel1 motor", "left wheel motor", "motor_left"]
        motorNamesRight = ["wheel2 motor", "right wheel motor", "motor_right"]

        for name in motorNamesLeft:
            try:
                self.leftMotor = self.robot.getDevice(name)
                if self.leftMotor is not None:
                    break
            except Exception:
                pass

        for name in motorNamesRight:
            try:
                self.rightMotor = self.robot.getDevice(name)
                if self.rightMotor is not None:
                    break
            except Exception:
                pass

        self.maxMotorSpeed = 6.28
        if self.leftMotor:
            self.leftMotor.setPosition(float('inf'))
            self.leftMotor.setVelocity(0.0)
            self.maxMotorSpeed = self.leftMotor.getMaxVelocity() or 6.28

        if self.rightMotor:
            self.rightMotor.setPosition(float('inf'))
            self.rightMotor.setVelocity(0.0)

        # Variables de velocidad (-100 a 100)
        self.velLeftPercent = 0.0
        self.velRightPercent = 0.0

        # 2. Sensores de distancia
        # ps7 / distance sensor 2 = Frontal Izquierdo
        # ps0 / distance sensor 3 = Frontal Derecho
        # ps1 / ps2 / distance sensor 4 = Lateral Derecho
        self.distanceSensorLeft = None
        self.distanceSensorRight = None
        self.distanceSensorSideRight = None
        self.floorColorSensor = None

        dsLeftNames = ["ps7", "distance sensor2", "distance sensor1", "sensorDistanciaI", "ds_left", "ds0"]
        dsRightNames = ["ps0", "distance sensor3", "sensorDistanciaD", "ds_right", "ds1"]
        dsSideRightNames = ["distance sensor4", "ps1", "ps2", "ds_side_right", "sensorLateralD"]
        floorNames = ["colour_sensor", "colorPiso", "ground_sensor", "ps_floor"]

        for name in dsLeftNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.distanceSensorLeft = dev
                    self.distanceSensorLeft.enable(self.timeStep)
                    break
            except Exception:
                pass

        for name in dsRightNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.distanceSensorRight = dev
                    self.distanceSensorRight.enable(self.timeStep)
                    break
            except Exception:
                pass

        for name in dsSideRightNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.distanceSensorSideRight = dev
                    self.distanceSensorSideRight.enable(self.timeStep)
                    break
            except Exception:
                pass

        for name in floorNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.floorColorSensor = dev
                    self.floorColorSensor.enable(self.timeStep)
                    break
            except Exception:
                pass

        # 3. Bumpers / Touch sensors (opcionales)
        self.bumperLeft = None
        self.bumperRight = None
        bumperLeftNames = ["bumper_left", "touch sensor left", "bi", "ts_left"]
        bumperRightNames = ["bumper_right", "touch sensor right", "bd", "ts_right"]

        for name in bumperLeftNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.bumperLeft = dev
                    self.bumperLeft.enable(self.timeStep)
                    break
            except Exception:
                pass

        for name in bumperRightNames:
            try:
                dev = self.robot.getDevice(name)
                if dev is not None:
                    self.bumperRight = dev
                    self.bumperRight.enable(self.timeStep)
                    break
            except Exception:
                pass

        # 4. Radio / Telemetria con el Supervisor (Emitter / Receiver)
        self.emitter = None
        self.receiver = None

        try:
            self.emitter = self.robot.getDevice("emitter")
        except Exception:
            pass

        try:
            self.receiver = self.robot.getDevice("receiver")
            if self.receiver is not None:
                self.receiver.enable(self.timeStep)
        except Exception:
            pass

        # Estado de telemetria
        self._scoreActual: float = 0.0
        self._tiempoRestante: int = 480
        self._tiempoRealRestante: int = 540
        self._bateriaActual: float = 100.0
        self._ultimoPedidoEstado: float = -1.0

    def step(self) -> bool:
        """Avanza un paso de tiempo en la simulacion y actualiza telemetria."""
        self._actualizarTelemetria()
        return self.robot.step(self.timeStep) != -1

    def _syncVelocities(self) -> None:
        """Sincroniza las velocidades de motor con el supervisor via customData."""
        try:
            radL = (self.velLeftPercent / 100.0) * self.maxMotorSpeed
            radR = (self.velRightPercent / 100.0) * self.maxMotorSpeed
            self.robot.setCustomData(f"{radL:.3f},{radR:.3f}")
        except Exception:
            pass

    def _actualizarTelemetria(self) -> None:
        """Consulta y recibe periodicamente bateria y tiempos del Supervisor cada 5 segundos."""
        try:
            t = self.robot.getTime()
            # Enviar solicitud 'G' cada 5.0 segundos (o en el primer step)
            if self.emitter is not None and (t - self._ultimoPedidoEstado >= 5.0 or self._ultimoPedidoEstado < 0):
                self.emitter.send(struct.pack('c', b'G'))
                self._ultimoPedidoEstado = t

            # Procesar paquetes en cola del receptor
            if self.receiver is not None:
                while self.receiver.getQueueLength() > 0:
                    data = self.receiver.getBytes()
                    dataLen = len(data)
                    # Formato V2: 'c f i i f' (20 bytes) -> G, score, gameTime, realTime, battery
                    if dataLen == 20:
                        tup = struct.unpack('c f i i f', data)
                        if tup[0].decode('utf-8', errors='ignore') == 'G':
                            self._scoreActual = float(tup[1])
                            self._tiempoRestante = int(tup[2])
                            self._tiempoRealRestante = int(tup[3])
                            self._bateriaActual = float(tup[4])
                    # Formato V1: 'c f i i' (16 bytes)
                    elif dataLen == 16:
                        tup = struct.unpack('c f i i', data)
                        if tup[0].decode('utf-8', errors='ignore') == 'G':
                            self._scoreActual = float(tup[1])
                            self._tiempoRestante = int(tup[2])
                            self._tiempoRealRestante = int(tup[3])
                    self.receiver.nextPacket()
        except Exception:
            pass

    def setVel(self, velIzquierda: float, velDerecha: float) -> None:
        """Establece la velocidad de ambas ruedas (-100 a 100)."""
        self.setVI(velIzquierda)
        self.setVD(velDerecha)

    def setVelRad(self, velIzquierdaRad: float, velDerechaRad: float) -> None:
        """Establece la velocidad de ambas ruedas directamente en radianes/segundo (ej. 2.0, 4.0, 6.28)."""
        radL = max(-self.maxMotorSpeed, min(self.maxMotorSpeed, float(velIzquierdaRad)))
        radR = max(-self.maxMotorSpeed, min(self.maxMotorSpeed, float(velDerechaRad)))
        self.velLeftPercent = (radL / self.maxMotorSpeed) * 100.0
        self.velRightPercent = (radR / self.maxMotorSpeed) * 100.0
        if self.leftMotor:
            self.leftMotor.setVelocity(radL)
        if self.rightMotor:
            self.rightMotor.setVelocity(radR)
        self._syncVelocities()

    def setVI(self, velIzquierda: float) -> None:
        """Establece la velocidad de la rueda izquierda (-100 a 100)."""
        self.velLeftPercent = max(-100.0, min(100.0, float(velIzquierda)))
        if self.leftMotor:
            radPerSec = (self.velLeftPercent / 100.0) * self.maxMotorSpeed
            self.leftMotor.setVelocity(radPerSec)
        self._syncVelocities()

    def setVD(self, velDerecha: float) -> None:
        """Establece la velocidad de la rueda derecha (-100 a 100)."""
        self.velRightPercent = max(-100.0, min(100.0, float(velDerecha)))
        if self.rightMotor:
            radPerSec = (self.velRightPercent / 100.0) * self.maxMotorSpeed
            self.rightMotor.setVelocity(radPerSec)
        self._syncVelocities()

    def getVI(self) -> float:
        """Retorna la velocidad actual configurada de la rueda izquierda (-100 a 100)."""
        return self.velLeftPercent

    def getVD(self) -> float:
        """Retorna la velocidad actual configurada de la rueda derecha (-100 a 100)."""
        return self.velRightPercent

    def getDI(self) -> float:
        """Retorna el valor del sensor de distancia frontal izquierdo (0 a 100 aproximado)."""
        if self.distanceSensorLeft:
            val = self.distanceSensorLeft.getValue()
            return min(100.0, max(0.0, val / 10.0))
        return 0.0

    def getDD(self) -> float:
        """Retorna el valor del sensor de distancia frontal derecho (0 a 100 aproximado)."""
        if self.distanceSensorRight:
            val = self.distanceSensorRight.getValue()
            return min(100.0, max(0.0, val / 10.0))
        return 0.0

    def getDLD(self) -> float:
        """Retorna el valor del sensor de distancia lateral derecho si esta equipado (0 a 100 aproximado)."""
        if self.distanceSensorSideRight:
            val = self.distanceSensorSideRight.getValue()
            return min(100.0, max(0.0, val / 10.0))
        return self.getDD()

    def getColorPiso(self) -> float:
        """Retorna la lectura del sensor de piso / color."""
        if self.floorColorSensor:
            val = self.floorColorSensor.getValue()
            return min(100.0, max(0.0, val))
        return 0.0

    def getBI(self) -> bool:
        """Retorna True si el bumper izquierdo esta presionado."""
        if self.bumperLeft:
            return self.bumperLeft.getValue() > 0.5
        return False

    def getBD(self) -> bool:
        """Retorna True si el bumper derecho esta presionado."""
        if self.bumperRight:
            return self.bumperRight.getValue() > 0.5
        return False

    def getBateria(self) -> float:
        """Retorna el porcentaje actual de bateria (0.0 a 100.0) reportado por el Supervisor."""
        return round(self._bateriaActual, 2)

    def getTiempoRestante(self) -> int:
        """Retorna los segundos restantes de la partida (ej. 420) reportados por el Supervisor."""
        return self._tiempoRestante

    def getTiempoRealRestante(self) -> int:
        """Retorna los segundos restantes de tiempo real de partida."""
        return self._tiempoRealRestante

    def getPuntaje(self) -> float:
        """Retorna el puntaje actual del juego."""
        return self._scoreActual

    def getBateriaTexto(self) -> str:
        """Retorna un texto representativo del porcentaje de bateria (ej. '98.52%')."""
        return f"{self._bateriaActual:.2f}%"

    def mostrarEstado(self) -> None:
        """Muestra en la consola el mensaje oficial de telemetria."""
        print(f"Game Score: {self._scoreActual}  Remaining time: {self._tiempoRestante}  Remaining real-world time: {self._tiempoRealRestante} Battery : {self._bateriaActual:.2f}%")

    def esperar(self, segundos: float) -> None:
        """Pausa la ejecucion durante 'segundos' manteniendo los motores y la telemetria activos."""
        steps = int((segundos * 1000.0) / self.timeStep)
        for _ in range(max(1, steps)):
            if not self.step():
                break

    def tiempoActual(self) -> float:
        """Retorna el tiempo transcurrido en la simulacion en segundos."""
        return self.robot.getTime()
