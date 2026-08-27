from controller import Robot, Motor, DistanceSensor, TouchSensor, Camera
import time

class RobotRL:
    """
    Clase simplificada RobotRL para facilitar la programacion educativa de robots en Webots / Erebus.
    Utiliza directamente los nombres de hardware del robot 'Erebus_Bot' (custom_robot.proto)
    para evitar advertencias o errores en la consola de Webots.
    Todo el codigo y variables siguen la notacion camelCase.
    """

    def __init__(self):
        self.robot = Robot()
        self.timeStep = int(self.robot.getBasicTimeStep())
        if self.timeStep <= 0:
            self.timeStep = 32

        # 1. Motores (nombres exactos de Erebus_Bot: wheel1 motor = Izquierda, wheel2 motor = Derecha)
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

        # 2. Sensores de distancia frontales (ps7 = Frontal Izquierdo, ps0 = Frontal Derecho)
        self.distanceSensorLeft = None
        self.distanceSensorRight = None
        self.floorColorSensor = None

        dsLeftNames = ["ps7", "sensorDistanciaI", "ds_left", "ds0"]
        dsRightNames = ["ps0", "sensorDistanciaD", "ds_right", "ds1"]
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

        # Intentar conectar solo si estan presentes
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

    def step(self) -> bool:
        """Avanza un paso de tiempo en la simulacion."""
        return self.robot.step(self.timeStep) != -1

    def _syncVelocities(self) -> None:
        """Sincroniza las velocidades de motor con el supervisor via customData."""
        try:
            radL = (self.velLeftPercent / 100.0) * self.maxMotorSpeed
            radR = (self.velRightPercent / 100.0) * self.maxMotorSpeed
            self.robot.setCustomData(f"{radL:.3f},{radR:.3f}")
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
        """Retorna el valor del sensor de distancia frontal izquierdo ps7 (0 a 100 aproximado)."""
        if self.distanceSensorLeft:
            val = self.distanceSensorLeft.getValue()
            # Escalar lectura IR para aproximar porcentaje de proximidad (0 - 100)
            return min(100.0, max(0.0, val / 10.0))
        return 0.0

    def getDD(self) -> float:
        """Retorna el valor del sensor de distancia frontal derecho ps0 (0 a 100 aproximado)."""
        if self.distanceSensorRight:
            val = self.distanceSensorRight.getValue()
            return min(100.0, max(0.0, val / 10.0))
        return 0.0

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

    def esperar(self, segundos: float) -> None:
        """Pausa la ejecucion durante 'segundos' manteniendo los motores activos en Webots."""
        steps = int((segundos * 1000.0) / self.timeStep)
        for _ in range(max(1, steps)):
            if not self.step():
                break

    def tiempoActual(self) -> float:
        """Retorna el tiempo transcurrido en la simulacion en segundos."""
        return self.robot.getTime()

    def getBateriaTexto(self) -> str:
        """Retorna un texto representativo de la bateria."""
        return "100%"
