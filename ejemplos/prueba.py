from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# Obtener los motores
motor_izq = robot.getDevice("wheel1 motor")
motor_der = robot.getDevice("wheel2 motor")

# Poner velocidad 0 → robot quieto
motor_izq.setVelocity(0.0)
motor_der.setVelocity(0.0)

while robot.step(time_step) != -1:
    pass  # no hace nada, el robot se queda quieto
