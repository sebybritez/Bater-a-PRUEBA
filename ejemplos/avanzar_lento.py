from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())

# ── Obtener motores ────────────────────────────────────────────────────────────
motor_izq = robot.getDevice("wheel1 motor")
motor_der = robot.getDevice("wheel2 motor")

# Velocidad infinita (posición), modo velocidad
motor_izq.setPosition(float('inf'))
motor_der.setPosition(float('inf'))

# Velocidad lenta: 1.0 rad/s (máx permitido suele ser 6.28)
VELOCIDAD = 1.0
motor_izq.setVelocity(VELOCIDAD)
motor_der.setVelocity(VELOCIDAD)

print("Robot avanzando lento...")

while robot.step(time_step) != -1:
    pass  # Los motores mantienen la velocidad solos
