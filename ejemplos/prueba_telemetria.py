from RobotRL import RobotRL

def main():
    robot = RobotRL()
    robot.setVelRad(3.0, 3.0)
    ultimo_print = -1.0

    while robot.step():
        tiempo_actual = robot.tiempoActual()

        # Mostrar estado en consola cada 5 segundos con el formato oficial
        if tiempo_actual - ultimo_print >= 5.0 or ultimo_print < 0:
            robot.mostrarEstado()
            ultimo_print = tiempo_actual

        # Si detecta obstáculo adelante, girar suavemente
        if robot.getDI() > 50 or robot.getDD() > 50:
            robot.setVelRad(-2.0, 2.0)
        else:
            robot.setVelRad(3.0, 3.0)

if __name__ == "__main__":
    main()
