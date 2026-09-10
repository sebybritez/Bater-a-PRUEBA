from RobotRL import RobotRL

def main():
    robot = RobotRL()
    ultimo_tiempo_telemetria = -1.0

    while robot.step():
        tiempo_actual = robot.tiempoActual()

        # Reporte de Telemetría cada 5 segundos
        if tiempo_actual - ultimo_tiempo_telemetria >= 5.0 or ultimo_tiempo_telemetria < 0:
            robot.mostrarEstado()
            ultimo_tiempo_telemetria = tiempo_actual

        # Lectura de Sensores
        dist_izq = robot.getDI()
        dist_der = robot.getDD()
        dist_lat_der = robot.getDLD()
        choque_izq = robot.getBI()
        choque_der = robot.getBD()

        # Lógica de Navegación por la Derecha (Seguidor de Pared Derecha)
        # A. Si choca con un bumper o pared muy cerca al frente -> retroceder y girar izquierda
        if choque_izq or choque_der or dist_izq > 60 or dist_der > 60:
            robot.setVelRad(-3.0, -3.0)
            robot.esperar(0.2)
            robot.setVelRad(-3.0, 3.0)
            robot.esperar(0.3)

        # B. Si hay pared al frente (distancia media) -> girar a la izquierda
        elif dist_izq > 35 or dist_der > 35:
            robot.setVelRad(-2.5, 3.5)

        # C. Si la pared derecha está muy cerca -> alejarse suavemente hacia la izquierda
        elif dist_lat_der > 45:
            robot.setVelRad(3.0, 4.5)

        # D. Si la pared derecha está a distancia ideal (18 a 45) -> avanzar derecho
        elif dist_lat_der >= 18:
            robot.setVelRad(4.5, 4.5)

        # E. Si no detecta pared a la derecha (esquina abierta) -> doblar a la derecha para rodearla
        else:
            robot.setVelRad(4.5, 2.0)

if __name__ == "__main__":
    main()
