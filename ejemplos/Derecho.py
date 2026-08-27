from RobotRL import RobotRL

def main():
    robot = RobotRL()
    print("==================================================")
    print("🚀 INICIANDO PRUEBA DE BATERIA POR VELOCIDADES (Derecho.py)")
    print("==================================================")

    # Fase 1: Velocidad 2.0 rad/s durante 2.0 segundos
    print("\n[FASE 1] 🟢 Velocidad: 2.0 rad/s (Baja) durante 2.0 segundos...")
    robot.setVelRad(2.0, 2.0)
    tiempoInicio = robot.tiempoActual()
    while robot.step():
        tiempoTranscurrido = robot.tiempoActual() - tiempoInicio
        if tiempoTranscurrido >= 2.0:
            break

    # Fase 2: Velocidad 4.0 rad/s durante 3.0 segundos
    print("\n[FASE 2] 🟠 Velocidad: 4.0 rad/s (Media) durante 3.0 segundos...")
    robot.setVelRad(4.0, 4.0)
    tiempoInicio = robot.tiempoActual()
    while robot.step():
        tiempoTranscurrido = robot.tiempoActual() - tiempoInicio
        if tiempoTranscurrido >= 3.0:
            break

    # Fase 3: Velocidad 6.28 rad/s (Maxima) durante 2.8 segundos
    print("\n[FASE 3] 🔴 Velocidad: 6.28 rad/s (MAXIMA) durante 2.8 segundos...")
    robot.setVelRad(6.28, 6.28)
    tiempoInicio = robot.tiempoActual()
    while robot.step():
        tiempoTranscurrido = robot.tiempoActual() - tiempoInicio
        if tiempoTranscurrido >= 2.8:
            break

    # Detener el robot
    print("\n[FIN] ⏹️ Robot detenido (Velocidad: 0.0 rad/s). Prueba completada con exito.")
    robot.setVelRad(0.0, 0.0)
    while robot.step():
        pass

if __name__ == "__main__":
    main()
