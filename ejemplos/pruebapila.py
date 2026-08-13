from controller import Robot

robot = Robot()
time_step = int(robot.getBasicTimeStep())  # en milisegundos

# ──────────────────────────────────────────────
# Configuración de batería gestionada por Python
# ──────────────────────────────────────────────
battery_level = 100.0          # % inicial
CONSUMO_POR_SEGUNDO = 0.01     # % que se descuenta por segundo de simulación

last_battery_update = 0.0      # última vez que actualizamos la batería (s)
last_print_time = 0.0          # última vez que imprimimos (s)

while robot.step(time_step) != -1:
    current_time = robot.getTime()  # segundos de simulación transcurridos

    # ── Actualizar batería cada step ──────────────────────────────────────────
    dt = current_time - last_battery_update          # tiempo transcurrido (s)
    battery_level -= CONSUMO_POR_SEGUNDO * dt        # descuento proporcional
    battery_level = max(battery_level, 0.0)          # nunca bajar de 0
    last_battery_update = current_time

    # ── Imprimir cada 1 segundo de simulación ─────────────────────────────────
    if current_time - last_print_time >= 1.0:
        print(f"[{current_time:.1f}s] Nivel de Batería: {battery_level:.4f}%")
        last_print_time = current_time
