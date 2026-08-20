"""bateria.py – Módulo de batería reutilizable para supervisores de Erebus.

Uso básico
----------
En el __init__ del Robot (o cualquier supervisor):

    from bateria import Battery
    self.battery = Battery(robot_num=self._num, rws=self._erebus.rws)

Para activarla (cuando se detecta el componente Battery en el JSON):

    self.battery.configure(max_energy=100)   # activa + ajusta consumo

En el update loop (ej. update_time_elapsed):

    self.battery.update(time_elapsed)

Para leer el estado desde afuera:

    self.battery.level          # float, % restante (0.0 – 100.0)
    self.battery.active         # bool, True si tiene batería
"""

from __future__ import annotations
from typing import Optional

from ConsoleLog import Console


class Battery:
    """Gestiona el nivel de batería de un robot individual.

    Es completamente independiente de la clase Robot: sólo necesita el
    número de robot y una referencia al RobotWindowSender (rws) para
    poder enviar actualizaciones a la ventana HTML.

    Args:
        robot_num (int): Índice del robot (0 o 1).
        rws: Instancia de RobotWindowSender (erebus.rws).
        consumo_por_segundo (float): % descontado por segundo de simulación.
            Por defecto 0.01 (100 % / 10 000 s).
    """

    DEFAULT_CONSUMO: float = 0.01   # % por segundo
    PRINT_INTERVAL: float = 1.0     # cada cuántos segundos se imprime/envía

    def __init__(
        self,
        robot_num: int,
        rws,
        consumo_por_segundo: float = DEFAULT_CONSUMO,
    ) -> None:
        self._num: int = robot_num
        self._rws = rws

        self.active: bool = False               # True cuando se detecta componente Battery
        self.level: float = 100.0               # % de batería restante
        self.consumo_por_segundo: float = consumo_por_segundo

        self._last_time: Optional[float] = None        # último step procesado
        self._last_print: Optional[float] = None       # último envío a la UI

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def configure(self, max_energy: float = 100.0) -> None:
        """Activa la batería y ajusta el consumo según la energía máxima del JSON.

        Llamar una vez cuando se detecta el componente Battery en el JSON
        del robot. Si se llama de nuevo (recarga de JSON) resetea el estado.

        Args:
            max_energy (float): Valor de maxEnergy del componente Battery.
        """
        self.active = True
        self.level = 100.0
        self._last_time = None
        self._last_print = None

        # A mayor energía máxima, menor consumo relativo (% por segundo)
        self.consumo_por_segundo = round(100.0 / max_energy * 0.01, 6)

        Console.log_info(
            f"[Robot {self._num}] Batería activada: maxEnergy={max_energy}. "
            f"Consumo: {self.consumo_por_segundo:.6f} %/s"
        )

    def deactivate(self) -> None:
        """Desactiva la batería (robot sin componente Battery)."""
        self.active = False
        self.level = 100.0
        self._last_time = None
        self._last_print = None

    def update(self, time_elapsed: float) -> None:
        """Descuenta batería y envía el nivel actualizado a la ventana HTML.

        Debe llamarse en cada step del supervisor (ej. dentro de
        ``update_time_elapsed``). Si la batería no está activa, retorna
        inmediatamente sin hacer nada.

        Args:
            time_elapsed (float): Tiempo transcurrido de simulación (en segundos).
        """
        if not self.active:
            return

        # Primer step: inicializar referencias de tiempo
        if self._last_time is None:
            self._last_time = time_elapsed
            self._last_print = time_elapsed
            return

        # Descontar consumo
        dt: float = time_elapsed - self._last_time
        if dt > 0:
            self.level -= self.consumo_por_segundo * dt
            self.level = max(self.level, 0.0)
            self._last_time = time_elapsed

        # Enviar al HTML cada PRINT_INTERVAL segundos de simulación
        if (self._last_print is not None and
                time_elapsed - self._last_print >= self.PRINT_INTERVAL):
            Console.log_info(
                f"[Robot {self._num}] Nivel de Batería: {self.level:.2f}%"
            )
            self._rws.send(
                "batteryUpdate",
                f"{self._num},{self.level:.2f}"
            )
            self._last_print = time_elapsed
