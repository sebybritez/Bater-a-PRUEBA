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

    DEFAULT_CONSUMO: float =0.016   # % por segundo (baja 0.016% por cada segundo de simulación)
    PRINT_INTERVAL: float = 1.0     # cada cuántos segundos se imprime/envía

    def __init__(
        self,
        robot_num: int,
        rws,
        consumo_por_segundo: float = DEFAULT_CONSUMO,
    ) -> None:
        self._num: int = robot_num
        self._rws = rws

        self.active: bool = True                # Siempre activa (batería de regalo por defecto)
        self.has_battery_pack: bool = False     # True si tiene el componente Battery en el JSON
        self.level: float = 100.0               # % de batería restante
        self.consumo_por_segundo: float = consumo_por_segundo

        self._last_time: Optional[float] = None        # último step procesado
        self._last_print: Optional[float] = None       # último envío a la UI

    # ------------------------------------------------------------------
    # API pública
    # ------------------------------------------------------------------

    def configure(self, has_component: bool = False, max_energy: float = 100.0) -> None:
        """Configura la batería según si tiene o no el componente Battery.

        - Si NO tiene componente (has_component=False): recibe batería estándar
          de regalo (consumo normal DEFAULT_CONSUMO).
        - Si SÍ tiene componente (has_component=True): consume la mitad (dura el doble).

        Args:
            has_component (bool): True si el robot equipó el componente Battery.
            max_energy (float): Valor de maxEnergy del componente Battery.
        """
        self.active = True
        self.has_battery_pack = has_component
        self.level = 100.0
        self._last_time = None
        self._last_print = None

        if has_component:
            # Consume a la mitad de velocidad (dura el doble)
            mult = (100.0 / max_energy) * 0.5
            self.consumo_por_segundo = round(self.DEFAULT_CONSUMO * mult, 6)
            Console.log_info(
                f"[Robot {self._num}] Batería MEJORADA equipada (maxEnergy={max_energy}). "
                f"Consumo a mitad de velocidad (duración x2): {self.consumo_por_segundo:.6f} %/s"
            )
        else:
            # Batería estándar de regalo
            self.consumo_por_segundo = self.DEFAULT_CONSUMO
            Console.log_info(
                f"[Robot {self._num}] Batería ESTÁNDAR de regalo activada. "
                f"Consumo normal: {self.consumo_por_segundo:.6f} %/s"
            )

    def deactivate(self) -> None:
        """Desactiva la batería."""
        self.active = False
        self.level = 100.0
        self._last_time = None
        self._last_print = None

    @property
    def is_depleted(self) -> bool:
        """Indica si la batería está activa y se ha agotado por completo."""
        return self.active and self.level <= 0.0

    def update(self, time_elapsed: float) -> None:
        """Descuenta batería y envía el nivel actualizado a la ventana HTML.

        Debe llamarse en cada step del supervisor (ej. dentro de
        ``update_time_elapsed``). Si la batería no está activa o ya está agotada,
        retorna inmediatamente.

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

        # Enviar al HTML cada PRINT_INTERVAL segundos de simulación (o si llegó a 0)
        if self._last_print is not None and (time_elapsed - self._last_print >= self.PRINT_INTERVAL or self.level <= 0.0):
            Console.log_info(f"[Robot {self._num}] Nivel de Batería: {self.level:.2f}%")
            self._rws.send("batteryUpdate", f"{self._num},{self.level:.2f}")
            self._last_print = time_elapsed

