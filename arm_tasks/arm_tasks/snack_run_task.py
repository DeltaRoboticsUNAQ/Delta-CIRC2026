# src/arm_tasks/arm_tasks/snack_run_task.py
"""
Task: Snack Run — CIRC 2026
Hardware del brazo:
  - Base: motor DC + encoder     → bracket_joint
  - Hombro: actuador + potenciómetro → humerus_low_joint
  - Codo: actuador + potenciómetro   → forearm_low_joint
  - Muñeca: diferencial 2 motores DC → ubracket_joint + endeffector_joint
"""

import time
from arm_tasks.base_task import BaseTask
from rclpy.node import Node


class SnackRunTask(BaseTask):
    """
    Snack Run:
    1. Tap del credit cube al lector
    2. Presionar botón de selección (botón 1"x1")
    3. Posicionar caja de transporte
    4. Servicio de la máquina si es necesario
    """

    # ── Poses específicas para esta task ─────────────────────────────────────
    # IMPORTANTE: Ajustar estos valores en campo midiendo
    # la posición real de la máquina expendedora

    # Posición frente al lector NFC/crédito
    # [bracket, humerus, forearm, ubracket, endeffector]
    POSE_CREDIT_APPROACH = [0.3,  -0.6,  0.7,  0.0,  0.0]
    POSE_CREDIT_TAP      = [0.3,  -0.6,  0.85, 0.0,  0.0]  # +forearm para tap

    # Posición frente al botón de selección
    POSE_BUTTON_APPROACH = [0.0,  -0.7,  0.75, 0.0,  0.0]
    POSE_BUTTON_PRESS    = [0.0,  -0.7,  0.85, 0.0,  0.0]  # +forearm para press

    # Posición para servicio (cables, switches) - ajustar en campo
    POSE_SERVICE_PANEL   = [-0.3, -0.5,  0.6,  0.1,  0.0]

    def __init__(self, node: Node):
        super().__init__(node, "SnackRun")
        self.credits_loaded = 0
        self.items_dispensed = 0

    def execute(self) -> bool:
        self._start_timer()
        self.log_info("═══ INICIANDO SNACK RUN ═══")

        try:
            # ── Paso 1: Ir a HOME ─────────────────────────────────────────
            self.log_info("Paso 1/4: Posición inicial")
            if not self.arm.go_home(duration_sec=2.0):
                self.log_error("No se pudo ir a HOME")
                return False

            # ── Paso 2: Tap del credit cube ───────────────────────────────
            self.log_info("Paso 2/4: Tap credit cube")
            if not self._tap_credit_cube(n_taps=1):
                self.log_error("Falló tap de crédito")
                # Continuar de todos modos (puede haber crédito previo)

            # ── Paso 3: Seleccionar item en la máquina ────────────────────
            self.log_info("Paso 3/4: Seleccionando item")
            if not self._press_selection_button():
                self.log_error("Falló selección de item")
                return False

            # ── Paso 4: Volver a HOME ─────────────────────────────────────
            self.log_info("Paso 4/4: Volviendo a HOME")
            self.arm.go_home(duration_sec=2.0)

            elapsed = self.elapsed_time()
            self.log_info(
                f"═══ SNACK RUN COMPLETADA en {elapsed:.1f}s ═══"
            )
            return True

        except Exception as e:
            self.log_error(f"Excepción: {e}")
            return False

        finally:
            # Siempre volver a HOME al terminar
            self.go_home_safe()

    def _tap_credit_cube(self, n_taps: int = 1) -> bool:
        """
        Toca el credit cube contra el lector de la máquina.
        El cubo está en el end-effector del brazo.
        n_taps: número de taps (uno por item a comprar)
        """
        self.log_info(f"Ejecutando {n_taps} tap(s) de crédito")

        for i in range(n_taps):
            self.log_info(f"  Tap {i+1}/{n_taps}")

            # Approach al lector
            if not self.arm.go_to_joint_positions(
                self.POSE_CREDIT_APPROACH, duration_sec=2.0
            ):
                return False

            # Tap: avanzar forearm_low_joint (codo)
            # Nota: el actuador de hombro/codo usa potenciómetro,
            # así que el movimiento es más lento
            if not self.arm.go_to_joint_positions(
                self.POSE_CREDIT_TAP, duration_sec=1.0
            ):
                return False

            # Mantener tap 0.5s
            self.wait(0.5, "manteniendo tap")

            # Retractar
            if not self.arm.go_to_joint_positions(
                self.POSE_CREDIT_APPROACH, duration_sec=1.0
            ):
                return False

            self.credits_loaded += 1

            # Esperar entre taps si hay múltiples
            if i < n_taps - 1:
                self.wait(0.5)

        self.log_info(f"✓ {n_taps} crédito(s) cargado(s)")
        return True

    def _press_selection_button(self) -> bool:
        """
        Presiona el botón de selección de item (1"x1" ≈ 25mm x 25mm).
        Usa el forearm_low_joint (codo) para el movimiento de presión,
        ya que tiene potenciómetro y responde bien a movimientos lentos.
        """
        self.log_info("Presionando botón de selección")

        # Usar método especializado de press_button del arm client
        success = self.arm.press_button(
            approach_positions=self.POSE_BUTTON_APPROACH,
            press_depth_j3=0.08  # 0.08 rad de avance en forearm
        )

        if success:
            self.items_dispensed += 1
            self.log_info(f"✓ Botón presionado — items: {self.items_dispensed}")
        else:
            self.log_error("✗ Falló press del botón")

        return success

    def service_machine(self, service_type: str) -> bool:
        """
        Servicios de la máquina expendedora.
        Llamar según el manual técnico de CIRC 2026.

        service_type: 'percussive' | 'bypass_code' | 'reconnect'
        """
        self.log_info(f"Servicio de máquina: {service_type}")

        if service_type == 'percussive':
            # Golpe suave con el end-effector
            return self._percussive_maintenance()

        elif service_type == 'reconnect':
            # Conectar componente suelto
            return self._reconnect_component()

        else:
            self.log_warn(f"Tipo de servicio '{service_type}' no implementado")
            return False

    def _percussive_maintenance(self) -> bool:
        """
        Golpe suave de mantenimiento (percussive maintenance).
        Mueve el brazo a la posición del panel y hace un tap rápido.
        """
        self.log_info("Mantenimiento percusivo")

        if not self.arm.go_to_joint_positions(
            self.POSE_SERVICE_PANEL, duration_sec=2.0
        ):
            return False

        # Golpe rápido: avanzar y retractar en forearm
        tap_pos = list(self.POSE_SERVICE_PANEL)
        tap_pos[2] += 0.1  # +0.1 rad en forearm

        if not self.arm.go_to_joint_positions(tap_pos, duration_sec=0.3):
            return False

        return self.arm.go_to_joint_positions(
            self.POSE_SERVICE_PANEL, duration_sec=0.3
        )

    def _reconnect_component(self) -> bool:
        """
        Reconecta un componente usando la muñeca diferencial.
        """
        self.log_info("Reconectando componente")

        if not self.arm.go_to_joint_positions(
            self.POSE_SERVICE_PANEL, duration_sec=2.0
        ):
            return False

        # Rotar muñeca para reconectar
        return self.arm.wrist_differential_move(
            pitch_rad=0.2,
            roll_rad=1.57,  # 90° de rotación
            duration_sec=1.0
        )