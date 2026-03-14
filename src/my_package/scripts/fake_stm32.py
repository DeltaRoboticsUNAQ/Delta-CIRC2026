#!/usr/bin/env python3
"""
Simulador de STM32 para pruebas sin hardware real.

Crea un puerto serial virtual (PTY) e imprime el nombre del dispositivo
para usarlo con el stm32_hardware_bridge de ROS 2.

Protocolo recibido (ROS → STM32):
  CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>\\r\\n

Protocolo enviado (STM32 → ROS):
  FB:BE:<ticks>;W1:<ticks>;W2:<ticks>;P1:<adc>;P2:<adc>\\n

Uso:
  # Terminal 1 — iniciar el simulador:
  python3 fake_stm32.py

  # Terminal 2 — lanzar el bridge apuntando al puerto virtual:
  ros2 launch my_package hardware.launch.py port:=<puerto que imprime el script>
"""

import math
import os
import pty
import select
import sys
import termios
import threading
import time
import tty

# ── Calibración (debe coincidir con los parámetros del bridge) ─────────────────
BASE_TICKS_PER_REV  = 2000
WRIST_TICKS_PER_REV = 2000
POT_MIN  = 200
POT_MAX  = 3800
ACT_RANGE_RAD = 3.0

# ── Velocidad de respuesta del simulador ───────────────────────────────────────
# Fracción por la que el estado actual avanza hacia el target en cada tick.
SLEW_ALPHA = 0.08   # 1.0 = instantáneo, valores bajos = respuesta lenta

# ── Frecuencia de envío de feedback ───────────────────────────────────────────
FEEDBACK_HZ = 50.0


def _deg_to_ticks(deg: float, ticks_per_rev: int) -> int:
    return int((deg / 360.0) * ticks_per_rev)


def _pct_to_adc(pct: float) -> int:
    clamped = max(0.0, min(100.0, pct))
    return int(POT_MIN + (clamped / 100.0) * (POT_MAX - POT_MIN))


class FakeSTM32:
    def __init__(self, master_fd: int):
        self._fd = master_fd

        # Estado simulado (en las unidades "nativas" del STM32)
        self._be_ticks: float = 0.0   # base encoder ticks
        self._w1_ticks: float = 0.0   # wrist motor 1 ticks
        self._w2_ticks: float = 0.0   # wrist motor 2 ticks
        self._p1_adc:   float = float(_pct_to_adc(50.0))   # actuador 1 ADC
        self._p2_adc:   float = float(_pct_to_adc(50.0))   # actuador 2 ADC

        # Targets (actualizados con cada CMD recibido)
        self._tgt_be:  float = 0.0
        self._tgt_w1:  float = 0.0
        self._tgt_w2:  float = 0.0
        self._tgt_p1:  float = float(_pct_to_adc(50.0))
        self._tgt_p2:  float = float(_pct_to_adc(50.0))

        self._lock = threading.Lock()
        self._buf  = b''
        self._running = True

    # ── Loop principal ─────────────────────────────────────────────────────────

    def run(self):
        read_thread = threading.Thread(target=self._read_loop, daemon=True)
        read_thread.start()

        period = 1.0 / FEEDBACK_HZ
        while self._running:
            t0 = time.monotonic()
            self._step_simulation()
            self._send_feedback()
            elapsed = time.monotonic() - t0
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def stop(self):
        self._running = False

    # ── Simulación (slew hacia target) ────────────────────────────────────────

    def _step_simulation(self):
        with self._lock:
            self._be_ticks += SLEW_ALPHA * (self._tgt_be - self._be_ticks)
            self._w1_ticks += SLEW_ALPHA * (self._tgt_w1 - self._w1_ticks)
            self._w2_ticks += SLEW_ALPHA * (self._tgt_w2 - self._w2_ticks)
            self._p1_adc   += SLEW_ALPHA * (self._tgt_p1 - self._p1_adc)
            self._p2_adc   += SLEW_ALPHA * (self._tgt_p2 - self._p2_adc)

    # ── Envío de feedback ─────────────────────────────────────────────────────

    def _send_feedback(self):
        with self._lock:
            be = self._be_ticks
            w1 = self._w1_ticks
            w2 = self._w2_ticks
            p1 = self._p1_adc
            p2 = self._p2_adc

        line = (
            f'FB:BE:{be:.1f};'
            f'W1:{w1:.1f};'
            f'W2:{w2:.1f};'
            f'P1:{int(p1)};'
            f'P2:{int(p2)}\n'
        )
        try:
            os.write(self._fd, line.encode('ascii'))
        except OSError:
            self._running = False

    # ── Lectura de comandos ───────────────────────────────────────────────────

    def _read_loop(self):
        while self._running:
            try:
                ready, _, _ = select.select([self._fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._fd, 256)
                if not data:
                    continue
                self._buf += data
                while b'\n' in self._buf:
                    idx = self._buf.index(b'\n')
                    line = self._buf[:idx].decode('ascii', errors='ignore').strip()
                    self._buf = self._buf[idx + 1:]
                    if line:
                        self._parse_command(line)
            except OSError:
                break

    def _parse_command(self, line: str):
        """
        Parsea CMD:B:<deg>;W1:<deg>;W2:<deg>;A1:<pct>;A2:<pct>
        y actualiza los targets de la simulación.
        """
        if not line.startswith('CMD:'):
            return
        try:
            fields: dict = {}
            for part in line[4:].split(';'):
                if ':' in part:
                    k, v = part.split(':', 1)
                    fields[k.strip()] = float(v.strip())

            b_deg  = fields.get('B',  0.0)
            w1_deg = fields.get('W1', 0.0)
            w2_deg = fields.get('W2', 0.0)
            a1_pct = fields.get('A1', 50.0)
            a2_pct = fields.get('A2', 50.0)

            with self._lock:
                self._tgt_be = float(_deg_to_ticks(b_deg,  BASE_TICKS_PER_REV))
                self._tgt_w1 = float(_deg_to_ticks(w1_deg, WRIST_TICKS_PER_REV))
                self._tgt_w2 = float(_deg_to_ticks(w2_deg, WRIST_TICKS_PER_REV))
                self._tgt_p1 = float(_pct_to_adc(a1_pct))
                self._tgt_p2 = float(_pct_to_adc(a2_pct))

            print(
                f'[CMD] B={b_deg:.1f}°  W1={w1_deg:.1f}°  W2={w2_deg:.1f}°  '
                f'A1={a1_pct:.1f}%  A2={a2_pct:.1f}%',
                flush=True,
            )
        except (ValueError, KeyError) as e:
            print(f'[WARN] No se pudo parsear: "{line}" → {e}', flush=True)


# ── Punto de entrada ───────────────────────────────────────────────────────────

def main():
    master_fd, slave_fd = pty.openpty()

    # Poner el slave en modo raw para que no interfiera con el protocolo ASCII
    tty.setraw(slave_fd)

    slave_path = os.ttyname(slave_fd)

    print('=' * 60)
    print('  Fake STM32 — simulador de hardware para brazo robótico')
    print('=' * 60)
    print(f'  Puerto virtual creado : {slave_path}')
    print()
    print('  Lanzar el bridge en otra terminal con:')
    print(f'    ros2 launch my_package hardware.launch.py port:={slave_path}')
    print()
    print('  Presiona Ctrl+C para salir.')
    print('=' * 60)
    sys.stdout.flush()

    sim = FakeSTM32(master_fd)
    try:
        sim.run()
    except KeyboardInterrupt:
        print('\n[INFO] Simulador detenido.')
    finally:
        sim.stop()
        try:
            os.close(slave_fd)
            os.close(master_fd)
        except OSError:
            pass


if __name__ == '__main__':
    main()
