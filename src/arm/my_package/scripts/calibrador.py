import hid
import time
import sys

# Conectar al Acteck AGJ-4000
try:
    joystick = hid.device()
    joystick.open(0x12bd, 0xa02f)
    joystick.set_nonblocking(True)
    print("🕹️ Joystick conectado. Iniciando mapeo crudo...")
    time.sleep(1)
except Exception as e:
    print(f"Error conectando: {e}")
    print("Recuerda los permisos: sudo chmod 666 /dev/hidraw*")
    sys.exit(1)

try:
    while True:
        report = joystick.read(64)
        if report:
            # Limpiar consola para que la lectura se mantenga fija y no baje como loca
            print(chr(27) + "[2J" + chr(27) + "[H", end="")
            print("=== CALIBRACIÓN DEL EXTREME PILOT AGJ-4000 ===")
            
            # Ejes (Valores típicamente de 0 a 255)
            print(f"Eje Lateral (0) : {report[0]:3d}  <-- Izquierda/Derecha")
            print(f"Eje Frontal (1) : {report[1]:3d}  <-- Arriba/Abajo")
            print(f"Eje Twist   (2) : {report[2]:3d}  <-- Torsión de la palanca")
            print(f"Throttle    (4) : {report[4]:3d}  <-- Palanquita de aceleración")
            
            print("\n--- Botones (Representación en Bits) ---")
            # Convertimos el número a binario para ver claramente qué bit se enciende
            print(f"Byte 6: {bin(report[6])[2:]:>8}  <-- Ej. Gatillo, botones frontales")
            print(f"Byte 7: {bin(report[7])[2:]:>8}  <-- Ej. Botones de la base")
            
            print("\nPresiona Ctrl+C para salir.")
            print("==============================================")
            
        time.sleep(0.05)  # Refresco a 20Hz para que sea legible
except KeyboardInterrupt:
    print("\nSaliendo del calibrador...")