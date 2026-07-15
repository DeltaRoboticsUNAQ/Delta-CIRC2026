# Control de chasis desde base con control de Xbox conectado por Bluetooth.
# Se conecta a través de rosbridge con la libreria roslibpy.
# Usa axis y botónes del control. 
# Se suscribe al tópico cmd_vel y publica mensajes tipo Twist para controlar el chasis.
# 
# Sebastián López Tena | bastianlopezt@gmail.com

import pygame
import sys

import roslibpy
from time import sleep

# Constantes
DELAY = 0.05
DRIFT = 0.1
THRESHOLD = 0.05

# /////////////////////////////////////////////////////////////////
# ROS Websocket con roslibpy
ros  = roslibpy.Ros(host='192.168.1.30', port=9090)
ros.run() # Inicializar Websocket

# Chassis velocity publisher
chassis_pub = roslibpy.Topic(ros, '/cmd_vel', 'geometry_msgs/Twist')

# Inicializar pygame. Si no encuentra controles conectados, se detiene el código.
pygame.init()
pygame.joystick.init()

# Listar joysticks
def list_joysticks():
    
    count = pygame.joystick.get_count()
    if count == 0:
        print("No joysticks detected. Conect a controller and run again.")
        sys.exit()
        return False
    
    print(f"{count} detected joystick(s): ")
    for i in range(count):
        js = pygame.joystick.Joystick(i)
        js.init()
        print(f" {i}: {js.get_name()}")
    return True

# Se enlistan joysticks. Si se regresa False, se termina el código con error.
if not list_joysticks():
    sys.exit(1)
    
# Use first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"\nUsing joystick: {joystick.get_name()}")
print("Press buttons or move sticks. Ctrl+C to exit.\n")

# Map buttons
button_names = {
    0: 'A',
    1: 'B',
    2: 'X',
    3: 'Y',
    4: 'SHARE',
    5: 'ONOFF',
    6: 'OPTIONS',
    7: 'Left_stick',
    8: 'Right_stick',
    9: 'LB',
    10: 'RB',
    11: 'UP',
    12: 'DOWN',
    13: 'LEFT',
    14: 'RIGHT'
}

# Revisar estado previo para detectar cambios y evitar inputs repetidos
buttons_prev_states = [False] * joystick.get_numbuttons()
axis_prev = [0.0] * joystick.get_numaxes()
hat_prev = (0, 0)

modes = ["Chassis", "Arm"]
mode_idx = 0

def change_mode():
    global mode_idx
    global modes
    mode_idx = (mode_idx + 1) % len(modes) # Alternar en loop
    print(f"Switched to {modes[mode_idx]} mode.")

# ///////////////////////////////////////////////////////////////
# Funciones de movimiento con control

# ///////////////////////////////////////////////////////////////
# CHASSIS
# Mover con flechas
def chassis_command(event):
    if event == 'UP':
        linear_x = 100
        angular_x = 0.0
    elif event == 'DOWN':
        linear_x = -100.0
        angular_z = 0.0
    elif event == 'RIGHT':
        linear_x == 0.0
        angular_z == 1.0
    elif event == 'LEFT':
        linear_x = 0.0
        angular_z = -1.0
    elif event == 'STOP' or event == 'B':
        linear_x = 0.0
        angular_x = 0.0
        
    try:
        twist_msg = {
            'linear': {'x': linear_x, 'y':0.0, 'z':0.0},
            'angular': {'x':0.0, 'y':0.0, 'z':angular_z}
        }
        chassis_pub.publish(twist_msg)
        return True
    
    except:
        print(f"Chassis command not recognized: {event}")
        return False

# Mover con joysticks
def chassis_axis_command(axis_values):
    try:
        linear_x = ((axis_values[1]-axis_values[3])/2.0)*255.0
        angular_z = ((axis_values[1]+axis_values[3])/2.0)*255.0
        twist_msg = {
        'linear': {'x': linear_x, 'y': 0.0, 'z': 0.0},
        'angular': {'x': 0.0, 'y': 0.0, 'z': angular_z}
        }
        chassis_pub.publish(twist_msg)
        return True
    except:
        print(f"Chassis axis failed: {axis_values}")
        return False

humerus_last_pos = 0.0
forearm_last_pos = 0.0

# ///////////////////////////////////////////////////////////////
# ARM
# def arm_command(event)
# def arm_axis_command(axis_values)

def full_stop():
    print("Full stop")
    axis_stop_values = [0.0] * joystick.get_numaxes()
    
    chassis_command('Stop')
    #arm_axis_command(axis_stop_values)
    
    sleep(5)

def handle_event(event):
    global modes
    global mode_idx
    if event == 'B':
        full_stop()
        return
    elif event == 'LB':
        change_mode()
        return True
    
    if modes[mode_idx] == "Chassis":
        chassis_command(event)
    elif modes[mode_idx] == "Arm":
        #arm_command(event)
        print(event)
    elif event == 'default':
        print(event)

def handle_axis(axis_values):
    if modes[mode_idx] == "Chassis":
        chassis_axis_command(axis_values)
    elif modes[mode_idx] == "Arm":
        #arm_axis_command(axis_values)
        print("Arm mode active!")

try:
    while True:
        # Procesar eventos para mantener acutalizado el sistema interno
        for event in pygame.event.get():
            # Eventos 
            if event.type == pygame.JOYBUTTONDOWN:
                name = button_names.get(event.button, f"Button {event.button}")
                print(f"[EVENT] {name} pressed")
                handle_event(button_names.get(event.button, 'default'))
            elif event.type == pygame.JOYBUTTONUP:
                name = button_names.get(event.button, f"Button {event.button}")
                print(f"[EVENT] {name} released")
            elif event.type == pygame.JOYHATMOTION:
                if event.value != (0, 0):
                    print(f"[EVENT] D-Pad: {event.value}")
            elif event.type == pygame.JOYAXISMOTION:
                val = event.value
                if abs(val) > DRIFT:
                    print(f"[EVENT] Axis {event.axis} moved: {val:.2f}")
                    
        # Botones (solo si cambian)
        for i in range(joystick.get_numbuttons()):
            actual = bool(joystick.get_button(i))
            if actual != buttons_prev_states[i]:
                name = button_names.get(i, f"Botón {i}")
                state = "pressed" if actual else "released"
                print(f"[POLL] {name} {state}")
                buttons_prev_states[i] = actual

        # Ejes (con deadzone y cambio significativo)
        axis_nums = joystick.get_numaxes()
        for i in range(axis_nums):
            val = joystick.get_axis(i)
            if abs(val) < DRIFT:
                val = 0.0  # estabilizar en zona muerta
            # Solo reportar si cambio mayor a umbral pequeño para evitar ruido
            if abs(val - axis_prev[i]) > THRESHOLD:
                print(f"[POLL] Eje {i} movido: {val:.2f}")
                axis_prev[i] = val
                handle_axis(axis_prev)

        # Hat / D-Pad
        '''
        if joystick.get_numhats() > 0:
            hat = joystick.get_hat(0)
            if hat != hat_prev:
                if hat != (0, 0):
                    print(f"[POLL] D-Pad: {hat}")
                else:
                    print("[POLL] D-Pad centrado")
                hat_prev = hat '''

        # Pequeña pausa para no saturar CPU
        sleep(DELAY)

except KeyboardInterrupt:
    print("\nExiting...")

finally:
    pygame.quit()