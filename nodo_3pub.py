#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import time

# Configuración de los servos
SERVO_TOPICS = ['servo1_cmd', 'servo2_cmd', 'servo3_cmd']

# Publishers (se crean después)
publishers = []

def main():
    # Inicializar ROS2
    rclpy.init()
    
    # Crear nodo
    node = rclpy.create_node('servo_controller')
    
    # Crear publishers para cada servo
    for i, topic in enumerate(SERVO_TOPICS):
        pub = node.create_publisher(Int32, topic, 10)
        publishers.append(pub)
        print(f'Publisher creado para: {topic}')
    
    try:
        # Ejemplo 1: Mover todos los servos a posiciones específicas
        print("Moviendo servos a posición inicial...")
        move_servo(node, 0, 90)   # Servo 1 a 90°
        move_servo(node, 1, 45)   # Servo 2 a 45°
        move_servo(node, 2, 135)  # Servo 3 a 135°
        time.sleep(2)
        
        # Ejemplo 2: Barrido completo de un servo
        print("Barrido del servo 1...")
        sweep_servo(node, 0)
        
        # Ejemplo 3: Movimiento coordinado
        print("Movimiento coordinado...")
        coordinated_movement(node)
        
        # Ejemplo 4: Control por consola
        print("Control manual - ingresa ángulos:")
        manual_control(node)
        
    except KeyboardInterrupt:
        print("Apagando servos...")
        # Mover todos a posición neutral
        for i in range(len(SERVO_TOPICS)):
            move_servo(node, i, 90)
        time.sleep(1)
    
    finally:
        # Limpiar
        node.destroy_node()
        rclpy.shutdown()

def move_servo(node, servo_index, angle):
    """Mueve un servo específico a un ángulo"""
    if servo_index < len(publishers):
        # Crear mensaje
        msg = Int32()
        msg.data = angle
        
        # Publicar
        publishers[servo_index].publish(msg)
        
        print(f'Servo {servo_index + 1} → {angle}°')
        time.sleep(0.1)  # Pequeña pausa entre movimientos

def sweep_servo(node, servo_index):
    """Realiza un barrido completo de 0° a 180°"""
    # De 0° a 180°
    for angle in range(0, 181, 10):
        move_servo(node, servo_index, angle)
        time.sleep(0.2)
    
    # De 180° a 0°
    for angle in range(180, -1, -10):
        move_servo(node, servo_index, angle)
        time.sleep(0.2)

def coordinated_movement(node):
    """Movimiento coordinado de los 3 servos"""
    positions = [
        [0, 90, 180],    # Posición 1
        [90, 45, 90],    # Posición 2  
        [180, 135, 0],   # Posición 3
        [45, 180, 45],   # Posición 4
    ]
    
    for pos in positions:
        for i, angle in enumerate(pos):
            move_servo(node, i, angle)
        time.sleep(1)  # Esperar entre posiciones

def manual_control(node):
    """Control manual desde consola"""
    try:
        while True:
            print("\n--- Control Manual ---")
            print("Formato: servo,ángulo (ej: 1,90)")
            print("Servos: 1, 2, 3 | Ángulo: 0-180")
            print("'q' para salir")
            
            user_input = input("Ingresa comando: ").strip()
            
            if user_input.lower() == 'q':
                break
                
            try:
                # Procesar entrada
                parts = user_input.split(',')
                if len(parts) == 2:
                    servo_num = int(parts[0]) - 1  # Convertir a índice (0-based)
                    angle = int(parts[1])
                    
                    if 0 <= servo_num < len(SERVO_TOPICS) and 0 <= angle <= 180:
                        move_servo(node, servo_num, angle)
                    else:
                        print("Error: Servo debe ser 1-3, ángulo 0-180")
                else:
                    print("Error: Usa formato 'servo,ángulo'")
                    
            except ValueError:
                print("Error: Ingresa números válidos")
                
    except KeyboardInterrupt:
        print("\nSaliendo del control manual...")

def sequence_demo(node):
    """Demostración de secuencia predefinida"""
    sequences = [
        # (servo_index, ángulo, delay)
        (0, 0, 0.5),
        (1, 0, 0.5), 
        (2, 0, 0.5),
        (0, 90, 0.5),
        (1, 90, 0.5),
        (2, 90, 0.5),
        (0, 180, 0.5),
        (1, 180, 0.5),
        (2, 180, 0.5),
        (0, 90, 1.0),
        (1, 90, 1.0),
        (2, 90, 1.0),
    ]
    
    print("Ejecutando secuencia demo...")
    for servo_idx, angle, delay in sequences:
        move_servo(node, servo_idx, angle)
        time.sleep(delay)

if __name__ == '__main__':
    main()
