#!/usr/bin/env python3
import rclpy
import threading
import numpy as np
import math
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Float32, Int32MultiArray, Bool
from copy import copy

# ============= FUNCIONES DE CINEMÁTICA =============
cos = np.cos
sin = np.sin
pi = np.pi

def dh(d, theta, a, alpha):
    sth = np.sin(theta)
    cth = np.cos(theta)
    sa = np.sin(alpha)
    ca = np.cos(alpha)

    T = np.array([
        [cth, -ca*sth, sa*sth, a*cth],
        [sth, ca*cth, -sa*cth, a*sth],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0]
    ])
    return T


def fkine_ur5(q):
    l1v = 0.6975
    l2v = 1.0074
    l3v = 1.2915
    l4v = 0
    l5v = 0.745

    T1 = dh(l1v, q[0], -0.125, -pi/2)
    T2 = dh(0, q[1]-pi/2, l2v, 0)
    T3 = dh(0, q[2], l3v, 0)
    T4 = dh(-0.1014, q[3], 0, pi/2)
    T5 = dh(l5v, q[4], 0, 0)

    T = T1 @ T2 @ T3 @ T4 @ T5
    return T


def jacobian_position(q, delta=0.0001):
    n = 5
    J = np.zeros((3, n))
    x = fkine_ur5(q)

    for i in range(n):
        dq = copy(q)
        dq[i] += delta
        dx = fkine_ur5(dq)
        J[:, i] = (dx[0:3, 3] - x[0:3, 3]) / delta

    return J


def rot2quat(R):
    dEpsilon = 1e-6
    quat = 4 * [0., ]
    quat[0] = 0.5*np.sqrt(R[0,0]+R[1,1]+R[2,2]+1.0)

    quat[1] = 0.5*np.sign(R[2,1]-R[1,2])*np.sqrt(max(R[0,0]-R[1,1]-R[2,2]+1.0, 0))
    quat[2] = 0.5*np.sign(R[0,2]-R[2,0])*np.sqrt(max(R[1,1]-R[2,2]-R[0,0]+1.0, 0))
    quat[3] = 0.5*np.sign(R[1,0]-R[0,1])*np.sqrt(max(R[2,2]-R[0,0]-R[1,1]+1.0, 0))

    return np.array(quat)


def TF2xyzquat(T):
    quat = rot2quat(T[0:3, 0:3])
    return np.array([T[0,3], T[1,3], T[2,3], quat[0], quat[1], quat[2], quat[3]])

def rad_to_servo(radians, min_rad=-math.pi/2, max_rad=math.pi/2, min_servo=0, max_servo=180):
    """Convertir radianes a ángulos de servo (0-180 grados)"""
    normalized = np.clip(radians, min_rad, max_rad)
    servo_angle = min_servo + (normalized - min_rad) * (max_servo - min_servo) / (max_rad - min_rad)
    return int(np.clip(servo_angle, min_servo, max_servo))


# ============= NODO PRINCIPAL =============
def main():
    rclpy.init()
    node = rclpy.create_node('control_cinematico')

    # Publicadores
    pub_js = node.create_publisher(JointState, 'joint_states', 10)
    pub_ee = node.create_publisher(PoseStamped, 'ee_pose', 10)
    pub_esp32 = node.create_publisher(Int32MultiArray, 'servo_commands', 10)
    
    # NUEVO: Publicador de estado (si alcanzó objetivo)
    pub_objetivo_alcanzado = node.create_publisher(Bool, 'objetivo_alcanzado', 10)

    # Objetivo de posición
    xd = np.array([-0.87, -0.1014, 2.9964])
    xd_lock = threading.Lock()

    # Ángulo del gripper
    gripper_angle = [0.0]
    gripper_target = [0.0]
    gripper_lock = threading.Lock()
    
    # Variables para control de envío al ESP32
    last_sent_q = np.array([0.0] * 6)
    last_send_time = node.get_clock().now().nanoseconds / 1e9
    send_threshold = math.radians(2.0)
    min_send_interval = 0.1

    # Tolerancias (ajustadas para ser más realistas)
    POSICION_TOLERANCIA = 0.05  # 5cm de tolerancia para posición
    GRIPPER_TOLERANCIA = math.radians(10.0)  # 10 grados para gripper
    
    # Estado del objetivo
    objetivo_alcanzado = [False]

    def cmd_xyz_callback(msg: Point):
        nonlocal xd
        with xd_lock:
            xd = np.array([msg.x, msg.y, msg.z])
            objetivo_alcanzado[0] = False  # Nuevo objetivo = no alcanzado
        node.get_logger().info(f"📍 Nuevo objetivo: {xd}")

    def gripper_callback(msg: Float32):
        with gripper_lock:
            gripper_target[0] = float(msg.data)
            objetivo_alcanzado[0] = False  # Nuevo comando gripper = no alcanzado
        node.get_logger().info(f"🟢 Gripper objetivo: {msg.data:.3f} rad")

    # Subscribers
    node.create_subscription(Point, 'cmd_xyz_position', cmd_xyz_callback, 10)
    node.create_subscription(Float32, 'cmd_gripper', gripper_callback, 10)

    # Thread para spin
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # Joint names
    jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

    # Configuración inicial
    q = np.zeros(6)

    # Límites
    limits = [
        (0, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (0, math.pi),
        (-math.pi, math.pi)
    ]

    K = 0.5
    freq = 50
    dt = 1.0 / freq
    rate = node.create_rate(freq)
    
    # Contadores
    msg_count = 0
    esp32_sent_count = 0

    node.get_logger().info("=== Control Cinemático Iniciado ===")
    node.get_logger().info("Publicando estado en /objetivo_alcanzado")

    while rclpy.ok():
        now = node.get_clock().now().to_msg()
        current_time = node.get_clock().now().nanoseconds / 1e9
        msg_count += 1

        # Obtener objetivo seguro
        with xd_lock:
            xd_current = xd.copy()
        
        with gripper_lock:
            gripper_target_current = gripper_target[0]

        # Posición actual
        T = fkine_ur5(q)
        x = T[0:3, 3]

        # Error de posición
        e = xd_current - x
        error_posicion = np.linalg.norm(e)

        # Control de posición (articulaciones 0-4)
        if error_posicion > POSICION_TOLERANCIA:
            J = jacobian_position(q)
            J_pinv = J.T @ np.linalg.inv(J @ J.T)
            q_dot = K * (J_pinv @ e)
            q[:5] += q_dot * dt
            posicion_ok = False
        else:
            q_dot = np.zeros(5)
            posicion_ok = True
            if msg_count % 50 == 0:  # Log cada segundo
                node.get_logger().info(f"🎯 Posición OK: error={error_posicion:.4f}m")

        # Aplicar límites
        for i in range(5):
            q[i] = np.clip(q[i], limits[i][0], limits[i][1])

        # Control de Gripper (articulación 5) con velocidad limitada
        error_gripper = gripper_target_current - gripper_angle[0]
        
        if abs(error_gripper) > GRIPPER_TOLERANCIA:
            # Velocidad máxima del gripper (rad/s)
            max_gripper_speed = math.radians(90)  # 90 grados/segundo
            gripper_speed = np.clip(error_gripper * 2.0, -max_gripper_speed, max_gripper_speed)
            gripper_angle[0] += gripper_speed * dt
            gripper_angle[0] = np.clip(gripper_angle[0], limits[5][0], limits[5][1])
            gripper_ok = False
        else:
            gripper_angle[0] = gripper_target_current
            gripper_ok = True
            if msg_count % 50 == 0:  # Log cada segundo
                node.get_logger().info(f"✋ Gripper OK: {math.degrees(gripper_angle[0]):.1f}°")

        q[5] = gripper_angle[0]

        # Verificar si alcanzó el objetivo completo
        objetivo_completo = posicion_ok and gripper_ok
        
        # Publicar estado continuamente
        if msg_count % 10 == 0:  # Cada 200ms
            msg_alcanzado = Bool()
            msg_alcanzado.data = objetivo_completo
            pub_objetivo_alcanzado.publish(msg_alcanzado)
        
        if objetivo_completo and not objetivo_alcanzado[0]:
            objetivo_alcanzado[0] = True
            node.get_logger().info(f"✅ OBJETIVO ALCANZADO - Pos: [{x[0]:.3f}, {x[1]:.3f}, {x[2]:.3f}], Gripper: {math.degrees(gripper_angle[0]):.1f}°")
        elif not objetivo_completo and objetivo_alcanzado[0]:
            objetivo_alcanzado[0] = False
            if msg_count % 50 == 0:
                node.get_logger().info(f"🔄 Ajustando - Error pos: {error_posicion:.4f}m, gripper: {math.degrees(abs(error_gripper)):.1f}°")

        # ========== PUBLICAR PARA ESP32 ==========
        should_send = False
        
        if (current_time - last_send_time) >= min_send_interval:
            max_change = 0.0
            for i, (qi, last_qi) in enumerate(zip(q, last_sent_q)):
                change = abs(qi - last_qi)
                if change > max_change:
                    max_change = change
                if change > send_threshold:
                    should_send = True
                    break
        
        if should_send:
            q_degrees = [int(round(math.degrees(qi))) for qi in q]
            esp32_msg = Int32MultiArray()
            esp32_msg.data = q_degrees
            pub_esp32.publish(esp32_msg)
            last_sent_q = q.copy()
            last_send_time = current_time
            esp32_sent_count += 1

        # Publicar joint_states
        js = JointState()
        js.header.stamp = now
        js.name = jnames
        js.position = q.tolist()
        pub_js.publish(js)

        # Publicar pose del efector
        pose_vec = TF2xyzquat(T)
        ps = PoseStamped()
        ps.header.stamp = now
        ps.header.frame_id = "base_link"
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = pose_vec[:3]
        ps.pose.orientation.w, ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z = pose_vec[3:]
        pub_ee.publish(ps)
        
        # Estadísticas
        if msg_count % 500 == 0:
            efficiency = (esp32_sent_count / msg_count) * 100
            node.get_logger().info(
                f"📊 Stats: {esp32_sent_count}/{msg_count} msg ESP32 "
                f"({efficiency:.1f}%) | Error: {error_posicion:.4f}m"
            )

        rate.sleep()


if __name__ == '__main__':
    main()
