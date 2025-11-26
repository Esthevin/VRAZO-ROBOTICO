#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Float32, Bool
import math
import threading

GRIPPER_ABIERTO = (110-180)*math.pi/180
GRIPPER_CERRADO = (132-180)*math.pi/180
GRIPPER_HOME = 0*math.pi/180

class Trayectorias(Node):

    def __init__(self):
        super().__init__("trayectorias_p")

        # Publicadores
        self.pub_pos = self.create_publisher(Point, "cmd_xyz_position", 10)
        self.pub_gripper = self.create_publisher(Float32, "cmd_gripper", 10)

        # Subscriber para confirmación
        self.objetivo_alcanzado = False
        self.objetivo_lock = threading.Lock()
        
        self.create_subscription(
            Bool, 
            "objetivo_alcanzado", 
            self.objetivo_callback, 
            10
        )

        # Thread para spin
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        self.spin_thread.start()

        self.get_logger().info("=== Nodo de Trayectorias Inicializado ===")

    def objetivo_callback(self, msg: Bool):
        """Callback que recibe confirmación cuando se alcanza el objetivo"""
        with self.objetivo_lock:
            if msg.data:
                self.objetivo_alcanzado = True
                # Solo log si cambió de estado
                if not hasattr(self, '_last_estado') or not self._last_estado:
                    self.get_logger().info("✅ Confirmación recibida")
                self._last_estado = True
            else:
                self._last_estado = False

    def esperar_objetivo(self, timeout=10.0):
        """Espera hasta que el objetivo sea alcanzado o timeout"""
        tiempo_inicio = self.get_clock().now()
        rate = self.create_rate(10)  # Verificar a 10Hz
        
        with self.objetivo_lock:
            self.objetivo_alcanzado = False
        
        self.get_logger().info(f"⏳ Esperando confirmación (max {timeout}s)...")
        
        contador = 0
        while rclpy.ok():
            with self.objetivo_lock:
                if self.objetivo_alcanzado:
                    self.get_logger().info("✓ Movimiento completado")
                    return True
            
            # Verificar timeout
            tiempo_actual = self.get_clock().now()
            tiempo_transcurrido = (tiempo_actual - tiempo_inicio).nanoseconds / 1e9
            
            # Log de progreso cada 2 segundos
            if contador % 20 == 0 and contador > 0:
                self.get_logger().info(f"⏱️ Esperando... {tiempo_transcurrido:.1f}s / {timeout}s")
            
            if tiempo_transcurrido > timeout:
                self.get_logger().error(f"❌ TIMEOUT ({timeout}s) - El robot no confirmó el objetivo")
                self.get_logger().error("   Posibles causas:")
                self.get_logger().error("   1. El nodo de control no está corriendo")
                self.get_logger().error("   2. Las tolerancias son muy estrictas")
                self.get_logger().error("   3. La posición objetivo es inalcanzable")
                return False
            
            contador += 1
            rate.sleep()
        
        return False

    def mover_a(self, x, y, z, timeout=15.0):
        """
        Mueve el robot a una posición y espera confirmación
        
        Args:
            x, y, z: Coordenadas destino
            timeout: Tiempo máximo de espera en segundos
        
        Returns:
            True si alcanzó el objetivo, False si timeout
        """
        msg = Point()
        msg.x, msg.y, msg.z = x, y, z
        self.pub_pos.publish(msg)
        self.get_logger().info(f"📍 Comando: Mover a [{x:.3f}, {y:.3f}, {z:.3f}]")
        
        return self.esperar_objetivo(timeout)

    def set_gripper(self, angle, timeout=8.0):
        """
        Ajusta el gripper y espera confirmación
        
        Args:
            angle: Ángulo objetivo en radianes
            timeout: Tiempo máximo de espera
        
        Returns:
            True si alcanzó el objetivo, False si timeout
        """
        msg = Float32()
        msg.data = float(angle)
        self.pub_gripper.publish(msg)
        
        estado = "ABIERTO" if angle < GRIPPER_CERRADO else "CERRADO"
        self.get_logger().info(f"✋ Comando: Gripper → {estado} ({math.degrees(angle):.1f}°)")
        
        return self.esperar_objetivo(timeout)


def main():
    rclpy.init()
    node = Trayectorias()

    # Definir posiciones clave
    HOME = (-0.87, -0.1014, 2.9964)
    OBJETO = (-1.0, -2.0, 1.5)
    DESTINO = (1.5, -2.0, 1.5)
    PRE_AGARRE = (-1.0, -2.0, 2.0)  # Posición sobre el objeto
    PRE_DESTINO = (1.5, -2.0, 2.0)  # Posición sobre destino

    node.get_logger().info("🚀 Iniciando secuencia de pick and place...")
    
    try:
        # 1. Ir a HOME
        node.get_logger().info("--- PASO 1: Ir a HOME ---")
        if not node.mover_a(*HOME):
            node.get_logger().error("❌ Fallo al ir a HOME")
            return
        
        # 2. Abrir gripper
        node.get_logger().info("--- PASO 2: Abrir Gripper ---")
        if not node.set_gripper(GRIPPER_ABIERTO):
            node.get_logger().error("❌ Fallo al abrir gripper")
            return
        
        # 3. Moverse sobre el objeto
        node.get_logger().info("--- PASO 3: Aproximación al objeto ---")
        if not node.mover_a(*PRE_AGARRE):
            node.get_logger().error("❌ Fallo en aproximación")
            return
        
        # 4. Bajar al objeto
        node.get_logger().info("--- PASO 4: Descender al objeto ---")
        if not node.mover_a(*OBJETO):
            node.get_logger().error("❌ Fallo al descender")
            return
        
        # 5. Cerrar gripper (agarrar)
        node.get_logger().info("--- PASO 5: Agarrar objeto ---")
        if not node.set_gripper(GRIPPER_CERRADO):
            node.get_logger().error("❌ Fallo al cerrar gripper")
            return
        
        # 6. Levantar objeto
        node.get_logger().info("--- PASO 6: Levantar objeto ---")
        if not node.mover_a(*PRE_AGARRE):
            node.get_logger().error("❌ Fallo al levantar")
            return
        
        # 7. Moverse sobre destino
        node.get_logger().info("--- PASO 7: Transportar al destino ---")
        if not node.mover_a(*PRE_DESTINO):
            node.get_logger().error("❌ Fallo en transporte")
            return
        
        # 8. Bajar al destino
        node.get_logger().info("--- PASO 8: Descender al destino ---")
        if not node.mover_a(*DESTINO):
            node.get_logger().error("❌ Fallo al descender")
            return
        
        # 9. Abrir gripper (soltar)
        node.get_logger().info("--- PASO 9: Soltar objeto ---")
        if not node.set_gripper(GRIPPER_ABIERTO):
            node.get_logger().error("❌ Fallo al soltar")
            return
        
        # 10. Levantar
        node.get_logger().info("--- PASO 10: Retirarse ---")
        if not node.mover_a(PRE_DESTINO[0]-1.0, PRE_DESTINO[1]+1.5, PRE_DESTINO[2]-0.5):
            node.get_logger().error("❌ Fallo al retirarse")
            return
        
        # 11. Volver a HOME
        node.get_logger().info("--- PASO 11: Regresar a HOME ---")
        if not node.mover_a(*HOME):
            node.get_logger().error("❌ Fallo al regresar")
            return
        node.set_gripper(GRIPPER_HOME)


        node.get_logger().info("=" * 50)
        node.get_logger().info("🎉 ¡SECUENCIA COMPLETADA EXITOSAMENTE! 🎉")
        node.get_logger().info("=" * 50)

    except KeyboardInterrupt:
        node.get_logger().info("⚠️ Secuencia interrumpida por usuario")
    except Exception as e:
        node.get_logger().error(f"❌ Error inesperado: {str(e)}")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
