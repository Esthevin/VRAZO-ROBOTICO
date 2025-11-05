#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <ESP32Servo.h>

// ================ CONFIGURACIÓN ================
#define N_SERVOS 6
// Pines SEGUROS en ESP32 (evite 6-11, 0, 2, 12, 15):
const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int SERVO_MIN_US = 500;   // ajuste si su servo requiere distinto
const int SERVO_MAX_US = 2500;

// Posición HOME (grados para cada servo)
const int SERVO_HOME[N_SERVOS] = {0, 90, 90, 90, 0, 0};

// ========== OPTIMIZACIONES ==========
// Velocidad de interpolación (grados por ciclo)
const float SERVO_SPEED = 3.0;  // Ajustar según necesidad (más alto = más rápido)

// Intervalo de movimiento en ms
const int MOVE_INTERVAL = 20;  // 50 Hz de actualización de servos

// Debug por Serial
#define DEBUG_SERIAL true  // Cambiar a false para desactivar debug
// ====================================

// --------------- Infra micro-ROS ---------------
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Buffer estático para deserializar el multiarray (OBLIGATORIO en micro-ROS)
static int32_t data_buffer[N_SERVOS];

// Macros de chequeo
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

// --------------- Servos ------------------------
Servo servos[N_SERVOS];

// --------------- Posición actual y objetivo ---------------
int current_relative_pos[N_SERVOS] = {0, 0, 0, 0, 0, 0}; // Posición actual relativa
int target_relative_pos[N_SERVOS] = {0, 0, 0, 0, 0, 0};  // Posición objetivo relativa

// Control de tiempo
unsigned long last_move_time = 0;
unsigned long msg_received_count = 0;
unsigned long last_stats_time = 0;

// --------------- Utilidades --------------------
void error_loop() {
  pinMode(2, OUTPUT); 
  while (1) {
    digitalWrite(2, !digitalRead(2));
    delay(300);
  }
}

void move_to_home() {
  if (DEBUG_SERIAL) {
    Serial.println("=== Moviendo a HOME ===");
  }
  for (int i = 0; i < N_SERVOS; i++) {
    int angle = SERVO_HOME[i];
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    servos[i].write(angle);
    current_relative_pos[i] = 0;
    target_relative_pos[i] = 0;
    delay(100);
  }
  if (DEBUG_SERIAL) {
    Serial.println("HOME alcanzado");
  }
}

void update_servo_positions() {
  bool any_moving = false;
  
  for (int i = 0; i < N_SERVOS; i++) {
    int current = current_relative_pos[i];
    int target = target_relative_pos[i];
    
    // Si no está en el objetivo, mover gradualmente
    if (current != target) {
      any_moving = true;
      int diff = target - current;
      int step = SERVO_SPEED;
      
      // Ajustar el paso si la diferencia es menor que la velocidad
      if (abs(diff) < step) {
        step = abs(diff);
      }
      
      // Mover en la dirección correcta
      if (diff > 0) {
        current += step;
      } else {
        current -= step;
      }
      
      // Calcular ángulo absoluto
      int absolute_angle = SERVO_HOME[i] + current;
      
      // Limitar a rango físico del servo
      if (absolute_angle < 0) absolute_angle = 0;
      if (absolute_angle > 180) absolute_angle = 180;
      
      // Mover el servo
      servos[i].write(absolute_angle);
      current_relative_pos[i] = current;
    }
  }
  
  // Debug opcional cuando termina el movimiento
  if (DEBUG_SERIAL && !any_moving && (millis() - last_stats_time > 2000)) {
    Serial.print("Posición actual (relativa): [");
    for (int i = 0; i < N_SERVOS; i++) {
      Serial.print(current_relative_pos[i]);
      if (i < N_SERVOS - 1) Serial.print(", ");
    }
    Serial.println("]");
  }
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32MultiArray * m =
    (const std_msgs__msg__Int32MultiArray *) msgin;

  msg_received_count++;
  
  size_t n = m->data.size;
  if (n > N_SERVOS) n = N_SERVOS;

  // Debug: mostrar mensaje recibido
  if (DEBUG_SERIAL) {
    Serial.print("Msg #");
    Serial.print(msg_received_count);
    Serial.print(" recibido: [");
    for (size_t i = 0; i < n; i++) {
      Serial.print(m->data.data[i]);
      if (i < n - 1) Serial.print(", ");
    }
    Serial.println("]");
  }
  
  // Actualizar objetivos (valores relativos al HOME)
  for (size_t i = 0; i < n; i++) {
    target_relative_pos[i] = (int)m->data.data[i];
  }
  
  // Si se recibieron menos de 6 valores, mantener los objetivos actuales
  for (size_t i = n; i < N_SERVOS; i++) {
    // No hacer nada, mantener el objetivo anterior
  }
}

void print_stats() {
  if (millis() - last_stats_time > 5000) {  // Cada 5 segundos
    Serial.print("Mensajes recibidos: ");
    Serial.print(msg_received_count);
    Serial.print(" | Posición: [");
    for (int i = 0; i < N_SERVOS; i++) {
      Serial.print(current_relative_pos[i]);
      if (i < N_SERVOS - 1) Serial.print(", ");
    }
    Serial.print("] | Target: [");
    for (int i = 0; i < N_SERVOS; i++) {
      Serial.print(target_relative_pos[i]);
      if (i < N_SERVOS - 1) Serial.print(", ");
    }
    Serial.println("]");
    
    last_stats_time = millis();
    msg_received_count = 0;
  }
}

void setup() {
  // Inicializar Serial para debug
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=================================");
    Serial.println("ESP32 Servo Controller OPTIMIZADO");
    Serial.println("=================================");
  }

  // Configurar transporte micro-ROS
  set_microros_transports();

  // Inicializar mensaje
  std_msgs__msg__Int32MultiArray__init(&msg);
  msg.layout.dim.capacity = 0;
  msg.layout.dim.size = 0;
  msg.layout.data_offset = 0;
  msg.data.data = data_buffer;
  msg.data.capacity = N_SERVOS;
  msg.data.size = 0;

  allocator = rcl_get_default_allocator();

  // Inicializar soporte y nodo
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  // Inicializar servos
  if (DEBUG_SERIAL) {
    Serial.println("Inicializando servos...");
  }
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
    if (DEBUG_SERIAL) {
      Serial.print("  Servo ");
      Serial.print(i);
      Serial.print(" en pin ");
      Serial.println(SERVO_PINS[i]);
    }
  }

  // Mover a posición HOME
  move_to_home();

  // Suscriptor a /servo_commands
  if (DEBUG_SERIAL) {
    Serial.println("Suscribiendo a /servo_commands...");
  }
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_commands"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         &subscription_callback, ON_NEW_DATA));

  if (DEBUG_SERIAL) {
    Serial.println("Setup completo. Esperando comandos...");
    Serial.println("=================================\n");
  }
  
  last_move_time = millis();
  last_stats_time = millis();
}

void loop() {
  // Procesar mensajes de micro-ROS (reducido a 10ms para mejor respuesta)
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  
  // Actualizar posiciones de servos a intervalo fijo
  unsigned long current_time = millis();
  if (current_time - last_move_time >= MOVE_INTERVAL) {
    update_servo_positions();
    last_move_time = current_time;
  }
  
  // Imprimir estadísticas periódicamente
  if (DEBUG_SERIAL) {
    print_stats();
  }
  
  delay(2);  // Pequeño delay para estabilidad
}
