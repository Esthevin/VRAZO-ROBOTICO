#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <ESP32Servo.h>

// ---------------- Configuración ----------------
#define N_SERVOS 6
// Pines SEGUROS en ESP32 (evite 6-11, 0, 2, 12, 15):
const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int SERVO_MIN_US = 500;   // ajuste si su servo requiere distinto
const int SERVO_MAX_US = 2500;

// Posición HOME (grados para cada servo)
const int SERVO_HOME[N_SERVOS] = {0, 90, 90, 90, 0, 0};

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

// --------------- Posición actual relativa al HOME ---------------
int current_relative_pos[N_SERVOS] = {0, 0, 0, 0, 0, 0}; // Inicialmente en HOME (0 relativo)

// --------------- Utilidades --------------------
void error_loop() {
  pinMode(2, OUTPUT); 
  while (1) {
    digitalWrite(2, !digitalRead(2));
    delay(300);
  }
}

void move_to_home() {
  for (int i = 0; i < N_SERVOS; i++) {
    int angle = SERVO_HOME[i];
    if (angle < 0)   angle = 0;
    if (angle > 180) angle = 180;
    servos[i].write(angle);
    current_relative_pos[i] = 0; // Resetear posición relativa a 0
    delay(200); // pequeño retardo para que no vayan todos a la vez brusco
  }
}

void move_relative_to_home(const int relative_angles[N_SERVOS]) {
  for (int i = 0; i < N_SERVOS; i++) {
    // Calcular posición absoluta: HOME + offset relativo
    int absolute_angle = SERVO_HOME[i] + relative_angles[i];
    
    // Limitar el ángulo a los límites físicos del servo (0-180)
    if (absolute_angle < 0) absolute_angle = 0;
    if (absolute_angle > 180) absolute_angle = 180;
    
    // Mover el servo
    servos[i].write(absolute_angle);
    
    // Actualizar la posición relativa actual
    current_relative_pos[i] = relative_angles[i];
  }
}

void subscription_callback(const void * msgin)
{
  const std_msgs__msg__Int32MultiArray * m =
    (const std_msgs__msg__Int32MultiArray *) msgin;

  size_t n = m->data.size;
  if (n > N_SERVOS) n = N_SERVOS;

  // Crear array temporal para los ángulos relativos
  int relative_angles[N_SERVOS];
  
  // Copiar los datos recibidos (son relativos al HOME)
  for (size_t i = 0; i < n; i++) {
    relative_angles[i] = (int)m->data.data[i];
  }
  
  // Si se recibieron menos de 6 valores, mantener los actuales para los restantes
  for (size_t i = n; i < N_SERVOS; i++) {
    relative_angles[i] = current_relative_pos[i];
  }
  
  // Mover los servos respecto al HOME
  move_relative_to_home(relative_angles);
}

void setup() {
  set_microros_transports();

  std_msgs__msg__Int32MultiArray__init(&msg);
  msg.layout.dim.capacity = 0;
  msg.layout.dim.size = 0;
  msg.layout.data_offset = 0;
  msg.data.data = data_buffer;
  msg.data.capacity = N_SERVOS;
  msg.data.size = 0;

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  // Inicializar servos
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  }

  // Mover a posición HOME
  move_to_home();

  // Suscriptor a /servo_commands (arreglo de 6 enteros - RELATIVOS AL HOME)
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_commands"));

  // Executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg,
                                         &subscription_callback, ON_NEW_DATA));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(5);
}
