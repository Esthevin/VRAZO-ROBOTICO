#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/bool.h>
#include <ESP32Servo.h>

// ================= CONFIGURACIÓN =================
#define N_SERVOS 6
#define NUM_ANGULOS 5   // cantidad que viene por UART

const int SERVO_PINS[N_SERVOS] = {4, 16, 17, 5, 18, 19};
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_HOME[N_SERVOS] = {0, 90, 90, 90, 0, 180};

const float SERVO_SPEED = 3.0;
const int MOVE_INTERVAL = 20;
#define DEBUG_SERIAL true

// UART desde Arduino (usar Serial2, RX=16, TX=17 o ajustar)
#define UART_RX_PIN 22
#define UART_TX_PIN 23
#define UART_BAUDRATE 115200

// ================= SENSOR DE PRESENCIA =================
#define SENSOR_PRESENCIA_PIN 36
#define DEBOUNCE_TIME 10  // ms para evitar rebotes
bool presencia_detectada = false;
bool ultimo_estado_sensor = false;
unsigned long ultimo_cambio = 0;

// ================= micro-ROS =================
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t publisher_presencia;  // Nuevo publisher para el sensor
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// mensajes
std_msgs__msg__Int32MultiArray sub_msg;
std_msgs__msg__Int32MultiArray pub_msg;
std_msgs__msg__Bool msg_presencia;  // Nuevo mensaje para el sensor

static int32_t sub_data_buffer[N_SERVOS];
static int32_t pub_data_buffer[NUM_ANGULOS];

// ================= Servos =================
Servo servos[N_SERVOS];
int current_relative_pos[N_SERVOS] = {0};
int target_relative_pos[N_SERVOS] = {0};

// ================= Utilidades =================
#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  pinMode(2, OUTPUT);
  while (1) {
    digitalWrite(2, !digitalRead(2));
    delay(300);
  }
}

void move_to_home() {
  if (DEBUG_SERIAL) Serial.println("Moviendo a HOME...");
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].write(SERVO_HOME[i]);
    current_relative_pos[i] = 0;
    target_relative_pos[i] = 0;
    delay(100);
  }
}

void update_servo_positions() {
  for (int i = 0; i < N_SERVOS; i++) {
    int current = current_relative_pos[i];
    int target = target_relative_pos[i];

    if (current != target) {
      int diff = target - current;
      int step = SERVO_SPEED;
      if (abs(diff) < step) step = abs(diff);
      current += (diff > 0) ? step : -step;

      int absolute_angle = SERVO_HOME[i] + current;
      if (absolute_angle < 0) absolute_angle = 0;
      if (absolute_angle > 180) absolute_angle = 180;

      servos[i].write(absolute_angle);
      current_relative_pos[i] = current;
    }
  }
}

// ================= Lectura Sensor de Presencia =================
void leer_sensor_presencia() {
  bool estado_actual = (digitalRead(SENSOR_PRESENCIA_PIN) == HIGH);
  
  // Debounce
  if (estado_actual != ultimo_estado_sensor) {
    ultimo_cambio = millis();
  }
  
  if ((millis() - ultimo_cambio) > DEBOUNCE_TIME) {
    if (estado_actual != presencia_detectada) {
      presencia_detectada = estado_actual;
      
      if (DEBUG_SERIAL) {
        if (presencia_detectada) {
          Serial.println("🔴 PRESENCIA DETECTADA - Activando brazo robótico");
        } else {
          Serial.println("✅ Presencia desaparecida");
        }
      }
    }
  }
  
  ultimo_estado_sensor = estado_actual;
}

// ================= CALLBACK micro-ROS =================
void subscription_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * m = (const std_msgs__msg__Int32MultiArray *) msgin;
  size_t n = m->data.size;
  if (n > N_SERVOS) n = N_SERVOS;
  for (size_t i = 0; i < n; i++) {
    target_relative_pos[i] = (int)m->data.data[i];
  }

  if (DEBUG_SERIAL) {
    Serial.print("Comando recibido: [");
    for (size_t i = 0; i < n; i++) {
      Serial.print(target_relative_pos[i]);
      if (i < n - 1) Serial.print(", ");
    }
    Serial.println("]");
  }
}

// ================= UART Parsing =================
bool parse_uart_data(float angles_out[NUM_ANGULOS]) {
  static String buffer;
  while (Serial2.available()) {
    char c = Serial2.read();
    if (c == '\n') {
      if (buffer.startsWith("DATA:")) {
        buffer.remove(0, 5); // quitar "DATA:"
        int count = 0;
        char *token = strtok((char*)buffer.c_str(), ",");
        while (token != NULL && count < NUM_ANGULOS) {
          angles_out[count++] = atof(token);
          token = strtok(NULL, ",");
        }
        buffer = "";
        return (count == NUM_ANGULOS);
      }
      buffer = "";
    } else {
      buffer += c;
    }
  }
  return false;
}

// ================= SETUP =================
void setup() {
  if (DEBUG_SERIAL) {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n=================================");
    Serial.println("ESP32 micro-ROS Servo + UART Reader");
    Serial.println("Con Sensor de Presencia GPIO36");
    Serial.println("=================================");
  }

  // Configurar pin del sensor de presencia
  pinMode(SENSOR_PRESENCIA_PIN, INPUT);
  Serial.print("🔧 Sensor de presencia configurado en GPIO");
  Serial.println(SENSOR_PRESENCIA_PIN);

  // UART para leer ángulos desde Arduino
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);

  // micro-ROS
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_servo_node", "", &support));

  // --- Suscriptor ---
  std_msgs__msg__Int32MultiArray__init(&sub_msg);
  sub_msg.data.data = sub_data_buffer;
  sub_msg.data.capacity = N_SERVOS;
  sub_msg.data.size = 0;
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_commands"
  ));

  // --- Publicador de ángulos ---
  std_msgs__msg__Int32MultiArray__init(&pub_msg);
  pub_msg.data.data = pub_data_buffer;
  pub_msg.data.capacity = NUM_ANGULOS;
  pub_msg.data.size = NUM_ANGULOS;
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_angles"
  ));

  // --- NUEVO: Publicador de presencia ---
  std_msgs__msg__Bool__init(&msg_presencia);
  RCCHECK(rclc_publisher_init_default(
    &publisher_presencia,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "presencia_detectada"
  ));

  // --- Executor ---
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
                                         &subscription_callback, ON_NEW_DATA));

  // Inicializar servos
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].attach(SERVO_PINS[i], SERVO_MIN_US, SERVO_MAX_US);
  }
  move_to_home();

  if (DEBUG_SERIAL) Serial.println("Setup completo. Esperando datos...");
}

// ================= LOOP =================
void loop() {
  static unsigned long last_move = 0;
  static unsigned long last_publish = 0;
  static unsigned long last_presencia_publish = 0;
  static bool ultima_presencia_publicada = false;

  // micro-ROS
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

  // Leer sensor de presencia
  leer_sensor_presencia();

  // Publicar estado del sensor (solo cuando cambie)
  if (millis() - last_presencia_publish > 100) { // 10Hz
    if (presencia_detectada != ultima_presencia_publicada) {
      msg_presencia.data = presencia_detectada;
      RCSOFTCHECK(rcl_publish(&publisher_presencia, &msg_presencia, NULL));
      
      if (DEBUG_SERIAL) {
        Serial.print("📤 Publicado /presencia_detectada: ");
        Serial.println(presencia_detectada ? "true" : "false");
      }
      
      ultima_presencia_publicada = presencia_detectada;
    }
    last_presencia_publish = millis();
  }

  // Movimiento de servos
  if (millis() - last_move > MOVE_INTERVAL) {
    update_servo_positions();
    last_move = millis();
  }

  // Leer ángulos desde UART y publicar
  if (millis() - last_publish > 50) { // cada 50ms ≈ 20Hz
    float angles[NUM_ANGULOS];
    if (parse_uart_data(angles)) {
      for (int i = 0; i < NUM_ANGULOS; i++) {
        pub_data_buffer[i] = (int)angles[i];
      }
      RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));

      if (DEBUG_SERIAL) {
        Serial.print("📤 Publicado /servo_angles: [");
        for (int i = 0; i < NUM_ANGULOS; i++) {
          Serial.print(pub_data_buffer[i]);
          if (i < NUM_ANGULOS - 1) Serial.print(", ");
        }
        Serial.println("]");
      }
    }
    last_publish = millis();
  }

  delay(2);
}
