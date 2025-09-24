#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

// Definición de servos y pines
#define NUM_SERVOS 3
#define SERVO1_PIN 5
#define SERVO2_PIN 18
#define SERVO3_PIN 19

Servo servos[NUM_SERVOS];
int servo_pins[NUM_SERVOS] = {SERVO1_PIN, SERVO2_PIN, SERVO3_PIN};

// Subscriptores y mensajes
rcl_subscription_t subscribers[NUM_SERVOS];
std_msgs__msg__Int32 msgs[NUM_SERVOS];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// Nombres de tópicos
const char* topic_names[NUM_SERVOS] = {
  "servo1_cmd",
  "servo2_cmd", 
  "servo3_cmd"
};

// MACROS CORREGIDAS - AGREGAR RCSOFTCHECK
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}} // Esta línea faltaba

void error_loop(){
  while(1){
    delay(100);
  }
}

// Callback para cada servo
void subscription_callback(const void * msgin, void * context){
  int servo_index = *(int*)context;
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  
  int angle = msg->data;
  if (angle >= 0 && angle <= 180) {
    // LÍNEA DONDE SE MUEVE EL SERVO
    servos[servo_index].write(angle);
    
    Serial.print("Servo ");
    Serial.print(servo_index);
    Serial.print(" movido a: ");
    Serial.println(angle);
  }
}

void setup() {
  Serial.begin(115200);
  set_microros_transports();
  
  // Inicializar servos
  for(int i = 0; i < NUM_SERVOS; i++) {
    servos[i].attach(servo_pins[i]);
  }

  allocator = rcl_get_default_allocator();

  // init ROS
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "multi_servo_node", "", &support));

  // Crear subscriptores para cada servo
  for(int i = 0; i < NUM_SERVOS; i++) {
    // Asignar memoria para el contexto (índice del servo)
    int* context = (int*)malloc(sizeof(int));
    *context = i;
    
    RCCHECK(rclc_subscription_init_default(
      &subscribers[i],
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      topic_names[i]));
  }

  // Configurar executor con todos los subscriptores
  RCCHECK(rclc_executor_init(&executor, &support.context, NUM_SERVOS, &allocator));
  
  for(int i = 0; i < NUM_SERVOS; i++) {
    int* context = (int*)malloc(sizeof(int));
    *context = i;
    
    RCCHECK(rclc_executor_add_subscription_with_context(
      &executor, 
      &subscribers[i], 
      &msgs[i], 
      &subscription_callback, 
      context, 
      ON_NEW_DATA));
  }
}

void loop() {
  // LÍNEA CORREGIDA - ahora RCSOFTCHECK está definido
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  delay(10);
}
