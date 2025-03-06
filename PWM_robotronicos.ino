// Include Libraries to be used
#include <micro_ros_arduino.h>    //micro-ros-arduino library
#include <rcl/rcl.h>              //Core ROS 2 Client Library (RCL) for node management.
#include <rcl/error_handling.h>   //Error handling utilities for Micro-ROS.
#include <rclc/rclc.h>            //Micro-ROS Client library for embedded devices.
#include <rclc/executor.h>        //Micro-ROS Executor to manage callbacks
#include <std_msgs/msg/float32.h> //Predefined ROS 2 message type for float32
#include <stdio.h>                //Standard I/O library for debugging.
#include <math.h>

//Specifies GPIO pin 13 for controlling an LED
#define ENA 26
#define In1 14
#define In2 27

//Declare nodes to be used
rcl_node_t node;            //Represents a ROS 2 Node running on the microcontroller.

//Instantiate executor and its support classes
rclc_executor_t executor;   //Manages task execution (timers, callbacks, etc.).
rclc_support_t support;     //Handles initialization & communication setup.
rcl_allocator_t allocator;  //Manages memory allocation.

//Declare Publishers to be used
rcl_publisher_t publisher_pwm;  //Declares a ROS 2 publisher for sending messages.

//Declare Subscribers to be used
rcl_subscription_t subscriber;

//Declare Messages to be used
std_msgs__msg__Float32 msg_PWM;  //Defines a message of type Float32.

//Define Macros to be used
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

//Specifies GPIO pin 13 for controlling an LED
#define LED_PIN 22

// PWM Configuration
#define PWM_FRQ 980 //Define PWM Frequency
#define PWM_RES 8  //Define PWM Resolution
#define PWM_CHNL 0    //Define Channel
#define MSG_MIN_VAL -1 //Define min input value
#define MSG_MAX_VAL 1 //Define max input value

// Variables to be used
float pwm_set_point = 0.0;

//Define Error Functions
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Toggle LED state
    delay(100); // Wait 100 milliseconds
  }
}

//Define callbacks
void subscription_callback(const void *motor_robotronicos)
{  
  //Get the message received and store it on the message msg
  const std_msgs__msg__Float32 *msg = (const std_msgs__msg__Float32 *)motor_robotronicos;
  pwm_set_point = constrain(msg->data, MSG_MIN_VAL, MSG_MAX_VAL);

  //Movimiento del motor
  if (pwm_set_point > 0){
    digitalWrite(In1, HIGH);
    digitalWrite(In2, LOW);
  } else if (pwm_set_point == 0) {
    digitalWrite(In1, LOW);
    digitalWrite(In2, LOW);
  } else {
    digitalWrite(In1, LOW);
    digitalWrite(In2, HIGH);
  }

  int val_PWM = (int)(fabs(pwm_set_point) * 255);  // PMW entre 0 y 255
  ledcWrite(PWM_CHNL, val_PWM);

  //Publica el valor de PWM 
  msg_PWM.data = pwm_set_point;
  RCSOFTCHECK(rcl_publish(&publisher_pwm, &msg_PWM, NULL));
}

//Setup
void setup() {
  // Initializes communication between ESP32 and the ROS 2 agent (Serial).
  set_microros_transports();
  
  //Setup Microcontroller Pins
  pinMode(ENA, OUTPUT);
  pinMode(In1, OUTPUT);
  pinMode(In2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  ledcSetup(PWM_CHNL, PWM_FRQ, PWM_RES);  //Setup the PWM
  ledcAttachPin(ENA, PWM_CHNL);       //Setup Attach the Pin to the Channel   
  
  //Connection delay
  delay(2000);

  //Initializes memory allocation for Micro-ROS operations.
  allocator = rcl_get_default_allocator();

  //Creates a ROS 2 support structure to manage the execution context.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "motor_robotronicos", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher_pwm,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "pwm_robotronicos"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "cmd_pwm_robotronicos"));

  // Initializes the Micro-ROS Executor, which manages tasks and callbacks.
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_PWM, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  //Executor Spin
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}