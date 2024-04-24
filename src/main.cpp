#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist velocity;

rcl_publisher_t publisher;
geometry_msgs__msg__Twist position;

rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;


#define LED_PIN 13
// Motor A
int motor1Pin1 = 27; 
int motor1Pin2 = 26; 
int enable1Pin = 14; 

// Motor B
int motor2Pin1 = 32; 
int motor2Pin2 = 33; 
int enable2Pin = 25; 

// Encoder Right
const int encoderRight = 35; // Pin connected to the data output of the optical encoder
volatile int encoderCountR = 0; // Variable to store the number of ender toggles
int rotationR = 0; // Variable to store the number of rotations

// Encoder Left
const int encoderLeft = 34; // Pin connected to the data output of the optical encoder
volatile int encoderCountL = 0; // Variable to store the number of ender toggles
int rotationL = 0; // Variable to store the number of rotations


// Define encoder constants
const float WHEEL_DIAMETER_CM = 6.5;
const int ENCODER_RESOLUTION = 20;
const float DISTANCE_BETWEEN_WHEELS_CM = 12.5;
const float TICKS_PER_CM = ENCODER_RESOLUTION / (WHEEL_DIAMETER_CM * PI);

// Define robot variables
float x = 0; // Robot x position
float y = 0; // Robot y position
float theta = 0; // Robot orientation (in radians)

// Previous encoder counts
volatile int prevEncoderCountR = 0;
volatile int prevEncoderCountL = 0;


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// Setting PWM properties
const int freq = 500;
const int pwmChannelLeft = 0;
const int pwmChannelRight = 1;
const int resolution = 8;
int dutyCycle = 0;

// PID constants
const float KP_linear = 0.1;
const float KI_linear = 0.01;
const float KD_linear = 0.01;

const float KP_angular = 0.1;
const float KI_angular = 0.01;
const float KD_angular = 0.01;

// Desired velocity
float desired_linear_velocity = 0;
float desired_angular_velocity = 0;

// Actual velocity
float actual_linear_velocity = 0;
float actual_angular_velocity = 0;

// PID variables
float prev_error_linear = 0;
float integral_linear = 0;

float prev_error_angular = 0;
float integral_angular = 0;

float control_linear = 100;
float control_angular = 100;

unsigned long prevTime = 0;

// Previous encoder counts
volatile int prevEncoderCountPIDR = 0;
volatile int prevEncoderCountPIDL = 0;

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// Odometry calculation function
void updateOdometry(int encoderCountR, int encoderCountL) {
    // Calculate delta encoder counts
    int deltaEncoderR = encoderCountR - prevEncoderCountR;
    int deltaEncoderL = encoderCountL - prevEncoderCountL;

    // Calculate linear and angular displacement
    float linearDisplacement = (deltaEncoderR + deltaEncoderL) / 2.0 * TICKS_PER_CM;
    float angularDisplacement = (deltaEncoderR - deltaEncoderL) / DISTANCE_BETWEEN_WHEELS_CM;

    // Update x, y, and theta
    x += linearDisplacement * cos(theta);
    y += linearDisplacement * sin(theta);
    theta += angularDisplacement;

    // Normalize theta to the range [0, 2*PI)
    while (theta >= 2 * PI) {
        theta -= 2 * PI;
    }
    while (theta < 0) {
        theta += 2 * PI;
    }

    // Update previous encoder counts
    prevEncoderCountR = encoderCountR;
    prevEncoderCountL = encoderCountL;
}


void handleEncoderInterruptR() {
    encoderCountR++; // Increment the rotation count when a high signal is detected
    if (encoderCountR % 20 == 0 && encoderCountR != 0) {
        rotationR++;
    }
}
void handleEncoderInterruptL() {
    encoderCountL++; // Increment the rotation count when a high signal is detected
    if (encoderCountL % 20 == 0 && encoderCountL != 0) {
        rotationL++;
    }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &position, NULL));
      /*
      unsigned long currentTime = millis();
      float timeElapsed = (float)(currentTime - prevTime) / 1000;
      float RPM = (rotationR / timeElapsed) * 60;
      rotationR = 0;
      prevTime = currentTime;
      position.data = RPM;
      */
  }
  
  position.linear.x = x;
  position.linear.y = y;
  position.linear.z = (rotationL+rotationR)/2.0;
  position.angular.x = actual_linear_velocity;
  position.angular.y = actual_angular_velocity;
  position.angular.z = theta;
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  // if velocity in x direction is 0 turn off LED, if 1 turn on LED
  // if velocity in x direction is 0 stop the motors, if 1 move the motors
  int deltaEncoderR = encoderCountR - prevEncoderCountR;
  int deltaEncoderL = encoderCountL - prevEncoderCountL;
  float linearDisplacement = 0;
  float angularDisplacement = 0;
  float velLin = abs(msg->linear.x);
  float velAng = abs(msg->angular.z);
  desired_linear_velocity = velLin;
  desired_angular_velocity = velAng;

  if (msg->linear.x < 0.0 && msg -> angular.z == 0.0){
    linearDisplacement = -(deltaEncoderR + deltaEncoderL) / 2.0 * TICKS_PER_CM;
    angularDisplacement = (deltaEncoderR - deltaEncoderL) / DISTANCE_BETWEEN_WHEELS_CM;
    ledcWrite(pwmChannelLeft, control_linear);
    ledcWrite(pwmChannelRight, control_linear);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW); 
  } else if (msg->linear.x > 0.0 && msg -> angular.z == 0.0){
    linearDisplacement = +(deltaEncoderR + deltaEncoderL) / 2.0 * TICKS_PER_CM;
    angularDisplacement = (deltaEncoderR - deltaEncoderL) / DISTANCE_BETWEEN_WHEELS_CM;
    ledcWrite(pwmChannelLeft, control_linear);
    ledcWrite(pwmChannelRight, control_linear);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (msg->linear.x == 0.0 && msg -> angular.z < 0.0){
    linearDisplacement = (deltaEncoderR + deltaEncoderL) / 2.0 * TICKS_PER_CM;
    angularDisplacement = (deltaEncoderR - deltaEncoderL) / DISTANCE_BETWEEN_WHEELS_CM;
    ledcWrite(pwmChannelLeft, control_angular);
    ledcWrite(pwmChannelRight, control_angular);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else if (msg->linear.x == 0.0 && msg -> angular.z > 0.0){
    linearDisplacement = (deltaEncoderR + deltaEncoderL) / 2.0 * TICKS_PER_CM;
    angularDisplacement = -(deltaEncoderR - deltaEncoderL) / DISTANCE_BETWEEN_WHEELS_CM;
    ledcWrite(pwmChannelLeft, control_angular);
    ledcWrite(pwmChannelRight, control_angular);
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH); 
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }
  
  x += linearDisplacement * cos(theta);
  y += linearDisplacement * sin(theta);
  theta += angularDisplacement;
  while (theta >= 2 * PI) {
        theta -= 2 * PI;
    }
    while (theta < 0) {
        theta += 2 * PI;
    }
  // Update previous encoder counts
  prevEncoderCountR = encoderCountR;
  prevEncoderCountL = encoderCountL;
  

}

float updatePID_linear(float setpoint, float actual_value) {
    float error = setpoint - actual_value;
    integral_linear += error;
    float derivative = error - prev_error_linear;

    float output = KP_linear * error + KI_linear * integral_linear + KD_linear * derivative;

    prev_error_linear = error;

    return output;
}

// Update PID control for angular velocity
float updatePID_angular(float setpoint, float actual_value) {
    float error = setpoint - actual_value;
    integral_angular += error;
    float derivative = error - prev_error_angular;

    float output = KP_angular * error + KI_angular * integral_angular + KD_angular * derivative;

    prev_error_angular = error;

    return output;
}

void setup() {

  set_microros_transports();
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(pwmChannelLeft, freq, resolution);
  ledcSetup(pwmChannelRight, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannelLeft);
  ledcAttachPin(enable2Pin, pwmChannelRight);



  pinMode(encoderRight, INPUT);
  pinMode(encoderLeft, INPUT);
  attachInterrupt(digitalPinToInterrupt(encoderRight), handleEncoderInterruptR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLeft), handleEncoderInterruptL, RISING);
  
  // configure ROS
  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "microRosESP", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "turtlebot_cmdVel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "turtlebot_position"));

  // create timer,
  const unsigned int timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &velocity, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

}

void loop() {
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  // Calculate time elapsed since last measurement
  /*
  */
  unsigned long currentTime = millis();
  
  float timeElapsed = (currentTime - prevTime) / 1000.0; // in seconds
  // Calculate change in encoder counts
  int deltaEncoderR = encoderCountR - prevEncoderCountPIDR;
  int deltaEncoderL = encoderCountL - prevEncoderCountPIDL;
  float rpmR = (deltaEncoderR / 20) / (timeElapsed / 60.0); // RPM = (ticks / ticks_per_rev) / (time / 60)
  float rpmL = (deltaEncoderL / 20) / (timeElapsed / 60.0);
  // Update previous encoder counts
  prevEncoderCountPIDR = encoderCountR;
  prevEncoderCountPIDL = encoderCountL;
  actual_linear_velocity = (rpmR + rpmL) * WHEEL_DIAMETER_CM * PI / 2.0;
  actual_angular_velocity = (rpmR - rpmL) / DISTANCE_BETWEEN_WHEELS_CM;
  control_linear = updatePID_linear(desired_linear_velocity, actual_linear_velocity);
  control_angular = updatePID_angular(desired_angular_velocity, actual_angular_velocity);
  prevTime = currentTime;
  
 // updateOdometry(encoderCountR, encoderCountL);
 

}