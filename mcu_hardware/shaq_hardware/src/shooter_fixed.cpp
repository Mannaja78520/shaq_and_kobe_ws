#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <cmath>

#define LED_PIN 13

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int8.h>
#include <std_msgs/msg/int16_multi_array.h>
#include <geometry_msgs/msg/twist.h>

#include <motorevo.h>
#include <motorprik.h>
#include <encoder.h>
#include <PIDF.h>
#include "../config/shooter_output.h"
// #include "../config/drive_output_teensy.h"

#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            rclErrorLoop();          \
        }                            \
    }
#define RCSOFTCHECK(fn)              \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
        }                            \
    }
#define EXECUTE_EVERY_N_MS(MS, X)          \
    do                                     \
    {                                      \
        static volatile int64_t init = -1; \
        if (init == -1)                    \
        {                                  \
            init = uxr_millis();           \
        }                                  \
        if (uxr_millis() - init > MS)      \
        {                                  \
            X;                             \
            init = uxr_millis();           \
        }                                  \
    } while (0)

//------------------------------ < Define > -------------------------------------//

rcl_publisher_t debug_motor_publisher;
geometry_msgs__msg__Twist debug_motor_msg;

rcl_publisher_t encoder_publisher;
geometry_msgs__msg__Twist encoder_msg;

rcl_subscription_t shooter_motor_subscriber;
geometry_msgs__msg__Twist shooter_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long current_time = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


EVODrive motorshooter1(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BREAK, MOTORSHOOTER1_PWM, MOTORSHOOTER1_IN_A, MOTORSHOOTER1_IN_B);
EVODrive motorshooter2(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BREAK, MOTORSHOOTER2_PWM, MOTORSHOOTER2_IN_A, MOTORSHOOTER2_IN_B);
Motor motorlift(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BREAK, MOTORLIFT_PWM, MOTORLIFT_IN_A, MOTORLIFT_IN_B);
// EVODrive motorlift(PWM_FREQUENCY, PWM_BITS, MOTOR3_INV, MOTOR3_BREAK, MOTORLIFT_PWM, MOTORLIFT_IN_A, MOTORLIFT_IN_B);

Encoder motor1_encoder(MOTOR1_ENCODER_PIN_A, MOTOR1_ENCODER_PIN_B, COUNTS_PER_REV1, MOTOR1_ENCODER_INV, ENCODER_GEAR_RATIO);
Encoder motor2_encoder(MOTOR2_ENCODER_PIN_A, MOTOR2_ENCODER_PIN_B, COUNTS_PER_REV2, MOTOR2_ENCODER_INV, ENCODER_GEAR_RATIO);

PIDF motor1_controller(I_Min, I_Max, PWM_Min, PWM_Max, K_P, K_I, K_D, K_F);
PIDF motor2_controller(I_Min, I_Max, PWM_Min, PWM_Max, K_P, K_I, K_D, K_F);



//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void fullStop();

void Move();

//------------------------------ < Main > -------------------------------------//

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}

// Reboot the board (Teensy Only)
void doReboot()
{
  SCB_AIRCR = 0x05FA0004;
}


void setup()
{

    Serial.begin(115200);
    set_microros_serial_transports(Serial);
}

void loop()
{
    switch (state)
    {
    case WAITING_AGENT:
        EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
        break;
    case AGENT_AVAILABLE:
        state = (true == createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
        if (state == WAITING_AGENT)
        {
            destroyEntities();
        }
        break;
    case AGENT_CONNECTED:
        EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
        if (state == AGENT_CONNECTED)
        {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        }
        break;
    case AGENT_DISCONNECTED:
        motorshooter1.spin(0);
        motorshooter2.spin(0);
        motorlift.spin(0);

        fullStop();

        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Fuction > -------------------------------------//


void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        Move();
        publishData();
    }
}

void twistCallback(const void *msgin)
{
    prev_cmd_time = millis();
}

void twist2Callback(const void *msgin)
{
    prev_cmd_time = millis();
}

bool createEntities()
{

    flashLED(3);

    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "shaq_shooter_node", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/shaq/debug/cmd_shoot/rpm"));

    RCCHECK(rclc_subscription_init_default(
        &shooter_motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/shaq/cmd_shoot/rpm"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &shooter_motor_subscriber,
        &shooter_msg,
        &twistCallback,
        ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_subscription_fini(&shooter_motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    flashLED(5);

    return true;
}

void fullStop()
{
  shooter_msg.linear.x = 0.0;
  shooter_msg.linear.y = 0.0;
  shooter_msg.linear.z = 0.0;
  shooter_msg.angular.x = 0.0;
  shooter_msg.angular.y = 0.0;
  shooter_msg.angular.z = 0.0;

  motorshooter1.brake();
  motorshooter2.brake();
  motorlift.brake();

}


void Move()
{

    float motor1Speed = shooter_msg.linear.x;
    float motor2Speed = shooter_msg.linear.y;
    float motor3Speed = shooter_msg.linear.z;
    
    motorshooter1.spin(motor1Speed);
    motorshooter2.spin(motor2Speed);

    
    float current_rpm_motor1 = motor1_encoder.getRPM();
    float current_rpm_motor2 = motor2_encoder.getRPM();

    debug_motor_msg.angular.x = std::round(current_rpm_motor1 * 100.0) / 100.0;
    debug_motor_msg.angular.y = std::round(current_rpm_motor2 * 100.0) / 100.0;

    
    // motorshooter1.spin(motor1_controller.compute(motor1Speed, current_rpm_motor1));
    // motorshooter2.spin(motor2_controller.compute(motor2Speed, current_rpm_motor2));
    motorlift.spin(motor3Speed);

}



void publishData()
{

    debug_motor_msg.linear.x = shooter_msg.linear.x;
    debug_motor_msg.linear.y = shooter_msg.linear.y;
    debug_motor_msg.linear.z = shooter_msg.linear.z;

    

    struct timespec time_stamp = getTime();
    rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
}

void syncTime()
{
    // get the current time from the agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    // now we can find the difference between ROS time and uC time
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    // add time difference between uC time and ROS time to
    // synchronize time with ROS
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;

    return tp;
}

void rclErrorLoop()
{
    flashLED(2);
    doReboot();
}

