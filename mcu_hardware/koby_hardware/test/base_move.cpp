#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <micro_ros_platformio.h>
#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <motorevo.h>
#include <motorprik.h>

// Motor definitions for differential drive
EVODrive left_motor(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BREAK, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
EVODrive right_motor(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BREAK, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

rcl_publisher_t debug_motor_publisher;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Twist debug_motor_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

//------------------------------ < Function Prototypes > ------------------------------//
void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();
void controlCallback(rcl_timer_t *timer, int64_t last_call_time);
void cmdVelCallback(const void *msgin);

//------------------------------ < Main > -------------------------------------//

void setup()
{
    Serial.begin(115200);
    set_microros_serial_transports(Serial);  // Set up micro-ROS communication

    // Initialize micro-ROS
    state = WAITING_AGENT;
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
        left_motor.spin(0);
        right_motor.spin(0);
        destroyEntities();
        state = WAITING_AGENT;
        break;
    default:
        break;
    }
}

//------------------------------ < Function Definitions > ------------------------------//

void controlCallback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer != NULL)
    {
        // Differential drive control based on cmd_vel message
        float linear_x = cmd_vel_msg.linear.x;   // Forward/backward speed
        float angular_z = cmd_vel_msg.angular.z; // Rotation speed

        // Calculate wheel speeds for differential drive
        float left_speed = linear_x - angular_z;
        float right_speed = linear_x + angular_z;

        // Set motor speeds
        left_motor.spin(left_speed);
        right_motor.spin(right_speed);

        // Publish motor control data for debugging
        publishData();
    }
}

void cmdVelCallback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    cmd_vel_msg = *msg;
    prev_cmd_time = millis();
}

bool createEntities()
{
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // Create micro-ROS node
    RCCHECK(rclc_node_init_default(&node, "differential_drive_node", "", &support));

    // Create a publisher for debugging
    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/motor/velocity"));

    // Create a subscriber to receive cmd_vel messages for controlling the robot
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"));

    // Create a timer for controlling motors at 50 Hz
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));

    // Create an executor for handling subscriptions and timers
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    // Add the cmd_vel subscription to the executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &cmd_vel_subscriber,
        &cmd_vel_msg,
        &cmdVelCallback,
        ON_NEW_DATA));

    // Add the control timer to the executor
    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // Synchronize time with the ROS2 agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_subscription_fini(&cmd_vel_subscriber, &node);
    rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void publishData()
{
    debug_motor_msg.linear.x = cmd_vel_msg.linear.x;
    debug_motor_msg.angular.z = cmd_vel_msg.angular.z;

    struct timespec time_stamp = getTime();
    rcl_publish(&debug_motor_publisher, &debug_motor_msg, NULL);
}

void syncTime()
{
    // Synchronize time with the ROS2 agent
    unsigned long now = millis();
    RCCHECK(rmw_uros_sync_session(10));
    unsigned long long ros_time_ms = rmw_uros_epoch_millis();
    time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
    struct timespec tp = {0};
    unsigned long long now = millis() + time_offset;
    tp.tv_sec = now / 1000;
    tp.tv_nsec = (now % 1000) * 1000000;
    return tp;
}

void rclErrorLoop()
{
    while (true)
    {
    }
}

