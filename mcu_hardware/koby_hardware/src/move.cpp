#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>

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
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <std_msgs/msg/int32.h>

#include <motorprik.h>
#include <config.h>
#include <imu_bno055.h>

// Servo ESC1;
// Servo ESC2;

Motor motor1(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motor2(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

IMU_BNO055 bno055;



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
rcl_publisher_t debug_encoder_publisher;

rcl_subscription_t move_motor_subscriber;
geometry_msgs__msg__Twist move_msg;

geometry_msgs__msg__Twist debug_motor_msg;
geometry_msgs__msg__Twist debug_encoder_msg;

rcl_publisher_t imu_pos_angle_publisher;
geometry_msgs__msg__Twist imu_pos_angle_msg;

rcl_publisher_t imu_data_publisher;
sensor_msgs__msg__Imu imu_data_msg;

rcl_publisher_t imu_mag_publisher;
sensor_msgs__msg__MagneticField imu_mag_msg;

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

Motor motorcontroller1(PWM_FREQUENCY, PWM_BITS, MOTOR1_INV, MOTOR1_BRAKE, MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B);
Motor motorcontroller2(PWM_FREQUENCY, PWM_BITS, MOTOR2_INV, MOTOR2_BRAKE, MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B);

enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;


//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
void imu_pub();
struct timespec getTime();

void Move();

//------------------------------ < Main > -------------------------------------//

void setup()
{
    // ESC1.attach(8,1000,2000);
    // ESC2.attach(9,1000,2000);
    Serial.begin(115200);
    bno055.init();
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
        //Dont Forget Here!!
        motor1.spin(0);
        motor2.spin(0);
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
        imu_pub();
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
    allocator = rcl_get_default_allocator();

    init_options = rcl_get_zero_initialized_init_options();
    rcl_init_options_init(&init_options, allocator);
    rcl_init_options_set_domain_id(&init_options, 10);

    rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

    // create node
    RCCHECK(rclc_node_init_default(&node, "int32_publisher_rclc", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_motor_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/motor/move"));

    RCCHECK(rclc_subscription_init_default(
        &move_motor_subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/kobe/cmd_move/rpm"));

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_data_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/kobe/imu/data"));
      
    RCCHECK(rclc_publisher_init_best_effort(
         &imu_mag_publisher,
         &node,
         ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
         "/kobe/imu/mag"));
      
    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pos_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/kobe/imu/pos_angle"));

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
        &move_motor_subscriber,
        &move_msg,
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

    // rcl_publisher_fini(&debug_motor_publisher, &node);
    rcl_subscription_fini(&move_motor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rcl_publisher_fini(&imu_data_publisher, &node);
    rcl_publisher_fini(&imu_mag_publisher, &node);
    rcl_publisher_fini(&imu_pos_angle_publisher, &node);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void Move()
{

    

    float motor1Speed = move_msg.linear.x;
    float motor2Speed = move_msg.linear.y;
    motorcontroller1.spin(motor1Speed);
    motorcontroller2.spin(motor2Speed);


}

void imu_pub(){
    bno055.getIMUData(imu_data_msg, imu_mag_msg, imu_pos_angle_msg);

    struct timespec time_stamp = getTime();
    imu_data_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_data_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_data_msg.header.frame_id.data = "imu_link";
  
    imu_data_msg.angular_velocity_covariance[0] = 0.0001;
    imu_data_msg.angular_velocity_covariance[4] = 0.0001;
    imu_data_msg.angular_velocity_covariance[8] = 0.0001;
  
    imu_data_msg.linear_acceleration_covariance[0] = 0.04;
    imu_data_msg.linear_acceleration_covariance[4] = 0.04;
    imu_data_msg.linear_acceleration_covariance[8] = 0.04;
  
    imu_data_msg.orientation_covariance[0] = 0.0025;
    imu_data_msg.orientation_covariance[4] = 0.0025;
    imu_data_msg.orientation_covariance[8] = 0.0025;
  
    imu_mag_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
  
    rcl_publish(&imu_data_publisher, &imu_data_msg, NULL);
    rcl_publish(&imu_mag_publisher, &imu_mag_msg, NULL);
    rcl_publish(&imu_pos_angle_publisher, &imu_pos_angle_msg, NULL);
}



void publishData()
{
    debug_motor_msg.linear.x = move_msg.linear.x;
    debug_motor_msg.linear.y = move_msg.linear.y;


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
    while (true)
    {
    }
}

void flashLED(int n_times)
{
    for (int i = 0; i < n_times; i++)
    {

    }
    delay(1000);
}