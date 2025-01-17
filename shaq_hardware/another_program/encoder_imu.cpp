#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

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
#include <sensor_msgs/msg/magnetic_field.h>

#include <ESP32Encoder.h> 
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>

#include "../config/encoder_input.h"
// #include "../config/encoder_input_teensy.h"

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

rcl_publisher_t debug_encoder_publisher;
rcl_publisher_t imu_data_publisher;
rcl_publisher_t imu_pos_angle_publisher;
rcl_publisher_t imu_mag_publisher;

geometry_msgs__msg__Twist debug_encoder_msg;
geometry_msgs__msg__Twist imu_pos_angle_msg;
sensor_msgs__msg__Imu imu_data_msg;
sensor_msgs__msg__MagneticField imu_mag_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;
rcl_init_options_t init_options;


enum states
{
    WAITING_AGENT,
    AGENT_AVAILABLE,
    AGENT_CONNECTED,
    AGENT_DISCONNECTED
} state;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);

ESP32Encoder encoder1;
ESP32Encoder encoder2;
ESP32Encoder encoder3;
ESP32Encoder encoder4;

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
unsigned long prev_odom_update = 0;
unsigned long LastHarvest_time = 0;
unsigned long long current_time = 0;
unsigned long long lastTime = 0;
double time_diff;

float motor1RPM = 0;
float motor2RPM = 0;
float motor3RPM = 0;
float motor4RPM = 0;

// float GEAR_RATIO = 1.15;
const int PULSES_PER_REVOLUTION = 2500;
const int encoderTick = 2;

double xPos = 0, yPos = 0, zPos = 0, headingVel = 0, vX = 0, vY = 0, vZ = 0;
double DEG_2_RAD = 0.01745329251; //trig functions require radians, BNO055 outputs degrees
imu::Vector<3> gavity;

bool checkTime = false;
//------------------------------ < Fuction Prototype > ------------------------------//

void rclErrorLoop();
void syncTime();
bool createEntities();
bool destroyEntities();
void publishData();
struct timespec getTime();

void Encoder_read();
void getBNO055Data();
//------------------------------ < Main > -------------------------------------//

void setup()
{
    Serial.begin(115200);

    bno.begin();
    sensor_t sensor;
    bno.getSensor(&sensor);
    delay(500);

    /* Get the system status values (mostly for debugging purposes) */
    uint8_t system_status, self_test_results, system_error;
    system_status = self_test_results = system_error = 0;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    /* Display the results in the Serial Monitor */
    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);

    bno.setExtCrystalUse(true);

    if (ENCODER1_INV) encoder1.attachHalfQuad(ENCODER1_PIN_B, ENCODER1_PIN_A);
    else encoder1.attachHalfQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    if (ENCODER2_INV) encoder2.attachHalfQuad(ENCODER2_PIN_B, ENCODER2_PIN_A);
    else encoder2.attachHalfQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    if (ENCODER3_INV) encoder3.attachHalfQuad(ENCODER3_PIN_B, ENCODER3_PIN_A);
    else encoder3.attachHalfQuad(ENCODER3_PIN_A, ENCODER3_PIN_B);
    if (ENCODER4_INV) encoder4.attachHalfQuad(ENCODER4_PIN_B, ENCODER4_PIN_A);
    else encoder4.attachHalfQuad(ENCODER4_PIN_A, ENCODER4_PIN_B);

    // if (ENCODER1_INV) encoder1.attachFullQuad(ENCODER1_PIN_B, ENCODER1_PIN_A);
    // else encoder1.attachFullQuad(ENCODER1_PIN_A, ENCODER1_PIN_B);
    // if (ENCODER2_INV) encoder2.attachFullQuad(ENCODER2_PIN_B, ENCODER2_PIN_A);
    // else encoder2.attachFullQuad(ENCODER2_PIN_A, ENCODER2_PIN_B);
    // if (ENCODER3_INV) encoder3.attachFullQuad(ENCODER3_PIN_B, ENCODER3_PIN_A);
    // else encoder3.attachFullQuad(ENCODER3_PIN_A, ENCODER3_PIN_B);
    // if (ENCODER4_INV) encoder4.attachFullQuad(ENCODER4_PIN_B, ENCODER4_PIN_A);
    // else encoder4.attachFullQuad(ENCODER4_PIN_A, ENCODER4_PIN_B);

    // if (ENCODER1_INV) encoder1.attachSingleEdge(ENCODER1_PIN_B, ENCODER1_PIN_A);
    // else encoder1.attachSingleEdge(ENCODER1_PIN_A, ENCODER1_PIN_B);
    // if (ENCODER2_INV) encoder2.attachSingleEdge(ENCODER2_PIN_B, ENCODER2_PIN_A);
    // else encoder2.attachSingleEdge(ENCODER2_PIN_A, ENCODER2_PIN_B);
    // if (ENCODER3_INV) encoder3.attachSingleEdge(ENCODER3_PIN_B, ENCODER3_PIN_A);
    // else encoder3.attachSingleEdge(ENCODER3_PIN_A, ENCODER3_PIN_B);
    // if (ENCODER4_INV) encoder4.attachSingleEdge(ENCODER4_PIN_B, ENCODER4_PIN_A);
    // else encoder4.attachSingleEdge(ENCODER4_PIN_A, ENCODER4_PIN_B);

    gavity = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();
    set_microros_serial_transports(Serial);
    lastTime = micros();
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
            checkTime = false;
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
        current_time = micros();
        if (!checkTime) lastTime = current_time;
        checkTime = true;
        time_diff = (double)(current_time - lastTime) / 1000.0;
        Encoder_read();
        getBNO055Data();
        publishData();
        // Update lastTime for the next loop
        lastTime = current_time;
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
    RCCHECK(rclc_node_init_default(&node, "encoder_imu", "", &support));

    RCCHECK(rclc_publisher_init_best_effort(
        &debug_encoder_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "debug/move_encoder"));

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pos_angle_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "imu/pos_angle"));

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_data_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_mag_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "imu/mag"));

    // create timer for actuating the motors at 50 Hz (1000/20)
    const unsigned int control_timeout = 20;
    RCCHECK(rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(control_timeout),
        controlCallback));
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));

    // RCCHECK(rclc_executor_add_subscription(
    //     &executor,
    //     &moveMotor_subscriber,
    //     &moveMotor_msg,
    //     &twistCallback,
    //     ON_NEW_DATA));

    RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

    // synchronize time with the agent
    syncTime();

    return true;
}

bool destroyEntities()
{
    rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

    rcl_publisher_fini(&debug_encoder_publisher, &node);
    rcl_publisher_fini(&imu_pos_angle_publisher, &node);
    rcl_publisher_fini(&imu_data_publisher, &node);
    rcl_publisher_fini(&imu_mag_publisher, &node);
    // rcl_subscription_fini(&moveMotor_subscriber, &node);
    rcl_node_fini(&node);
    rcl_timer_fini(&control_timer);
    rclc_executor_fini(&executor);
    rclc_support_fini(&support);

    return true;
}

void publishData()
{
    struct timespec time_stamp = getTime();
    imu_data_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_data_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    imu_mag_msg.header.stamp.sec = time_stamp.tv_sec;
    imu_mag_msg.header.stamp.nanosec = time_stamp.tv_nsec;
    rcl_publish(&debug_encoder_publisher, &debug_encoder_msg, NULL);
    rcl_publish(&imu_pos_angle_publisher, &imu_pos_angle_msg, NULL);
    rcl_publish(&imu_data_publisher, &imu_data_msg, NULL);
    rcl_publish(&imu_mag_publisher, &imu_mag_msg, NULL);
    
}

void syncTime()
{
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    time_offset = (uint64_t)ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL - uxr_millis() * 1000ULL;
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

void Encoder_read()
{
    // Ensure time_diff is not zero to avoid division errors
    if (time_diff <= 0.0) return;

    float motor1RPM = ((double) encoder1.getCount() / (double)(ENCODER1_PULSES_PER_REVOLUTION * encoderTick)) * (60.0 / time_diff);
    float motor2RPM = ((double) encoder2.getCount() / (double)(ENCODER2_PULSES_PER_REVOLUTION * encoderTick)) * (60.0 / time_diff);
    float motor3RPM = ((double) encoder3.getCount() / (double)(ENCODER3_PULSES_PER_REVOLUTION * encoderTick)) * (60.0 / time_diff);
    float motor4RPM = ((double) encoder4.getCount() / (double)(ENCODER4_PULSES_PER_REVOLUTION * encoderTick)) * (60.0 / time_diff);

    // Log or use the RPM values
    debug_encoder_msg.linear.x = motor1RPM;
    debug_encoder_msg.linear.y = motor2RPM;
    debug_encoder_msg.linear.z = motor3RPM;
    debug_encoder_msg.angular.x = motor4RPM;

    // Reset encoders for the next calculations
    encoder1.clearCount();
    encoder2.clearCount();
    encoder3.clearCount();
    encoder4.clearCount();

}

void getBNO055Data(){
    if (time_diff <= 0.0) return;
    // sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    // bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    // bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    // bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    // bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    // bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    sensors_event_t event;
    bno.getEvent(&event);

    imu_data_msg.orientation.w = quat.w();
    imu_data_msg.orientation.x = quat.x();
    imu_data_msg.orientation.y = quat.y();
    imu_data_msg.orientation.z = quat.z();

    imu_data_msg.linear_acceleration.x = accel.x();
    imu_data_msg.linear_acceleration.y = accel.y();
    imu_data_msg.linear_acceleration.z = accel.z();

    imu_data_msg.angular_velocity.x = gyro.x();
    imu_data_msg.angular_velocity.y = gyro.y();
    imu_data_msg.angular_velocity.z = gyro.z();

    imu_mag_msg.magnetic_field.x = mag.x();
    imu_mag_msg.magnetic_field.y = mag.y();
    imu_mag_msg.magnetic_field.z = mag.z();

    // Convert degrees to radians
    double roll = DEG_2_RAD * event.orientation.roll;
    double pitch = DEG_2_RAD * event.orientation.pitch;
    double yaw = DEG_2_RAD * event.orientation.heading;
    
    // Compute the rotation matrix
    double R[3][3] = {
        {
            cos(yaw) * cos(pitch),
            cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),
            cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)
        },
        {
            sin(yaw) * cos(pitch),
            sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
            sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)
        },
        {
            -sin(pitch),
            cos(pitch) * sin(roll),
            cos(pitch) * cos(roll)
        }
    };

    // Transform local acceleration to world frame
    double accel_world_x = R[0][0] * accel.x() + R[0][1] * accel.y() + R[0][2] * accel.z();
    double accel_world_y = R[1][0] * accel.x() + R[1][1] * accel.y() + R[1][2] * accel.z();
    double accel_world_z = R[2][0] * accel.x() + R[2][1] * accel.y() + R[2][2] * accel.z();

    accel_world_z -= 9.81;
    

    //velocity = accel*dt (dt in seconds)
    //position = 0.5*accel*dt^2
    double ACCEL_POS_TRANSITION = 0.5 * time_diff * time_diff;
    // Update position using velocity and acceleration
    xPos += (vX * time_diff) + (accel_world_x * ACCEL_POS_TRANSITION);
    yPos += (vY * time_diff) + (accel_world_y * ACCEL_POS_TRANSITION);
    zPos += (vZ * time_diff) + (accel_world_z * ACCEL_POS_TRANSITION);

    vX += accel_world_x * time_diff;
    vY += accel_world_y * time_diff;
    vZ += accel_world_z * time_diff;
    // velocity of sensor in the direction it's facing
    // orientation.x from event is yaw heading from BNO in degree 0-360 degrees
    // headingVel = time_diff * accel.x() / cos(DEG_2_RAD * event.orientation.x);

    imu_pos_angle_msg.linear.x = xPos;
    imu_pos_angle_msg.linear.y = yPos;
    imu_pos_angle_msg.linear.z = zPos;
    imu_pos_angle_msg.angular.x = event.orientation.roll;
    imu_pos_angle_msg.angular.y = event.orientation.pitch;
    imu_pos_angle_msg.angular.z = event.orientation.heading;
}