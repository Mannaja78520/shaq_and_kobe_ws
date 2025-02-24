#ifndef IMU_BNO055_H
#define IMU_BNO055_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>

// Define the IMU class
class IMU_BNO055 {
public:
  IMU_BNO055();
  bool init();
  bool getQuaternion(float &qx, float &qy, float &qz, float &qw);
  bool getAngVelocity(float &wx, float &wy, float &wz);
  bool getLinearAcc(float &ax, float &ay, float &az);
  bool getMagnetometer(float &mx, float &my, float &mz);
  bool getAngle(float &roll, float &pitch, float &yaw);
  // void resetIMU();
  void getIMUData(sensor_msgs__msg__Imu &imu_msg, sensor_msgs__msg__MagneticField &mag_msg, geometry_msgs__msg__Twist &pos_angle_msg);
  // void getMagData(sen                                  sor_msgs__msg__MagneticField &mag_msg);
  // void getPositionAngle(geometry_msgs__msg__Twist &pos_angle_msg);

private:
  Adafruit_BNO055 bno;
  sensors_event_t angVelocityData , linearAccelData, magnetometerData, orientationData;
  imu::Vector<3> gavity;
  double xPos, yPos, zPos, vX, vY, vZ, time_diff;
  double DEG_2_RAD = 0.01745329251;
};

#endif // IMU_H
