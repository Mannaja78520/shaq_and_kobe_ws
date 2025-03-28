#include "imu_bno055.h"

IMU_BNO055::IMU_BNO055() : bno(Adafruit_BNO055(55, 0x29)) {}

bool IMU_BNO055::init() {
  if (!bno.begin()) {
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    return false;
  }
  bno.setExtCrystalUse(true);
  return true;
}

bool IMU_BNO055::getQuaternion(float &qx, float &qy, float &qz, float &qw) {
  imu::Quaternion quat = bno.getQuat();
  qx = (float)quat.x();
  qy = (float)quat.y();
  qz = (float)quat.z();
  qw = (float)quat.w();
  return true;
}

bool IMU_BNO055::getAngVelocity(float &wx, float &wy, float &wz) {
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  wx = angVelocityData.gyro.x;
  wy = angVelocityData.gyro.y;
  wz = angVelocityData.gyro.z;  
  return true;
}

bool IMU_BNO055::getLinearAcc(float &ax, float &ay, float &az) {
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ax = linearAccelData.acceleration.x;
  ay = linearAccelData.acceleration.y;
  az = linearAccelData.acceleration.z;  
  return true;
}

bool IMU_BNO055::getMagnetometer(float &mx, float &my, float &mz) {
  bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mx = magnetometerData.magnetic.x;
  my = magnetometerData.magnetic.y;
  mz = magnetometerData.magnetic.z;  
  return true;
}

bool IMU_BNO055::getAngle(float &yaw, float &pitch, float &roll) {
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  roll = orientationData.orientation.roll;
  pitch = orientationData.orientation.pitch;
  yaw = orientationData.orientation.heading;
  return true;
}

void IMU_BNO055::getIMUData(sensor_msgs__msg__Imu &imu_msg, sensor_msgs__msg__MagneticField &mag_msg, geometry_msgs__msg__Twist &pos_angle_msg) {
  float qx, qy, qz, qw;
  getQuaternion(qx, qy, qz, qw);
  imu_msg.orientation.x = qx;
  imu_msg.orientation.y = qy;
  imu_msg.orientation.z = qz;
  imu_msg.orientation.w = qw;
  float ax, ay, az;
  getLinearAcc(ax, ay, az);
  imu_msg.linear_acceleration.x = ax;
  imu_msg.linear_acceleration.y = ay;
  imu_msg.linear_acceleration.z = az;
  float wx, wy, wz;
  getAngVelocity(wx, wy, wz);
  imu_msg.angular_velocity.x = wx;
  imu_msg.angular_velocity.y = wy;
  imu_msg.angular_velocity.z = wz;
  float mx, my, mz;
  getMagnetometer(mx, my, mz);
  mag_msg.magnetic_field.x = mx;
  mag_msg.magnetic_field.y = my;
  mag_msg.magnetic_field.z = mz;

  // Convert degrees to radians
  float roll, pitch, yaw;
  getAngle(roll, pitch, yaw);
  pos_angle_msg.angular.x = roll;
  pos_angle_msg.angular.y = pitch;
  pos_angle_msg.angular.z = yaw;
  // roll *= DEG_2_RAD;
  // pitch *= DEG_2_RAD;
  // yaw *= DEG_2_RAD;
  
  // // Compute the rotation matrix
  // double R[3][3] = {
  //     {
  //         cos(yaw) * cos(pitch),
  //         cos(yaw) * sin(pitch) * sin(roll) - sin(yaw) * cos(roll),
  //         cos(yaw) * sin(pitch) * cos(roll) + sin(yaw) * sin(roll)
  //     },
  //     {
  //         sin(yaw) * cos(pitch),
  //         sin(yaw) * sin(pitch) * sin(roll) + cos(yaw) * cos(roll),
  //         sin(yaw) * sin(pitch) * cos(roll) - cos(yaw) * sin(roll)
  //     },
  //     {
  //         -sin(pitch),
  //         cos(pitch) * sin(roll),
  //         cos(pitch) * cos(roll)
  //     }
  // };

  // // Transform local acceleration to world frame
  // double accel_world_x = R[0][0] * ax + R[0][1] * ay + R[0][2] * az;
  // double accel_world_y = R[1][0] * ax + R[1][1] * ay + R[1][2] * az;
  // double accel_world_z = R[2][0] * ax + R[2][1] * ay + R[2][2] * az;

  // accel_world_z -= 9.81;
  

  // //velocity = accel*dt (dt in seconds)
  // //position = 0.5*accel*dt^2
  // double ACCEL_POS_TRANSITION = 0.5 * time_diff * time_diff;
  // // Update position using velocity and acceleration
  // xPos += (vX * time_diff) + (accel_world_x * ACCEL_POS_TRANSITION);
  // yPos += (vY * time_diff) + (accel_world_y * ACCEL_POS_TRANSITION);
  // zPos += (vZ * time_diff) + (accel_world_z * ACCEL_POS_TRANSITION);

  // vX += accel_world_x * time_diff;
  // vY += accel_world_y * time_diff;
  // vZ += accel_world_z * time_diff;
  // velocity of sensor in the direction it's facing
  // orientation.x from event is yaw heading from BNO in degree 0-360 degrees
  // headingVel = time_diff * accel.x() / cos(DEG_2_RAD * event.orientation.x);

  // pos_angle_msg.linear.x = xPos;
  // pos_angle_msg.linear.y = yPos;
  // pos_angle_msg.linear.z = zPos;
}

// void IMU_BNO055::resetIMU(){
  
// }