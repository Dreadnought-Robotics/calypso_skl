#include <ros.h>

#include <calypso_msgs/gin.h>
#include <Wire.h>
#include <Servo.h>

Servo t_left_servo;
Servo t_right_servo; 
Servo d_left_servo;
Servo d_right_servo;

int t_left, t_right, d_left, d_right;

void pub_rpm(calypso_msgs::gin& rpms) {
  
  t_left = rpms.t1;
  t_right = rpms.t2;
  d_left = rpms.d1;
  d_right = rpms.d2;

}

ros::NodeHandle getting_rpm;
ros::Subscriber<calypso_msgs::gin> rpm_sub("/rosetta/gin", &pub_rpm);

void setup() {

  getting_rpm.initNode();
  getting_rpm.getHardware()->setBaud(57600);
  getting_rpm.subscribe(rpm_sub);

  t_left_servo.attach(5);
  t_right_servo.attach(6); 
  d_left_servo.attach(9);
  d_right_servo.attach(10);

  t_left_servo.writeMicroseconds(1800);
  t_right_servo.writeMicroseconds(1800); 
  d_left_servo.writeMicroseconds(1800);
  d_right_servo.writeMicroseconds(1800);

  t_left_servo.writeMicroseconds(1500);
  t_right_servo.writeMicroseconds(1500); 
  d_left_servo.writeMicroseconds(1500);
  d_right_servo.writeMicroseconds(1500);

  delay(2000);

}

void loop() {

  t_left_servo.writeMicroseconds(t_left);
  t_right_servo.writeMicroseconds(t_right); 
  d_left_servo.writeMicroseconds(d_left);
  d_right_servo.writeMicroseconds(d_right);  
  getting_rpm.spinOnce();

}
