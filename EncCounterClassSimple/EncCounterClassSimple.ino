
#include <ros.h>
#include "EncCounter.h"
#include "MotorControl.h"
#include <std_msgs/Int16MultiArray.h>

unsigned long oldtime = millis();
MotorControl motor_r(4,5);
MotorControl motor_l(6,7);
EncCounter enc_l(8,9);
EncCounter enc_r(10,11);
long counts_l;
long counts_r;
ros::NodeHandle nh;
std_msgs::Int16MultiArray enc_msg;
ros::Publisher odom_pub("arduino_odom", &enc_msg);
void moto_callback(std_msgs::Int16MultiArray);
ros::Subscriber<std_msgs::Int16MultiArray> sub("arduino_moto", &moto_callback );


void moto_callback(std_msgs::Int16MultiArray data){
// data[0] is direction of left moto, data[1] left wheel PWM 0-1, data[2] right direction, data [3] right PWM
    motor_l.run_moto(data.data[0], data.data[1]);
    motor_r.run_moto(data.data[2], data.data[3]);
}

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(odom_pub);
  nh.subscribe(sub);
}

void loop() {
  counts_l = enc_l.counter();
  counts_r = enc_r.counter();
  nh.spinOnce();
  if (millis() - oldtime > 20) {
    short diff_r = enc_r.calc_diff();
    short diff_l = enc_l.calc_diff();
    short time_diff = (millis() - oldtime);
    short value[3] = {diff_l,diff_r, time_diff};
    enc_msg.data_length = 3;
    enc_msg.data =  value;
    odom_pub.publish( &enc_msg );
    oldtime = millis();
  };
}
