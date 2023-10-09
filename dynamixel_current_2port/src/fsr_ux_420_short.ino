//roslaunch dynamixel_current_2port dynamixel_current_2port
//rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600


// //ROS_Arduino
// #include <ros.h>
// #include <std_msgs/UInt8.h>

// ros::NodeHandle nh;

// std_msgs::UInt8 fsr_msg_L;
// std_msgs::UInt8 fsr_msg_R;

// ros::Publisher pub_fsr_L("FSR_L", &fsr_msg_L);
// ros::Publisher pub_fsr_R("FSR_R", &fsr_msg_R);

// char SensorPin_L = A0; //analog pin 0
// char SensorPin_R = A1; //analog pin 1
 
// void setup(){
//   nh.getHardware()->setBaud(57600);
// //  Serial.begin(57600);
//   nh.initNode();
//   nh.advertise(pub_fsr_L);
//   nh.advertise(pub_fsr_R);
// }

// void loop(){
//   uint8_t SensorReading_L = analogRead(SensorPin_L);
//   uint8_t SensorReading_R = analogRead(SensorPin_R);
// //  Serial.println(SensorReading_L);
// //  Serial.println(SensorReading_R);

//   uint8_t mfsr_r18_L = map(SensorReading_L, 0, 1024, 0, 255);
//   uint8_t mfsr_r18_R = map(SensorReading_R, 0, 1024, 0, 255);
  
//   fsr_msg_L.data = mfsr_r18_L;
//   fsr_msg_R.data = mfsr_r18_R;
  
//   pub_fsr_L.publish(&fsr_msg_L);
//   pub_fsr_R.publish(&fsr_msg_R);

//   nh.spinOnce();
// }
 

