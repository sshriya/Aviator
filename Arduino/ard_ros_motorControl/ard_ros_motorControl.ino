//---------autoBot - Arduino Interface -------------------------------
//Controls all four motors on Rover 5
//ROS interfacing
//Subscribes to two ros topics that publishes velocity commands as joint state message

#include <ros.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/JointState.h>

ros::NodeHandle  nh;

//Pin configurations:
int right_front_1 = 4;
int right_front_2 = 2;
int right_front_pwm = 3;

int right_rear_1 = 6;
int right_rear_2 = 7;
int right_rear_pwm = 5;

int left_front_1 = 8;
int left_front_2 = 10;
int left_front_pwm = 9;

int left_rear_1 = 12;
int left_rear_2 = 13;
int left_rear_pwm = 11;

//define global variables 
float rightM_vel = 0.0;
float leftM_vel = 0.0;

void leftCb(const sensor_msgs::JointState& msg){
  //msg is a joint state message contatining the reuired velocity for left motors
  leftM_vel = msg.velocity[0];
}

void rightCb(const sensor_msgs::JointState& msg){
  //msg is a joint state message contatining the reuired velocity for left motors
  rightM_vel = msg.velocity[0];
}


ros::Subscriber<sensor_msgs::JointState> S1("/gBot/left_wheel/cmd", leftCb );
ros::Subscriber<sensor_msgs::JointState> S2("/gBot/right_wheel/cmd", rightCb );

void setup(){
  pinMode(right_front_1, OUTPUT);
  pinMode(right_front_2, OUTPUT);
  pinMode(right_rear_1, OUTPUT);
  pinMode(right_rear_2, OUTPUT); 
  pinMode(right_front_pwm, OUTPUT);
  pinMode(right_rear_pwm, OUTPUT);
  
  pinMode(left_front_1, OUTPUT);
  pinMode(left_front_2, OUTPUT);
  pinMode(left_rear_1, OUTPUT);
  pinMode(left_rear_2, OUTPUT); 
  pinMode(left_front_pwm, OUTPUT);
  pinMode(left_rear_pwm, OUTPUT);
  
  nh.initNode();
  nh.subscribe(S1);
  nh.subscribe(S2);
  //ros::Subscriber sub_mes = nh.subscribe("arduino_controller/left_wheel/cmd", 1, &leftCb);
}

void loop(){
  //moveRobot(vel_r, vel_l)
    nh.spinOnce();
    moveRobot(rightM_vel, leftM_vel); //Move forward
    delay(10);

}

void moveRobot(float vel_r, float vel_l){
  if ((vel_r > 0) && (vel_l > 0) ){
    moveForward(vel_r, vel_l);
  }
  else if((vel_r > 0) && (vel_l <0)){
    moveLeft(vel_r, vel_l);
  }
  else if((vel_r <0) && (vel_l >0)){
    moveRight(vel_r, vel_l);
  }
  else if ((vel_r < 0) && (vel_l <0)){
    moveBack(vel_r, vel_l);
  }
  else{
    motorStop();
  }
}
  
float moveForward(float i, float j){
    
    digitalWrite(right_front_1, HIGH); 
    digitalWrite(right_front_2, LOW);
    digitalWrite(right_rear_1, HIGH); 
    digitalWrite(right_rear_2, LOW);
    analogWrite(right_front_pwm, abs(i));
    analogWrite(right_rear_pwm, abs(i));
    
    digitalWrite(left_front_1, HIGH); 
    digitalWrite(left_front_2, LOW);
    digitalWrite(left_rear_1, HIGH); 
    digitalWrite(left_rear_2, LOW);
    analogWrite(left_front_pwm, abs(j));
    analogWrite(left_rear_pwm, abs(j));

}

int moveLeft(float i, float j){
  
    digitalWrite(right_front_1, HIGH); 
    digitalWrite(right_front_2, LOW);
    digitalWrite(right_rear_1, HIGH); 
    digitalWrite(right_rear_2, LOW);
    analogWrite(right_front_pwm, abs(i));
    analogWrite(right_rear_pwm, abs(i));
    
    digitalWrite(left_front_1, LOW); 
    digitalWrite(left_front_2, HIGH);
    digitalWrite(left_rear_1, LOW); 
    digitalWrite(left_rear_2, HIGH);
    analogWrite(left_front_pwm, abs(j));
    analogWrite(left_rear_pwm, abs(j));

}

int moveRight(float i, float j){

    digitalWrite(right_front_1, LOW); 
    digitalWrite(right_front_2, HIGH);
    digitalWrite(right_rear_1, LOW); 
    digitalWrite(right_rear_2, HIGH);
    analogWrite(right_front_pwm, abs(i));
    analogWrite(right_rear_pwm, abs(i));
    
    digitalWrite(left_front_1, HIGH); 
    digitalWrite(left_front_2, LOW);
    digitalWrite(left_rear_1, HIGH); 
    digitalWrite(left_rear_2, LOW);
    analogWrite(left_front_pwm, abs(j));
    analogWrite(left_rear_pwm, abs(j));
}

int moveBack(float i, float j){
    
    digitalWrite(right_front_1, LOW); 
    digitalWrite(right_front_2, HIGH);
    digitalWrite(right_rear_1, LOW); 
    digitalWrite(right_rear_2, HIGH);
    analogWrite(right_front_pwm, abs(i));
    analogWrite(right_rear_pwm, abs(i));
    
    digitalWrite(left_front_1, LOW); 
    digitalWrite(left_front_2,HIGH);
    digitalWrite(left_rear_1, LOW); 
    digitalWrite(left_rear_2, HIGH);
    analogWrite(left_front_pwm, abs(j));
    analogWrite(left_rear_pwm, abs(j));

}

void motorStop(){
    digitalWrite(right_front_1, LOW); 
    digitalWrite(right_front_2, LOW);
    digitalWrite(right_rear_1, LOW); 
    digitalWrite(right_rear_2, LOW);
    
    digitalWrite(left_front_1, LOW); 
    digitalWrite(left_front_2, LOW);
    digitalWrite(left_rear_1, LOW); 
    digitalWrite(left_rear_2, LOW);
  }


    
  
  
  
  
  
  
  
  
 
