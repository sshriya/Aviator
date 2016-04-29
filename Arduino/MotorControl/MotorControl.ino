//---------autoBot - Arduino Interface -------------------------------
//Controls all four motors on Rover 5

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
}

void loop(){
  //moveRobot(vel_r, vel_l)
  moveRobot(0.1, 0.1); //Move forward
  delay(2000);
  /*moveRobot(-0.1, 0.1); //Turn Right
  delay(2000);
  moveRobot(0.1, -0.1); //Turn Left
  delay(2000);
  moveRobot(-0.1, -0.1); //Turn back
  delay(2000);
  moveRobot(0, 0); //Shrould stop
  */
  delay(2000);
}

void moveRobot(float vel_r, float vel_l){
  if (vel_r > 0 & vel_l > 0 ){
    moveForward(vel_r, vel_l);
  }
  else if(vel_r > 0 & vel_l <0){
    moveRight(vel_r, vel_l);
  }
  else if(vel_r <0 & vel_l >0){
    moveLeft(vel_r, vel_l);
  }
  else if (vel_r < 0 & vel_l <0){
    moveBack(vel_r, vel_l);
  }
  else{
    motorStop();
  }
}
  
int moveForward(float i, float j){
    int pwm_r = calculatePWM(i);
    int pwm_l = calculatePWM(j);
    
    digitalWrite(right_front_1, HIGH); 
    digitalWrite(right_front_2, LOW);
    digitalWrite(right_rear_1, HIGH); 
    digitalWrite(right_rear_2, LOW);
    analogWrite(right_front_pwm, pwm_r);
    analogWrite(right_rear_pwm, pwm_r);
    
    digitalWrite(left_front_1, HIGH); 
    digitalWrite(left_front_2, LOW);
    digitalWrite(left_rear_1, HIGH); 
    digitalWrite(left_rear_2, LOW);
    analogWrite(left_front_pwm, pwm_l);
    analogWrite(left_rear_pwm, pwm_l);

}

int moveRight(float i, float j){
    int pwm_r = calculatePWM(i);
    int pwm_l = calculatePWM(j);
    
    digitalWrite(right_front_1, HIGH); 
    digitalWrite(right_front_2, LOW);
    digitalWrite(right_rear_1, HIGH); 
    digitalWrite(right_rear_2, LOW);
    analogWrite(right_front_pwm, pwm_r);
    analogWrite(right_rear_pwm, pwm_r);
    
    digitalWrite(left_front_1, LOW); 
    digitalWrite(left_front_2, HIGH);
    digitalWrite(left_rear_1, HIGH); 
    digitalWrite(left_rear_2, LOW);
    analogWrite(left_front_pwm, pwm_l);
    analogWrite(left_rear_pwm, pwm_l);

}

int moveLeft(float i, float j){
    int pwm_r = calculatePWM(i);
    int pwm_l = calculatePWM(j);
    
    digitalWrite(right_front_1, LOW); 
    digitalWrite(right_front_2, HIGH);
    digitalWrite(right_rear_1, LOW); 
    digitalWrite(right_rear_2, HIGH);
    analogWrite(right_front_pwm, pwm_r);
    analogWrite(right_rear_pwm, pwm_r);
    
    digitalWrite(left_front_1, HIGH); 
    digitalWrite(left_front_2, LOW);
    digitalWrite(left_rear_1, HIGH); 
    digitalWrite(left_rear_2, LOW);
    analogWrite(left_front_pwm, pwm_l);
    analogWrite(left_rear_pwm, pwm_l);

}

int moveBack(float i, float j){
    int pwm_r = calculatePWM(i);
    int pwm_l = calculatePWM(j);
    
    digitalWrite(right_front_1, LOW); 
    digitalWrite(right_front_2, HIGH);
    digitalWrite(right_rear_1, LOW); 
    digitalWrite(right_rear_2, HIGH);
    analogWrite(right_front_pwm, pwm_r);
    analogWrite(right_rear_pwm, pwm_r);
    
    digitalWrite(left_front_1, LOW); 
    digitalWrite(left_front_2,HIGH);
    digitalWrite(left_rear_1, LOW); 
    digitalWrite(left_rear_2, HIGH);
    analogWrite(left_front_pwm, pwm_l);
    analogWrite(left_rear_pwm, pwm_l);

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

int calculatePWM(float vel){
  int Pwm = map(vel,-.20, .20, 0, 255);
  return Pwm;
}
    
  
  
  
  
  
  
  
  
 
