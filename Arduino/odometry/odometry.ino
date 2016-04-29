/*Read encoders attached to the front left and right motor of K.H.A.N
Left encoders: pin 3
Right encoder: pin 2*/
int leftE = 1; //pin 3
int rightE = 0; //Pin 2
//CUrrent ticks
volatile int left_ticks = 0;
volatile int right_ticks = 0;
//Previous ticks
int prev_right_tick = 0;
int prev_left_tick = 0;

//Robot dimension in mm
//wheel radius
const int R = 30;
//Robot length
const int L = 185; 

//Some useful constants
const float Pi = 3.14159;
const float N = 1000/3; //ticks per revolution
float m_per_ticks = 2*Pi*R/N;
float theta, theta_new;
int x, y, x_new, y_new;
//int d_tick_l, d_tick_r;



// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  Serial.begin(115200);
  pinMode(leftE, INPUT);
  pinMode(rightE, INPUT);
  attachInterrupt(leftE, left_count, RISING);  
  attachInterrupt(rightE, right_count, RISING);
  

}

// the loop routine runs over and over again forever:
void loop() {
  Serial.print("Left encoder tick count is:");
  Serial.println(left_ticks);
  Serial.print("Right encoder tick count is:");
  Serial.println(right_ticks);
  int d_tick_l = left_ticks - prev_left_tick;
  int d_tick_r = right_ticks - prev_right_tick;
  state_estimate(d_tick_l, d_tick_r);
  Serial.print("x estimate is: ");
  Serial.println(x);
  Serial.print("y estimate is:");
  Serial.println(y);
  Serial.print("theta estimate is:");
  Serial.println(theta);
  delay(1000);
}

//-------Function definitions----------------
void state_estimate(int d_tick_l,int d_tick_r)
{
  Serial.println("estimating states!");
  float Dr = m_per_ticks*d_tick_r;
  float Dl = m_per_ticks*d_tick_l;
  float Dc = (Dr+Dl)/2;
  float x_dt = Dc*cos(theta);
  float y_dt = Dc*sin(theta);
  float theta_dt = (Dr-Dl)/L;
  
  theta_new = theta+theta_dt;
  x_new = x + x_dt;
  y_new = y+y_dt;
  
  prev_left_tick = left_ticks;
  prev_right_tick = right_ticks;
  //return 0;
}
  
//-------ISR---------------------
//Left encoder count
void left_count(){
  left_ticks = left_ticks + 1;
  Serial.println("Left tick!!");
}
//Right encoder counts
void right_count(){
  right_ticks = right_ticks + 1;
  Serial.println("Right tick!!");
}
