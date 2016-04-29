/* Rotary encoder read example */
//left encoder definitions
#define ENC_A_l 14
#define ENC_B_l 15
#define ENC_PORT_l PINC

//right encoder definitions
#define ENC_A_r 8
#define ENC_B_r 9
#define ENC_PORT_r PINB
 
void setup()
{
  /* Setup encoder pins as inputs */
  pinMode(ENC_A_l, INPUT);
  digitalWrite(ENC_A_l, HIGH);
  pinMode(ENC_B_l, INPUT);
  digitalWrite(ENC_B_l, HIGH);
  
  pinMode(ENC_A_r, INPUT);
  digitalWrite(ENC_A_r, HIGH);
  pinMode(ENC_B_r, INPUT);
  digitalWrite(ENC_B_r, HIGH);
  Serial.begin (115200);
  Serial.println("Start");
}
 
void loop()
{
 static int32_t counter_l = 0;      //this variable will be changed by encoder input
 int8_t tmpdata_l;
 /**/
  tmpdata_l = read_encoder_l();
   
 static int32_t counter_r = 0;      //this variable will be changed by encoder input
 int8_t tmpdata_r;
 /**/
  tmpdata_l = read_encoder_l();
  tmpdata_r = read_encoder_r();
  if( tmpdata_l) {
    Serial.print("Left Counter value: ");
    Serial.println(counter_l, DEC);
    counter_l += tmpdata_l;
    Serial.print("Right Counter value: ");
    Serial.println(counter_r, DEC);
    counter_r += tmpdata_r;
  }
}
 
/* returns change in encoder state (-1,0,1) */
int8_t read_encoder_l()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT_l & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}

 
/* returns change in encoder state (-1,0,1) */
int8_t read_encoder_r()
{
  static int8_t enc_states[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2;                   //remember previous state
  old_AB |= ( ENC_PORT_r & 0x03 );  //add current state
  return ( enc_states[( old_AB & 0x0f )]);
}
