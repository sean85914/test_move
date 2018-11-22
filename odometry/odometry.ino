/* Using encoder to calculate the linear velocity of the motors
 * and transport them through serial to PI for control and dead  
 * reckoning, for usage of differential-drive mobile robot.
 * Developed by Sean Lu at Nov., 2018. 
 */
#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define RADIUS 0.032 // Wheel radius, in meter
#define CPR 2970.0   // Encoder Counts Per Revolution

volatile long encoder_pre_L, encoder_pre_R; // present
volatile long encoder_pos_L, encoder_pos_R; // post
long time_;
double hz = 50; // 50Hz

void setup()
{
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  encoder_pre_L = 0; encoder_pre_R = 0;
  encoder_pos_L = 0; encoder_pos_R = 0;
  Serial.begin(57600);
  // Regist interrupt callback functions
  // UNO pin 2, call Encoder_L when the signal is rising
  attachInterrupt(0, Encoder_L, RISING);
  // UNO pin 3, call Encoder_R when the signal is rising
  attachInterrupt(1, Encoder_R, RISING);
  time_ = millis();
}

void loop()
{
  if(millis() - time_ >= 1000/hz) {
    long dt = (millis() - time_); // Time difference, in ms
    time_ = millis(); // Update time
    // Calculate linear velocity
    double v_l = (encoder_pre_L - encoder_pos_L)*2*PI/CPR * RADIUS / dt * 1000;
    double v_r = (encoder_pre_R - encoder_pos_R)*2*PI/CPR * RADIUS / dt * 1000;
    // Update encoder
    encoder_pos_L = encoder_pre_L;
    encoder_pos_R = encoder_pre_R;
    // Print data in serial
    // Data format:
    // v_r v_l
    Serial.print(v_r); Serial.print(" ");
    Serial.println(v_l);
  }
}

void Encoder_L()
{
  if(digitalRead(SPD_INT_L2) == HIGH) --encoder_pre_L;
  else ++encoder_pre_L;
}

void Encoder_R()
{
  if(digitalRead(SPD_INT_R2) == HIGH) --encoder_pre_R;
  else ++encoder_pre_R;
} 
