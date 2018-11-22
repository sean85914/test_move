#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define WIDTH 0.179
#define RADIUS 0.032
#define CPR 2970.0

volatile long encoder_pre_L, encoder_pre_R; // present
volatile long encoder_pos_L, encoder_pos_R; // post
long time_;
double theta = 0;
double hz = 100; // 100Hz

void setup()
{
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  encoder_pre_L = 0; encoder_pre_R = 0;
  encoder_pos_L = 0; encoder_pos_R = 0;
  Serial.begin(57600);
  // regist interrupt callback function
  // UNO pin 2, call Encoder_L when the signal is rising
  attachInterrupt(0, Encoder_L, RISING);
  // UNO pin 3, call Encoder_R when the signal is rising
  attachInterrupt(1, Encoder_R, RISING);
  time_ = millis();
}

void loop()
{
  if(millis() - time_ >= 1000/hz) {
    long dt = (millis() - time_); // ms
    double s_l = (encoder_pre_L - encoder_pos_L)*2*PI/CPR * RADIUS, v_l = s_l/dt * 1000;
    double s_r = (encoder_pre_R - encoder_pos_R)*2*PI/CPR * RADIUS, v_r = s_r/dt * 1000;
    theta += (s_r-s_l)/WIDTH;
    if(theta>2*PI)  theta-=2*PI;
    if(theta<0) theta+=2*PI;
    // update encoder
    encoder_pos_L = encoder_pre_L;
    encoder_pos_R = encoder_pre_R;
    // print data in serial
    // data format:
    // v_r v_l theta
    Serial.print(v_r); Serial.print(" ");
    Serial.print(v_l); Serial.print(" ");
    Serial.println(theta);
    time_ = millis();
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
