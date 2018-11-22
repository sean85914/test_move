#define SPD_INT_L2 8
#define SPD_INT_R2 9
#define WIDTH 0.179
#define RADIUS 0.032
#define CPR 2970.0

volatile long encoder_pre_L, encoder_pre_R; // present
volatile long encoder_pos_L, encoder_pos_R; // post
long time_;
double theta;
double hz = 50; // 50Hz

void setup()
{
  pinMode(SPD_INT_L2, INPUT);
  pinMode(SPD_INT_R2, INPUT);
  theta = 0;
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
    time_ = millis();
    double v_l = (encoder_pre_L - encoder_pos_L)*2*PI/CPR * RADIUS / dt * 1000;
    double v_r = (encoder_pre_R - encoder_pos_R)*2*PI/CPR * RADIUS / dt * 1000;
    // update encoder
    encoder_pos_L = encoder_pre_L;
    encoder_pos_R = encoder_pre_R;
    // print data in serial
    // data format:
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
