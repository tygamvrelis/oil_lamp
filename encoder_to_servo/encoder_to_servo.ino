#include <Servo.h>

const int ch_a = 2; // CLK
const int ch_b = 3; // DT
const int sw = 4;
const int pwm = 9;

Servo roll;

volatile int pos = 0;
volatile bool pos_changed = false;
volatile bool aFlag = false;
volatile bool bFlag = false;

const int FULL_CIRCLE_RES = 20; // 20 positions for 360 degrees

// From https://www.instructables.com/id/Improved-Arduino-Rotary-Encoder-Reading/
void ch_a_callback(void)
{
  cli();
  byte a_value = digitalRead(ch_a);
  byte b_value = digitalRead(ch_b);
  if (a_value && b_value && aFlag)
  {
    --pos;
    pos_changed = true;
    aFlag = false;
    bFlag = false;
  }
  else if (a_value)
  {
    bFlag = true;
  }
  sei();
}

void ch_b_callback(void)
{
  cli();
  byte a_value = digitalRead(ch_a);
  byte b_value = digitalRead(ch_b);
  if (a_value && b_value && bFlag)
  {
    ++pos;
    pos_changed = true;
    aFlag = false;
    bFlag = false;
  }
  else if (b_value)
  {
    aFlag = true;
  }
  sei();
}

void setup()
{
  pinMode(ch_a, INPUT_PULLUP);
  pinMode(ch_b, INPUT_PULLUP);
  pinMode(sw, INPUT);
  digitalWrite(sw, HIGH);

  roll.attach(pwm);
  
  Serial.begin(9600); // initialize serial communications at 9600 bps

  attachInterrupt(digitalPinToInterrupt(ch_a), ch_a_callback, RISING);
  attachInterrupt(digitalPinToInterrupt(ch_b), ch_b_callback, RISING);
}

double pos_to_degree(int position)
{
  return position * (360.0 / static_cast<double>(FULL_CIRCLE_RES));
}

void loop()
{
  if (pos_changed)
  {
    double angle = pos_to_degree(pos);
    pos_changed = false;
    Serial.println(angle);
    roll.write(angle + 90);
  }

  // Reset position to 0 if the button is pressed
  if (digitalRead(sw) == LOW)
  {
    pos_changed = true;
    pos = 0;
  }
}
