int8_t count1 = 0;
int8_t count2 = 0;
int8_t last_count1 = 1;
int8_t last_count2 = 1;
uint8_t last11, last12, last21, last22;
uint32_t error1 = 0;
uint32_t error2 = 0;

#define SPEED 100
#define ANGLE_SCALE 20000
#define STEPS_PER_RADIAN 410
#define ENCODER_CALIBRATION1 160
#define RADIUS 10000000L
#define FOLLOW_MAX_Y 13000000L
#define FOLLOW_MAX_S 15000L

#define sign(x) ((x)<0?-1:1)
#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))

int16_t calibration_count1 = 0;
int16_t c=ANGLE_SCALE;
int16_t s=0;
int32_t x=0, y=0;
uint8_t state = 0;

// Encoder 1 uses INT0 and INT1
ISR(INT1_vect,ISR_ALIASOF(INT0_vect));
ISR(INT0_vect)
{
  uint8_t new11 = ((PIND & 0x01) != 0);
  uint8_t new12 = ((PIND & 0x02) != 0);
  
  count1 += (last11 ^ new12) - (int)(new11 ^ last12);
  
  if((last11 ^ new11) & (last12 ^ new12))
    error1 ++;
    
  last11 = new11;
  last12 = new12;
}

// Encoder two uses PCINT4 and PCINT7 (which trigger the PCINT0 interrupt)  
ISR(PCINT0_vect)
{
  uint8_t new21 = ((PINB & 0x10) != 0);
  uint8_t new22 = ((PINB & 0x80) != 0);
  
  count2 += (last21 ^ new22) - (int)(new21 ^ last22);  
  
  if((last21 ^ new21) & (last22 ^ new22))
    error2 ++;
  
  last21 = new21;
  last22 = new22;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT);
  cli();
  PCMSK0 = 0x90; // enable pin change interrupts on PCINT4 and 7 which are Arduino pins 8, 11, PB4 and PB7
  PCICR = 0xff; // turns on pin change interrupts in general
  PCIFR = 0; // clear interrupt flags
  
  EICRA = 0x05; // set INT0 and INT to interrupt on all edges
  EIMSK = 0x03; // enable INT0 and INT1
  EIFR = 0; // clear interrupt flags
  
  // 20kHz PWM copied from Zumo shield library
  TCCR1A = 0b10100000;
  TCCR1B = 0b00010001;
  ICR1 = 400;
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  
  sei();
  
  setMotors(0,0);
  
  delay(200);
}

void setMotors(int left, int right)
{
  if(left < 0)
  {
    digitalWrite(4, LOW);
    OCR1A = min(-left, 400);
  }
  else
  {
    digitalWrite(4, HIGH);
    OCR1A = min(left, 400);
  }
  
  if(right < 0)
  {
    digitalWrite(5, LOW);
    OCR1B = min(-right, 400);
  }
  else
  {
    digitalWrite(5, HIGH);
    OCR1B = min(right, 400);
  }
}

int16_t divide(int16_t a, int16_t b)
{
  return (a + sign(a)*(b/2-1)) / b;
}

void ticks1(int8_t n) {
  n = sign(n);
  
  int16_t dc = + divide(n*s - n*c/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  int16_t ds = - divide(n*c + n*s/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  
  c += dc;
  s += ds;
  x += n * c;
  y += n * s;
  
  last_count1 += n;
  
  // do it again every X times
  calibration_count1 ++;
  if(calibration_count1 >= ENCODER_CALIBRATION1 || calibration_count1 <= -ENCODER_CALIBRATION1)
  {
    last_count1 -= n;
    calibration_count1 = 0;
  }
}

void ticks2(int8_t n) {
  n = sign(n);
  
  int16_t dc = - divide(n*s + n*c/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  int16_t ds = + divide(n*c - n*s/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  c += dc;
  s += ds;
  x += n * c;
  y += n * s;
  
  last_count2 += n;
}

void positionUpdate() {
  if(count1 != last_count1) ticks1(count1 - last_count1);
  if(count2 != last_count2) ticks2(count2 - last_count2);
}

void goHome() {
  int16_t speed = SPEED;
  int32_t err;
  
  if(x > -20000000)
    speed = speed/2;
  
  if(c < 0)
  {
    // pointed backwards
    err = (s > 0 ? speed/2 : -speed/2);
  }
  else
  {    
    int32_t target_s = -max(min(y / 10000 * FOLLOW_MAX_S / (FOLLOW_MAX_Y / 10000), FOLLOW_MAX_S), -FOLLOW_MAX_S);
    err = (s - target_s)/100;
    err = max(min(err,speed),-speed);
  }
  if(err > 0)
    setMotors(speed, speed - err);
  else
    setMotors(speed + err, speed);
}

// direct-to the origin
void transform() {
  x -= RADIUS;
  
  double r = hypot((double)x, (double)y);
  double nx = (double)x/r;
  double ny = (double)y/r;
  int16_t new_c = -nx*c-ny*s;
  int16_t new_s = ny*c-nx*s;
  c = new_c;
  s = new_s;
  y = 0;
  x = -r;
  
  x += RADIUS;
}

uint16_t last_millis = 0;
uint8_t led = 0;

void encoderUpdate() {
  if(last_count1 != count1 || last_count2 != count2)
  {
    positionUpdate();
  }
}

uint16_t last_on_line_millis = 0;
uint16_t on_line_start_millis = 0;

uint8_t onLine()
{
  return analogRead(1) > 700 || analogRead(0) > 700;
}

int16_t readLine()
{  
  int16_t s1 = max(analogRead(1), 640) - 640;
  int16_t s0 = max(analogRead(0), 640) - 640;
  
  static int16_t last_value = 0;
  
  if(s1 > 20 || s0 > 20)
  {
    last_on_line_millis = millis();
    last_value = 1000*(int32_t)(s1 - s0)/(s1+s0); // positive is to the right of the line
  }
  else
  {
    // lost line
    last_value = sign(last_value)*1000;
  }
  return last_value;
}

void followLine()
{
  static int16_t last_p = 0;
  int16_t p = readLine();
  int16_t d = p - last_p;
  last_p = p;
  static int32_t i = 0;
  
  i += p;
  i = max(min(p, 1000000), -1000000);
  
  int16_t pid = p/15 + d*5 + i/50;
  pid = max(min(pid, SPEED), -SPEED);
  if(pid < 0)
  {
    setMotors(SPEED, SPEED + pid);
  }
  else
  {
    setMotors(SPEED - pid, SPEED);
  }
  
  last_p = p;
}

uint16_t getBatteryVoltage_mv() {
  return analogRead(2) * 10;
}

void debug() {  
  if(millis() - last_millis > 100)
  {
    led = !led;
    digitalWrite(13, led);
    
    Serial.print(getBatteryVoltage_mv());
    Serial.write("mV\t");
    Serial.print(readLine());
    Serial.write(" ");
    Serial.print(analogRead(0));
    Serial.write(" ");
    Serial.print(analogRead(1));
    Serial.write("\t");
    Serial.print(c);
    Serial.write(",");
    Serial.print(s);
    Serial.write("\t");
    Serial.print(x);
    Serial.write(",");
    Serial.print(y);
    Serial.write("\t");
    Serial.print(error1);
    Serial.write(",");  
    Serial.print(error2);
    
    Serial.println("");
    last_millis += 100;
  }
}

void loop() {
  static uint16_t battery_voltage_low_millis = 0;
  encoderUpdate();
  switch(state) {
  case 0:
    if(getBatteryVoltage_mv() < 3000)
      battery_voltage_low_millis = millis();
    if(millis() - battery_voltage_low_millis > 1000)
      state++;
    debug();
    break;
  case 1:
    if(onLine())
      {
      state++;
      on_line_start_millis = millis();
    }
    setMotors(100,100);
    break;
  case 2:
    followLine();
    if(millis() - on_line_start_millis > 5000)
      state++;
    break;
  case 3:
    followLine();
    if(millis() - last_on_line_millis > 500)
    {
      transform();
      state++;
    }
    break;
  case 4:
    goHome();
    if(x > -1500000)
    {
      state++;
    }
    break;
  case 5:
    setMotors(0,0);
    debug();
    break;
  }
}
