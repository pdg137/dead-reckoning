int8_t count1 = 0;
int8_t count2 = 0;
int8_t last_count1 = 1;
int8_t last_count2 = 1;
uint8_t last11, last12, last21, last22;
uint8_t error1 = 0;
uint8_t error2 = 0;

// Encoder 1 uses INT0 and INT1
ISR(INT1_vect,ISR_ALIASOF(INT0_vect));
ISR(INT0_vect)
{
  uint8_t new11 = ((PIND & 0x01) != 0);
  uint8_t new12 = ((PIND & 0x02) != 0);
  
  count1 += (last11 ^ new12) - (int)(new11 ^ last12);
  
  if((last11 ^ new11) & (last12 ^ new12))
    error1 = 1;
    
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
    error2 = 1;
  
  last21 = new21;
  last22 = new22;
}

ISR(PCINT1_vect,ISR_ALIASOF(PCINT0_vect));
ISR(PCINT2_vect,ISR_ALIASOF(PCINT0_vect));
#ifdef PCINT3_vect
ISR(PCINT3_vect,ISR_ALIASOF(PCINT0_vect));
#endif

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
}

#define min(a, b) ((a)<(b)?(a):(b))
#define max(a, b) ((a)>(b)?(a):(b))

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

// 30000:1
#define ANGLE_SCALE 20000
#define STEPS_PER_RADIAN 410
int16_t c=-ANGLE_SCALE;
int16_t s=0;
int32_t x=0, y=0; //200000000L;

#define sign(x) ((x)<0?-1:1)

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
//  c = max(-ANGLE_SCALE, min(ANGLE_SCALE, c));
//  s = max(-ANGLE_SCALE, min(ANGLE_SCALE, s));
  
  last_count1 += n;  
}

void ticks2(int8_t n) {
  if(n > 1)
    n = 1;
  if(n < -1)
    n = -1;
  
  int16_t dc = - divide(n*s + n*c/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  int16_t ds = + divide(n*c - n*s/2/STEPS_PER_RADIAN, STEPS_PER_RADIAN);
  c += dc;
  s += ds;
  x += n * c;
  y += n * s;
//  c = max(-ANGLE_SCALE, min(ANGLE_SCALE, c));
//  s = max(-ANGLE_SCALE, min(ANGLE_SCALE, s));
  
  last_count2 += n;
}

void positionUpdate() {
  if(count1 != last_count1) ticks1(count1 - last_count1);
  if(count2 != last_count2) ticks2(count2 - last_count2);
}

int32_t err;

// 13_000_000 is about 10 cm
#define FOLLOW_MAX_Y 13000000L
#define FOLLOW_MAX_S 15000L

void followLine() {
  if(c < 0)
  {
    // pointed backwards
    err = (s > 0 ? 50 : -50);
  }
  else
  {
    int32_t target_s = -max(min(y / 10000 * FOLLOW_MAX_S / (FOLLOW_MAX_Y / 10000), FOLLOW_MAX_S), -FOLLOW_MAX_S);
    err = (s - target_s)/200;
    err = max(min(err,50),-50);
  }
  if(err > 0)
    setMotors(100, 100 - err);
  else
    setMotors(100 + err, 100);
}

uint16_t last_millis = 0;

void encoderUpdate() {
  if(last_count1 != count1 || last_count2 != count2)
  {
    positionUpdate();
    followLine();
  }
  
  if(millis() - last_millis > 100)
  {
    Serial.print(s);
    Serial.write(",");
    Serial.print(c);
    Serial.write("\t");
    Serial.print(x);
    Serial.write(",");
    Serial.print(y);
    Serial.write("\t");
    Serial.print(err);
    
    Serial.println("");
    last_millis += 100;
  }
  
/*  if(s > ANGLE_SCALE/2 || s < -ANGLE_SCALE/2)
  {
    setMotors(0,0);
    Serial.println("Lost");
    while(1);
  }*/
  
  if(error1)
  {
    Serial.println("Error 1");
    error1 = 0;
  }
  if(error2)
  {
    Serial.println("Error 2");
    error2 = 0;
  }
}

void loop() {
  encoderUpdate();
}
