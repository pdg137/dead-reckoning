uint8_t x = HIGH;

int count1 = 0;
int count2 = 0;
int last_count1 = 1;
int last_count2 = 1;
uint8_t last11, last12, last21, last22;
int error1 = 0;
int error2 = 0;

ISR(PCINT0_vect)
{
    digitalWrite(13, 1);
    
  uint8_t new11 = ((PINB & 0x10) != 0);
  uint8_t new12 = ((PINB & 0x20) != 0);
  uint8_t new21 = ((PINB & 0x40) != 0);
  uint8_t new22 = ((PINB & 0x80) != 0);
  
  count1 += (last11 ^ new12) - (int)(new11 ^ last12);
  count2 += (last21 ^ new22) - (int)(new21 ^ last22);
  
  if((last11 ^ new11) & (last12 ^ new12))
    error1 = 1;
  
  if((last21 ^ new21) & (last22 ^ new22))
    error2 = 1;
  
  last11 = new11;
  last12 = new12;
  last21 = new21;
  last22 = new22;
    digitalWrite(13, 0);
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
  PCMSK0 = 0xf0; // enable pin change interrupts on PCINT4-7 which are Arduino pins 8-11, Port B 4-7
  PCICR = 0xff; // turns on pin change interrupts in general
  PCIFR = 0; // clear interrupt flags
  sei();
}

void loop() {
  if(last_count1 != count1 || last_count2 != count2)
  {
    Serial.print(count1);
    Serial.write(" ");
    Serial.println(count2);
    last_count1 = count1;
    last_count2 = count2;

  }
  
  
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
