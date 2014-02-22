uint8_t x = HIGH;

int count1 = 0;
int count2 = 0;
int last_count1 = 1;
int last_count2 = 1;
uint8_t last11, last12, last21, last22;
int error1 = 0;
int error2 = 0;

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

void loop() {
  
  if(count1 > 0 && count1 <= 50)
  {
    digitalWrite(4, LOW);
    OCR1A = count1*8;
  }
  else if(count1 < 0 && count1 >= -50)
  {
    digitalWrite(4, HIGH);
    OCR1A = -count1*8;
  }
  else
    OCR1A = 0;
  
  if(count2 > 0 && count2 <= 50)
  {
    digitalWrite(5, LOW);
    OCR1B = count2*8;
  }
  else if(count2 < 0 && count2 >= -50)
  {
    digitalWrite(5, HIGH);
    OCR1B = -count2*8;
  }
  else
    OCR1B = 0;
  
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
