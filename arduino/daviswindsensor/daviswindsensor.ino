/*
  Analog input, analog output, serial output
 
 Reads an analog input pin, maps the result to a range from 0 to 255
 and uses the result to set the pulsewidth modulation (PWM) of an output pin.
 Also prints the results to the serial monitor.
 
 The circuit:
 * potentiometer connected to analog pin 0.
   Center pin of the potentiometer goes to the analog pin.
   side pins of the potentiometer go to +5V and ground
 * LED connected from digital pin 9 to ground
 
 created 29 Dec. 2008
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 
 */

// These constants won't change.  They're used to give names
// to the pins used:
const int analogInPin = A4;  // Analog input pin that the potentiometer is attached to

void setup() {
  Serial.begin(38400);
  
  attachInterrupt(0, isr_count, FALLING);
  pinMode(2, INPUT_PULLUP);
  
}

volatile unsigned int rotation_count;
volatile unsigned long lastt;
volatile unsigned long lastperiod;
void isr_count()
{
  unsigned long t  = millis();
  if(t - lastt > 15){
    rotation_count++;
    lastperiod = t - lastt;
    lastt = t;
  }
}

static float lpdir, lp = .05;
int cnt;
void loop() {
  // read the analog in value:
  int  sensorValue = analogRead(analogInPin);            

  float dir = sensorValue / 1024.0 * 360;
  lpdir = lp*dir + (1-lp)*lpdir;

  // print the results to the serial monitor:
  static float knots;
  static int nowind;
  if(++cnt == 100) {
    if(rotation_count > 0) {
        rotation_count = 0;
        nowind = 0;
    } else
        nowind++;

    if(nowind == 20)
      knots = 0, nowind--;
    else
      knots = .868976 * 2.25 * 1000 / lastperiod;

    char buf[128];
    snprintf(buf, sizeof buf, "APMWV,%d.%d,R,%d.%d,N,A", (int)lpdir, (int)(lpdir*10)%10, (int)knots, (int)(knots*10)%10);
    
    uint8_t cksum = 0;
    for(int i=0; i<strlen(buf); i++)
        cksum ^= buf[i];
     
    char buf2[128];
    snprintf(buf2, sizeof buf2, "$%s*%02x\r\n", buf, cksum);
    Serial.print(buf2);

    cnt = 0;
  }
  delay(2);                     
}

