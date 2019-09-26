#include <Arduino.h>
#include <stdint.h>
#include <HardwareSerial.h>

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/boot.h>
#include <util/delay.h>



#include <RCSwitch.h>

#define DATA_PIN 2
#define DIR_PIN 3
#define PWR_PIN 4

RCSwitch mySwitch = RCSwitch();

volatile static uint32_t spiread_rfkeys, spiread_rfkeys_buf;
// SPI interrupt routine
ISR (SPI_STC_vect)
{
    uint8_t *d = (uint8_t*)&spiread_rfkeys_buf;
    static uint8_t spipos;
    SPDR = d[3-spipos++];

    if(spipos >= 4) {
        spipos = 0;
        spiread_rfkeys_buf = spiread_rfkeys;
        spiread_rfkeys = 0;
    }
}


void setup() {
// turn on SPI in slave mode
    SPCR |= _BV(SPE);

    // turn on interrupts
    SPCR |= _BV(SPIE);

    pinMode(MISO, OUTPUT);

    Serial.begin(38400);

  pinMode(DATA_PIN, INPUT);
  pinMode(DIR_PIN, INPUT);
  pinMode(PWR_PIN, OUTPUT);
  digitalWrite(PWR_PIN, HIGH);
  
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2
}

void loop() {
    
  if (mySwitch.available()) {
    
    int value = mySwitch.getReceivedValue();
    
    if (value == 0) {
      Serial.print("Unknown encoding");
    } else {
   
        Serial.print("Received ");
      Serial.print( mySwitch.getReceivedValue() );
      Serial.print(" / ");
      Serial.print( mySwitch.getReceivedBitlength() );
      Serial.print("bit ");
      Serial.print("Protocol: ");
      Serial.println( mySwitch.getReceivedProtocol() );


      if(mySwitch.getReceivedBitlength() == 24)
          spiread_rfkeys = mySwitch.getReceivedValue();
      
#if 0
              unsigned int *raw = mySwitch.getReceivedRawdata();
        Serial.print("timings: ");
        for(int i=0; i<16; i++) {
            Serial.print(raw[i]);
                    Serial.print(" ");
        }
        Serial.println("");
#endif
    }
    
    mySwitch.resetAvailable();
    
  }

}
