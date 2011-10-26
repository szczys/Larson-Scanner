#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

//User defined setting:
unsigned int fadeDelay = 400; // 1=32uSecs, 65535(max)=~2.1 secs
unsigned char fadeResolution = 8; //Fade steps (between 1 and 255)
unsigned int scanDelay = 3000; // Scanning speed (timing same as fadeDelay)


//Variables
volatile unsigned char pwmValues[8] = {0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00};	//PWM value storage
volatile unsigned char ledBuffer = 0x00;	//Buffer for prewinding PWM values
unsigned int fadeCount = 0;			//Counter used by interrupt to time fading
volatile unsigned char fadeFlag = 0;		//Flag set by interrupt for main loop fade timing
unsigned char scanBuffer = 0x01;		//Start scanning from bit 0
unsigned char scanDirection = 1;		//1=shift left, 0=shift right
unsigned int scanCount = 0;			//Counter used by interrupt to time scanning
volatile unsigned char scanFlag = 0;		//Flag set by interrupt for main loop scan timing

void fade(void)
{
  //Systematically reduces all non-zero pwmValues
  for (unsigned char i=0; i<8; i++) {
    if (pwmValues[i] > 0) {
      if (~((1<<i) & scanBuffer)) { //Do not dim active LEDs from scanBuffer
        if (pwmValues[i] < fadeResolution) pwmValues[i] = 0;
        else pwmValues[i] -= fadeResolution;
      }
    }
  }
}

void larson_scanner(void) {
  //Adjust scan direction if necessary
  if ((1<<0) & scanBuffer) scanDirection = 1;
  else if ((1<<7) & scanBuffer) scanDirection = 0;

  //Shift the scanBuffer
  if (scanDirection) scanBuffer <<= 1;
  else scanBuffer >>= 1;

  //Write new values to PWM buffer
  for (unsigned char i=0; i<8; i++) {
    if ((1<<i) & scanBuffer) pwmValues[i] = 0xFF;
  }
}

int main(void) 
{
  //Setup IO
  DDRD = 0xFF;
  PORTD = scanBuffer;

  //Setup Timer
  cli();		//disable all interrupts
  TCCR0B |= (1<<CS00);	//Start timer with no prescaler
  TIMSK0 |= (1<<TOIE0);	//Enable the overflow interrupt
  sei();		//enable all interrupts
  
  

  while(1)
  {
    if (fadeFlag) {	//Targed time has passed; time to do something
      fade();
      fadeFlag = 0;
    }
    if (scanFlag) {	//Time to shift the scanning buffer
      larson_scanner();
      scanFlag = 0;
    }
  }
}

ISR(TIMER0_OVF_vect)	//Timer0 overflow interrupt handler
{
  PORTD = ledBuffer;	//Set outputs prewound from last interrupt
  
  static unsigned char pwmCount = 255;	//Track overflows to compare with pwmValue

  //Increment counter an prewind pin values for next interrupt
  if (++pwmCount == 0)
  {
    ledBuffer = 0x00;
    for (unsigned char i=0; i<8; i++)
    {
      if (pwmValues[i] != 0) ledBuffer |= (1<<i);
    }
  }
  else
  {
    for (unsigned char i=0; i<8; i++)
    {
      if (pwmCount > pwmValues[i]) ledBuffer &= ~(1<<i);
    }
  }
 
  //Track time for fading effect
  if (++fadeCount > fadeDelay) {
    fadeFlag += 1;
    fadeCount = 0;
  }

  //Track time for scanning effect
  if (++scanCount > scanDelay) {
    scanFlag += 1;
    scanCount = 0;
  }
}
