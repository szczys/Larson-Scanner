#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

//User defined setting:
unsigned int fadeDelay = 624; // 1=32uSecs, 65535(max)=~2.1 secs
unsigned char fadeResolution = 8; //Fade steps (between 1 and 255)


//Variables
volatile unsigned char pwmValues[8] = {0x00,0x20,0x40,0x60,0x80,0xA0,0xC0,0xE0};
volatile unsigned char ledBuffer = 0x00;
unsigned int fadeCount = 0;
volatile unsigned char fadeFlag = 0;

void fade(void)
{
  //Systematically reduces all non-zero pwmValues
  for (unsigned char i=0; i<8; i++) {
    if (pwmValues[i] > 0) {
      if (pwmValues[i] < fadeResolution) pwmValues[i] = 0;
      else pwmValues[i] -= fadeResolution;
    }
  }
}

int main(void) 
{
  //Setup IO
  DDRD = 0xFF;
  PORTD = 0xFF;

  //Setup Timer
  cli();		//disable all interrupts
  TCCR0B |= (1<<CS00);	//Start timer with no prescaler
  TIMSK0 |= (1<<TOIE0);	//Enable the overflow interrupt
  sei();		//enable all interrupts
  
  

  while(1)
  {
    if (fadeFlag)	//Targer time has passed; time to do something
    {
      fade();
      fadeFlag = 0;
    }
    if (pwmValues[7] == 0) {	//Turn up all values
      for (unsigned char i=0; i<8; i++) pwmValues[i] = 0xFF;
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
}
