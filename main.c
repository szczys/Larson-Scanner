#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

volatile unsigned char pwmValues[8] = {0x00,0x20,0x40,0x60,0x80,0xA0,0xC0,0xE0,0xFF};
volatile unsigned char ledBuffer = 0x00;

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
    //Loop Forever
  }
}

ISR(TIMER0_OVF_vect)	//Timer0 overflow interrupt handler
{
  PORTD = ledBuffer;	//Set outputs prewound from last interrupt
  
  static unsigned char pwmCount = 255;	//Track overflows to compare with pwmValue
  if (++pwmCount == 0)
  {
    ledBuffer = 0x00;
    for (char i=0; i<8; i++)
    {
      if (pwmValues[i] != 0) ledBuffer |= (1<<i);
    }
  }
  else
  {
    for (char i=0; i<8; i++)
    {
      if (pwmCount > pwmValues[i]) ledBuffer &= ~(1<<i);
    }
  }
}
