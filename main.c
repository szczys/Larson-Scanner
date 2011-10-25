#define F_CPU 1000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

int main(void) {
  //Setup IO
  DDRD = 0xFF;
  PORTD = 0xFF;

  while(1){
    //Loop Forever
  }
}
