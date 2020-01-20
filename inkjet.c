#include "pinio.h"

void hpc6602_init()
{
  
  #if defined(INK_PINA) && INK_PINA > -1
    SET_OUTPUT(INK_PINA);
  #endif
  #if defined(INK_PINB) && INK_PINB > -1
    SET_OUTPUT(INK_PINB);
  #endif
  #if defined(INK_PINC) && INK_PINC > -1
    SET_OUTPUT(INK_PINC);
  #endif
  #if defined(INK_PIND) && INK_PIND > -1
    SET_OUTPUT(INK_PIND);
  #endif
  #if defined(INK_PULSE) && INK_PULSE > -1
    SET_OUTPUT(INK_PULSE);
  #endif

}