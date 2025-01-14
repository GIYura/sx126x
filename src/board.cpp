#include <Arduino.h>
#include "board.h"

void BoardDisableIrq(void)
{
#if 0
    noInterrupts();
#endif
}

void BoardEnableIrq(void)
{
#if 0
    interrupts();
#endif
}