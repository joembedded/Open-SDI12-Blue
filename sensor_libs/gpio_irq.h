/****************************************************
* gpio_irq.h - Test IRQ als ZAEHLEINGANG
*
* Schalter kann man detektieren, indem man misst, wielange er LOW->HIGH braucht
* Mit 10 nF kommen wir auf tau = 22msec, also 10 Hz. gehen alleweil 
***************************************************/

void test_the_irq(uint8_t val);

//***