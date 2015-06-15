#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>


void initLED();

void led_animate();

void led_set(uint8_t hours, uint8_t minutes);
