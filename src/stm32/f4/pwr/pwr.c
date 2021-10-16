#include "pwr.h"
#include "../gpio/gpio.h"
#include "../rcc/rcc.h"


void PWR_set_voltage_scaling(pwr_vscaling_t scaling) {
    PWR->CR |= scaling;
}
