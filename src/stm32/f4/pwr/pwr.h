#ifndef _COSMIC_PWR_H
#define _COSMIC_PWR_H

#define PWR_VOLTAGE_SCALING_OFFSET  (0x0E)

typedef enum {
    PWR_SCALE_3 = (1 << PWR_VOLTAGE_SCALING_OFFSET),
    PWR_SCALE_2 = (2 << PWR_VOLTAGE_SCALING_OFFSET),
    PWR_SCALE_1 = (3 << PWR_VOLTAGE_SCALING_OFFSET),
} pwr_vscaling_t;

void PWR_set_voltage_scaling(pwr_vscaling_t scaling);

#endif
