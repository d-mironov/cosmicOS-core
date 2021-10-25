#ifndef _STM32_DEFS_H
#define _STM32_DEFS_H
#include "f4/rcc/rcc.h"

#define ST_CTRL_ENABLE  (1 << 0)
#define ST_CTRL_INTEN   (1 << 1)
#define ST_CTRL_CLKSRC  (1 << 2)
#define ST_CTRL_COUNT   (1 << 16)

void cosmicOS_init();

#endif
