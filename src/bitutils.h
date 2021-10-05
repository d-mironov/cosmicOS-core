#ifndef _COSMIC_BITUTILS_H
#define _COSMIC_BITUTILS_H

#define setbit_var(x,y)     (x |= (1 << (y)))   /*!< Set bit `y` of variable `x` */
#define setbit(x)       (1 << x)                /*!< Set bit `x` */

#endif
