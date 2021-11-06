#ifndef _COSMIC_H
#define _COSMIC_H

#include <stdint.h> 

#define new_string(n)   (char[(n)]){}

//#define     u8      uint8_t

// ======| unsigned int |======
typedef uint8_t         u8;
typedef uint16_t        u16;
typedef uint32_t        u32;
typedef uint64_t        u64;

// ======| signed int |======
typedef int8_t          i8;
typedef int16_t         i16;
typedef int32_t         i32;
typedef int64_t         i64;

// ======| floats |======
typedef float           f32;
typedef double          f64;
 
typedef char*           string;

typedef uint8_t         byte;

#endif
