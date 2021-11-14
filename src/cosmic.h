#ifndef _COSMIC_H
#define _COSMIC_H

#include <stdint.h> 

#define from_string(n)   (char[(n)]){}

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
 
typedef char*           str;

typedef uint8_t         byte;


typedef struct _vec3 {
    f32     x;
    f32     y;
    f32     z;
} vec3;

typedef struct _quat {
    f32     w;
    f32     x;
    f32     y;
    f32     z;
} quat;

typedef struct _vec4 {
    f32     x;
    f32     y;
    f32     z;
    f32     w;
} vec4;

#endif
