/*
 * rtwtypes.h  –  standalone version
 */

#ifndef RTWTYPES_H
#define RTWTYPES_H

#include <stdint.h>
#include <stdbool.h>

/* Signed integer types */
typedef int8_t   int8_T;
typedef int16_t  int16_T;
typedef int32_t  int32_T;
typedef int64_t  int64_T;

/* Unsigned integer types */
typedef uint8_t  uint8_T;
typedef uint16_t uint16_T;
typedef uint32_t uint32_T;
typedef uint64_t uint64_T;

/* Floating point types */
typedef float   real32_T;
typedef double  real64_T;
typedef double  real_T;

/* Boolean */
typedef bool boolean_T;

/* Character */
typedef char   char_T;
typedef int    int_T;
typedef unsigned int uint_T;

/* Byte */
typedef unsigned char byte_T;

/* Generic pointer */
typedef void* pointer_T;

#endif /* RTWTYPES_H */
