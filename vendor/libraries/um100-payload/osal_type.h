/**
 * @file osal_type.h
 * @brief Definitions for <b>O</b>perating <b>S</b>ystem <b>A</b>bstraction <b>L</b>ayer  type
 * @author bgi@bespoon.com
 * @date 4/09/2012
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifndef _OSAL_TYPE_H_
#define _OSAL_TYPE_H_

#include <stdint.h>


/******* definitions of main PinPinter type  ********/
#ifdef __cplusplus
 extern "C"
#endif


typedef int8_t OSAL_s8;
typedef uint8_t OSAL_u8;
typedef int16_t OSAL_s16;
typedef uint16_t OSAL_u16;
typedef int32_t OSAL_s32;
typedef uint32_t OSAL_u32;
//#ifdef CONFIG_HAVE_64BITS
typedef int64_t OSAL_s64;
typedef uint64_t OSAL_u64;
//#endif
//#ifdef CONFIG_HAVE_OSAL_float
typedef float OSAL_float; //WARNING
typedef double OSAL_double;
//#endif
typedef uint32_t OSAL_stackLevel;
#ifdef __cplusplus
  #define OSAL_void void
#else
  typedef void OSAL_void;
#endif

#define max_s8  127 
#define max_u8  255 
#define max_s16  32767 
#define min_s16  -32768
#define max_u16  65535 
#define max_s32  2147483647L
#define min_s32  -2147483648L
#define max_u32  4294967295UL
#define max_s64  9223372036854775807LL
#define max_u64  18446744073709551615ULL

#define OSAL_READ_ONLY const
#define OSAL_PROD_SECT const

#define OSAL_PACKED_STRUCTURE  __attribute__((__packed__))
#define OSAL_PACKED_ENUM __attribute__((__packed__))

typedef enum {
	OSAL_true= 1,
	OSAL_false= 0
}OSAL_PACKED_ENUM OSAL_bool_t;

/*TODO add more relevant error code*/
typedef enum {
	OSAL_OK=0,
	OSAL_ERROR,
	OSAL_ENOMEM,
	OSAL_EAGAIN
}OSAL_PACKED_ENUM OSAL_error_t;

#define to_s8(x)  ((OSAL_s8)(x))
#define to_u8(x)  ((OSAL_u8)(x))
#define to_s16(x) ((OSAL_s16)(x))
#define to_u16(x) ((OSAL_u16)(x))
#define to_s32(x) ((OSAL_s32)(x))
#define to_u32(x) ((OSAL_u32)(x))
#ifdef CONFIG_HAVE_64BITS
#define to_s64(x) ((OSAL_s64)(x))
#define to_u64(x) ((OSAL_u64)(x))
#endif

#define OSAL_ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))

#ifndef NULL
#define NULL 0
#endif

#endif
