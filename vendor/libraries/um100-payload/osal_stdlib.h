/**
 * @file osal_trace.h
 * @brief Operating System Abstraction Layer for Standard Library routine - Implementation on linux userspace
 * @author cca@bespoon.com
 * @date 03/10/2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifndef _OSAL_STDLIB_H_
#define _OSAL_STDLIB_H_

#include "osal_type.h"
#include <string.h>
#include <stdlib.h>
#define OSAL_memset(s,c,n)  memset(s,c,n)
#define OSAL_malloc(size, id) malloc(size)
#define OSAL_realloc(p, size) realloc(p, size)
#define OSAL_free(p, id)	free(p)
#define OSAL_memcmp(s1, s2, n) memcmp(s1, s2, n)
#define OSAL_memcpy(dst, src, size) memcpy(dst, src, size)
#define OSAL_abs(val) ((val) < 0 ? -(val) : (val))

#endif // defined _OSAL_STDLIB_
