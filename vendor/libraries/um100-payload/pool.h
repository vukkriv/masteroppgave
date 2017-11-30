/**
 * @file pool.h
 * @brief manage a generic pool of element
 * @author cca@bespoon.com
 * @date 03/03/2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifndef _POOL_H_
#define _POOL_H_

#include <osal_type.h>

typedef struct pool {
	OSAL_u8* data;	// Start of the pool
	OSAL_u8* bitmap;	// Bitfield to tell which element is in use
	OSAL_u32 elmt_size; // Size of a single element
	OSAL_u32 nb_elmt;	// Number of element in the pool
} pool_t;

pool_t* allocate_pool(OSAL_u32 element_size, OSAL_u32 nb_element);
OSAL_error_t release_pool(pool_t* p);
OSAL_void* pool_get_buffer(pool_t* p);
OSAL_error_t pool_release_buffer(pool_t* p, OSAL_void* b);

#endif // defined _POOL_H_
