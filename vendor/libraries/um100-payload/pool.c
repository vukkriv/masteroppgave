/**
 * @file pool.c
 * @brief manage a generic pool of element
 * @author cca@bespoon.com
 * @date 03/03/2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#include <osal_type.h>
#include <osal_trace.h>
#include <osal_stdlib.h>

#include "pool.h"


#ifndef CEIL_DIVIDE
#define CEIL_DIVIDE(a,b) ((a) + (b) - 1)/(b)
#endif

pool_t* allocate_pool(OSAL_u32 element_size, OSAL_u32 nb_element)
{
	pool_t* ret = NULL;


	if(element_size == 0 || nb_element == 0)
		return NULL;
	ret = OSAL_malloc(sizeof(pool_t), POOL_ID);

	// Out Of Memory
	if(ret == NULL)
		return NULL;

	// Allocating pool
	ret->data = OSAL_malloc(element_size * nb_element, POOL_ID);

	ret->bitmap = OSAL_malloc(CEIL_DIVIDE(nb_element,8), POOL_ID);

	ret->elmt_size = element_size;
	ret->nb_elmt = nb_element;

	// Out Of Memory
	if(ret->data == NULL || ret->bitmap == NULL)
	{
		OSAL_free(ret->data, POOL_ID);
		OSAL_free(ret->bitmap, POOL_ID);
		OSAL_free(ret, POOL_ID);
		return NULL;
	}

	return ret;
}

OSAL_error_t release_pool(pool_t* p)
{
	if(p)
	{
		OSAL_free(p->data, POOL_ID);
		OSAL_free(p->bitmap, POOL_ID);
		OSAL_free(p, POOL_ID);
	}

	return OSAL_OK;
}

OSAL_void* pool_get_buffer(pool_t* p)
{
	OSAL_u32 idx;	// index of an element in the pool
	OSAL_u32 q,r;	// euclidien division of idx by 8
	OSAL_void* ret = NULL;

	if(!p)
		return NULL;

	for(idx=0; idx < p->nb_elmt; idx++)
	{
		q = idx / 8;
		r = idx % 8;
		if((p->bitmap[q] & (1<<r)) == 0)
		{
			p->bitmap[q] |= (1<<r);
			ret = &p->data[idx * p->elmt_size];
			return ret;
		}
	}
	return NULL;
}

OSAL_error_t pool_release_buffer(pool_t* p, OSAL_void* b)
{
	OSAL_u32 idx;	// index of an element in the pool
	OSAL_u32 q,r;	// euclidien division of idx by 8
	OSAL_u8 *pool_start,*pool_stop;
	OSAL_u8* ptr;

	if(!b || !p)
		return OSAL_ERROR;

	ptr = (OSAL_u8*)b; 
	pool_start = (OSAL_u8*)p->data;
	pool_stop = (OSAL_u8*)p->data + p->nb_elmt*p->elmt_size;

	if(!(pool_start <= ptr && pool_start < pool_stop))
	{
		OSAL_trace(TRACE_ERROR, "released conf sequence out-of-band");
		return OSAL_ERROR;
	}

	if( (( ptr - pool_start) % p->elmt_size) != 0) // `command' must be aligned!
	{
		OSAL_trace(TRACE_ERROR, "released conf sequence unaligned");
		return OSAL_ERROR;
	}

	idx = (ptr - pool_start)/p->elmt_size;

	q = idx / 8;
	r = idx % 8;
	if((p->bitmap[q] & (1<<r)) == 0)
		OSAL_trace(TRACE_WARN, "buffer already released: %p (idx %d)", b, idx);
		
	p->bitmap[q] &= (OSAL_u8)(~(1 << r));

	return OSAL_OK;
}

