/**
 * @file osal_trace.h
 * @brief Operating System Abstraction Layer for logging - Implementation on linux userspace
 * @author cca@bespoon.com
 * @date 03/10/2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifndef _OSAL_TRACE_H_
#define _OSAL_TRACE_H_

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <osal_type.h>

#define TRACE_NONE  0
#define TRACE_ERROR 0x1
#define TRACE_WARN  0x2
#define TRACE_INFO  0x4
#define TRACE_DEBUG 0x8
#define TRACE_VERBOSE 0x10

#ifndef TRACE_DFLT_LEVEL
	#define TRACE_DFLT_LEVEL (TRACE_ERROR|TRACE_WARN|TRACE_INFO)
#endif

#ifndef TRACE_ENV_VAR
	#define TRACE_ENV_VAR "OSAL_TRACE"
#endif

extern OSAL_s32 TRLEVEL;

#define TRACE_INIT() do{ \
		if(getenv(TRACE_ENV_VAR)!=NULL) TRLEVEL=(OSAL_s32) strtol(getenv(TRACE_ENV_VAR),NULL,0);\
		else TRLEVEL=TRACE_DFLT_LEVEL;\
	}while(0)

#define ENABLE_TRACE_ERROR
#define ENABLE_TRACE_WARN
#define ENABLE_TRACE_DEBUG
#define ENABLE_TRACE_INFO
#define ENABLE_TRACE_VERBOSE

	#define PRINT_ENDOFLINE "\n"

	#define print(fmt, args...)  do{fprintf(stderr,""fmt"",##args);fflush(stderr);}while(0)

	#define printline(fmt, args...) do{fprintf(stderr,""fmt""PRINT_ENDOFLINE,##args);fflush(stderr);}while(0)

	#define osal_trace_is_log_possible(unused, type)  ((type&TRLEVEL) ? OSAL_true : OSAL_false)

	#define OSAL_trace(type,fmt, args...) \
		do{ \
			if(type&TRLEVEL) {\
				switch(type){\
					case TRACE_ERROR : OSAL_ERR(fmt, ##args);break;\
					case TRACE_WARN : OSAL_WARN(fmt, ##args);break;\
					case TRACE_INFO : OSAL_INFO(fmt, ##args);break;\
					case TRACE_DEBUG : OSAL_DBG(fmt, ##args);break;\
					case TRACE_VERBOSE : OSAL_VERB(fmt, ##args);break;\
					default : break;} \
			} \
		}while(0)

	#define OSAL_trace_noend(type,fmt, args...) \
		do{ \
			if(type&TRLEVEL) {\
				switch(type){\
					case TRACE_ERROR : OSAL_ERR_NOEND(fmt, ##args);break;\
					case TRACE_WARN : OSAL_WARN_NOEND(fmt, ##args);break;\
					case TRACE_INFO : OSAL_INFO_NOEND(fmt, ##args);break;\
					case TRACE_DEBUG : OSAL_DBG_NOEND(fmt, ##args);break;\
					case TRACE_VERBOSE : OSAL_VERB_NOEND(fmt, ##args);break;\
					default : break;} \
			} \
		}while(0)

	#ifndef TRACE_ENV_DOMAIN
		#define PRINT(lvl, fmt, args...) \
			printf("<" lvl "> " fmt "\n" ,##args)
	#else
		#define PRINT(lvl, fmt, args...) \
			printf("[" TRACE_ENV_DOMAIN "]<" lvl "> " fmt "\n" ,##args)
	#endif
	#define PRINT_NOEND(lvl, fmt, args...) \
			printf(fmt ,##args)


	#define OSAL_ENTER_FUNC OSAL_trace(TRACE_VERBOSE,"Enter %s",__FUNCTION__)
	#define OSAL_EXIT_FUNC OSAL_trace(TRACE_VERBOSE,"Exit %s",__FUNCTION__)

	#define OSAL_PERROR(fmt,args...)  OSAL_trace(TRACE_ERROR,fmt "(%d:%s)",##args,errno,strerror(errno))

	#ifdef ENABLE_TRACE_VERBOSE
	#define OSAL_VERB(fmt, args...) PRINT("V", fmt, ##args)
	#define OSAL_VERB_NOEND(fmt, args...) PRINT_NOEND("V", fmt, ##args)
	#else
	#define OSAL_VERB(fmt, args...)
	#define OSAL_VERB_NOEND(fmt, args...)

	#endif

	#ifdef ENABLE_TRACE_DEBUG
	#define OSAL_DBG(fmt, args...) PRINT("D", fmt, ##args)
	#define OSAL_DBG_NOEND(fmt, args...) PRINT_NOEND("D", fmt, ##args)
	#else
	#define OSAL_DBG(fmt, args...)
	#define OSAL_DBG_NOEND(fmt, args...)
	#endif

	#ifdef ENABLE_TRACE_INFO
	#define OSAL_INFO(fmt, args...) PRINT("I", fmt, ##args)
	#define OSAL_INFO_NOEND(fmt, args...) PRINT_NOEND("I", fmt, ##args)
	#else
	#define OSAL_INFO(fmt, args...)
	#define OSAL_INFO_NOEND(fmt, args...)
	#endif

	#ifdef ENABLE_TRACE_WARN
	#define OSAL_WARN(fmt, args...) PRINT("W", fmt, ##args)
	#define OSAL_WARN_NOEND(fmt, args...) PRINT_NOEND("W", fmt, ##args)
	#else
	#define OSAL_WARN(fmt, args...)
	#define OSAL_WARN_NOEND(fmt, args...)
	#endif

	#ifdef ENABLE_TRACE_ERROR
	#define OSAL_ERR(fmt, args...)  PRINT("E", fmt, ##args)
	#define OSAL_ERR_NOEND(fmt, args...)  PRINT_NOEND("E", fmt, ##args)
	#else
	#define OSAL_ERR(fmt, args...)
	#define OSAL_ERR_NOEND(fmt, args...)
	#endif

#endif // defined _OSAL_TRACE_
