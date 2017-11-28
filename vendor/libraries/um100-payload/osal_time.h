/**
 * @file osal_time.h
 * @brief OSAL definitions for timers and delay functions
 * @author pbo@bespoon.com
 * @date 8/01/2013
 */
/*
 * Copyright (C) Bespoon 2013. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */
 //#include <linux/time.h>
#include <sys/time.h>
#include <time.h>
#include <linux/types.h>


// struct timespec {
//	__kernel_time_t	tv_sec;			/* seconds */
//	long		tv_nsec;		/* nanoseconds */
// };

// be sure its declared.
int nanosleep(const struct timespec *, struct timespec *);

#ifndef _OSAL_TIME_H_
#define _OSAL_TIME_H_

 /**
  *  @brief OSAL_timeval time val definition
  */
typedef struct timeval OSAL_timeval;

 /**
  *  @brief OSAL_timezone time zone definition
  */
typedef struct timezone OSAL_timezone;


/** OSAL_udelay create a tempo of usec_delay micro-seconds, cpu not sleeping or accurate timer*/
#define	 OSAL_sleep( _sec,_usec) do{ \
		struct timespec _ts = {0};\
		_ts.tv_sec  = (_sec) + (_usec)/1000000;\
		_ts.tv_nsec = ((_usec)%1000000)*1000;\
		nanosleep(&_ts, NULL);\
	}while(0)

/** OSAL_udelay create a tempo of usec_delay micro-seconds, cpu not sleeping or accurate timer*/
#define	 OSAL_udelay( usec_delay ) OSAL_sleep(0,usec_delay);

/** OSAL_msleep create a sleep time of msec_delay milli-seconds, cpu may sleep, not maximum accuracy */
#define	 OSAL_msleep( msec_sleep ) OSAL_sleep(0,(msec_sleep)*1000)

#define OSAL_gettimeofday(tv, tz) gettimeofday(tv, tz)
#define OSAL_settimeofday(tv, tz) settimeofday(tv, tz)

#ifndef timersub
	#define OSAL_timersub(a,b,diff) do {\
		(diff)->tv_sec = (a)->tv_sec - (b)->tv_sec; \
		(diff)->tv_usec = (a)->tv_usec - (b)->tv_usec;\
		if ((diff)->tv_usec < 0) {\
			--(diff)->tv_sec;\
			(diff)->tv_usec += 1000000;\
		}\
  } while (0)
#else
	#define OSAL_timersub(a,b,diff) timersub(a,b,diff)
#endif

#ifndef timeradd
	#define OSAL_timeradd(a,b,sum ) do { \
		(sum)->tv_sec = (a)->tv_sec + (b)->tv_sec;\
		(sum)->tv_usec = (a)->tv_usec + (b)->tv_usec;\
		if ((sum)->tv_usec >= 1000000) {\
			++(sum)->tv_sec;\
			(sum)->tv_usec -= 1000000;\
		}\
	} while (0)
#else
	#define OSAL_timeradd(a,b,sum) timeradd(a,b,sum)
#endif

#ifndef timerisset
	#define OSAL_timerisset(a) ((a)->tv_sec || (a)->tv_usec)
#else
	#define OSAL_timerisset(a) timerisset(a)
#endif

#endif
