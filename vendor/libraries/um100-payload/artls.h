/**
 * @file artls.h
 * @brief Assisted Real Time Localization System
 * @author cca@bespoon.com
 * @date 30/09/2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */


#include <osal_type.h>
#include <pinpointer_drv_api.h>

#ifndef _ARTLS_H_
#define _ARTLS_H_

#define SMAC_ID 100
#define POOL_ID 325

#ifndef DEFAULT_ADDR
#define DEFAULT_ADDR 0x0FFF
#endif

#define ARTLS_TICK_DURATION 1 // in seconds
#define ARTLS_MAX_DYNAMIC_LOCRATE 40//in SF 

#ifdef CONFIG_RTLS_FULL_LOC
typedef enum {
	DAS_DETACHED = 0,
	DAS_ATTACHING =1,
	DAS_ATTACHED = 2,
} device_attachment_status_t;

typedef struct device_tag device_t;

struct device_tag
{
	OSAL_u8		mac[UWB_MAC_LEN];
	OSAL_u32	addr;
	OSAL_u32	last_activity;
	device_attachment_status_t attach;

	// Slot description
	OSAL_u8		position_loc;
	OSAL_u8		loc_rate_ratio;
	OSAL_u8		channel;
	OSAL_u8		zone;
	OSAL_u8 	option_slotMethod_nb;
	OSAL_u8 	option_slotMethod_increment;
	OSAL_u8 	sent_slotMethod_nb;
	OSAL_u8 	sent_slotMethod_increment;
	rng_config_e	rngDataType;
	device_t*   friend; /**< a same device on multiple slots */
	device_t*   master; /**< for same device on multiple slots, the first one */
};

OSAL_void init_devices(OSAL_u32* nb_dev_used, superframe_info_t* sfi, OSAL_u32 loc_rate, OSAL_u8 option_slotMethod_nb, OSAL_u8 option_slotMethod_increment);
OSAL_void reset_devices(OSAL_u8 option_slotMethod_nb, OSAL_u8 option_slotMethod_increment);
//return a null terminated table of activ list
artls_pairing_request_t** get_activdevices_list();
OSAL_error_t simple_pairing_request(artls_pairing_request_t* request, artls_simple_pairing_reply_t* reply);
OSAL_error_t simple_pairing_request_finished(artls_simple_pairing_reply_t* reply, artls_cmd_return_code_t cmd_status);
OSAL_error_t notify_device_activity(OSAL_u32 addr);
OSAL_error_t device_timeout_tick(OSAL_u64 nb_tick);
#endif

#endif // defined _ARTLS_H_
