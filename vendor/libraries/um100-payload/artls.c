/**
 * @file artls.c
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
#include <osal_trace.h>
#include <osal_stdlib.h>

#include <pinpointer_drv_api.h>
#include "artls.h"

#define ARTLS_ID 538 // TODO: put that in a centralized place

#ifdef CONFIG_RTLS_FULL_LOC

#define DEVICE_TIMEOUT 20 // in ARTLS_TICK_DURATION (e.g. in second)

device_t* devices = NULL;
OSAL_u32 nb_devices = 0;


static const OSAL_u32 hash_coeff[] = {
	4278,	1547,
	2656,	4005,
	1274,	2383,
	3732,	1001,
	2110,	3459,
	728,	 1837,
	3186,	455,
	1564,	2913,
};

// Hash algorithm is based on a 4-bits Fletcher checksum of the uwb mac address repeated 3 times
OSAL_void uma_hash_from_address(OSAL_u8 mac[UWB_MAC_LEN], OSAL_u32* hash)
{
	OSAL_u32 r=0;
	OSAL_u32 a,b;
	OSAL_u8 i;

	if(!hash)
		return;

	for(i=0; i<8;i++)
	{
		a = mac[i] >> 4;	// Four most significants bits of mac[i]
		b = mac[i] & 0xF;	// Four less significants bits of mac[i]

		// MSB
		r +=  hash_coeff[2*i]*a;

		// LSB
		r += hash_coeff[2*i+1]*b;
	}

	// Running ones. Note that above sum never overflow on a 32 bits operation
	while(r >> 12)
		r = (r & 0xFFF) + (r >> 12);

	// Bad hack: prevent the hash to be all '0' or all '1'
	// except for 0000:0000:0000:0000 and FFFF:FFFF:FFFF:FFFF
	// Note: for what I saw, hash==0 only for 0000:0000:0000:0000
	// But it's not the case for 0xFFF (0000:0000:01AE:0320 is an example)
	if(r == 0)
		for(i=0; i<8; i++)
			r |= mac[i];	// r will be '0' only if all mac[i] are 0
	else if(r == 0xFFF)
		for(i=0; i<8;i++)
			r &= (0xF00 | mac[i]);	// r will be 0xFFF only if all mac[i] are 0xFF

	*hash = r;
}


static void upgrade_device(device_t* dev, OSAL_u8* devMac,OSAL_u32 devHash,device_attachment_status_t attach, OSAL_u32 last_activity)
{
	// Assign a hash
	if(dev->addr == DEFAULT_ADDR)
	{
		OSAL_memcpy(dev->mac, devMac, UWB_MAC_LEN);
		uma_hash_from_address(dev->mac,&dev->addr);
		//current_tag->addr = request->tag_hash;
		dev->attach = DAS_ATTACHING;
		dev->last_activity = 0;
	}
	else
	{
		dev->attach = attach;
		dev->last_activity = last_activity;
	}
	if(dev->friend) upgrade_device(dev->friend,devMac,devHash,attach,last_activity);
}

static OSAL_void reset_device(device_t* dev,OSAL_bool_t allOfThem)
{
	if(allOfThem == OSAL_true)
	{
		device_t* devToFree=NULL;
		OSAL_u32 i;
		for(i=0; i < nb_devices; i++)
		{
			devToFree = &devices[i];
			reset_device(devToFree,OSAL_false);
		}
	}
	else
	{
		if(!dev)
			return;

		if(dev->addr == DEFAULT_ADDR)
			return;

		OSAL_memset(dev->mac, 0, UWB_MAC_LEN);
		dev->addr = DEFAULT_ADDR;
		dev->attach = DAS_DETACHED;
		dev->last_activity = 0;
		dev->master = NULL;

		if(dev->friend) reset_device(dev->friend,OSAL_false);
		dev->friend=NULL;
		// Do *NOT* reset slot description
	}
}

static device_t* find_free_slot(OSAL_u8* mac, OSAL_u32 startIndex)
{
	OSAL_u32 new_ranging_slot = max_u32;
	device_t* tag = NULL, *current_tag = NULL, *lastfriend_tag=NULL;
	OSAL_u32 i = 0, j = 0;

	if(nb_devices == 0 || devices == NULL)
		return NULL;

	for(i=startIndex; i<nb_devices; i++)
	{
		tag = &devices[i];
		if(OSAL_memcmp(tag->mac, mac, UWB_MAC_LEN) == 0)
		{
			//its the same
			new_ranging_slot = tag->position_loc;
			current_tag = tag;
			break;
		}

		if(tag->addr == DEFAULT_ADDR && tag->position_loc < new_ranging_slot)
		{
			//its a new one
			current_tag = tag;
			new_ranging_slot =  tag->position_loc;
			break;
		}

	}

	if(!current_tag)
		OSAL_trace(TRACE_INFO, "no more slot");
	else
	{
		//now try to get nearest config of option_slotMethod_nb, option_slotMethod_increment;
		new_ranging_slot = max_u32;
		current_tag->sent_slotMethod_increment = current_tag->option_slotMethod_increment;
		current_tag->friend = NULL;
		i+=current_tag->sent_slotMethod_increment;
		lastfriend_tag=current_tag;
		lastfriend_tag->master = current_tag;
		for(j=i; j<nb_devices; j+=current_tag->sent_slotMethod_increment)
		{
			if(current_tag->sent_slotMethod_nb>=current_tag->option_slotMethod_nb) break; 
			tag = &devices[j];
			if(tag->addr == DEFAULT_ADDR && tag->position_loc < new_ranging_slot && tag->position_loc > lastfriend_tag->position_loc )
			{
				lastfriend_tag->friend = tag;
				lastfriend_tag->master = current_tag;
				lastfriend_tag = tag;
				current_tag->sent_slotMethod_nb++;
			}
			else
			{   //already got ?, stop here this loop, mean sent_slotMethod_nb != option_slotMethod_nb, but tag config still aligned
				OSAL_trace(TRACE_INFO, "already used!!");
				break;
			}
		}
	}
	return current_tag;
}


OSAL_void init_devices(OSAL_u32* nb_dev_used, superframe_info_t* sfi, OSAL_u32 loc_rate, OSAL_u8 option_slotMethod_nb, OSAL_u8 option_slotMethod_increment)
{
	OSAL_u32 i,j;
	OSAL_u8 nb_rng_slots = 0;

	for(i=0; i<sfi->nb_frames;i++)
	{
		if(sfi->frames_desc[i].act == RX && sfi->frames_desc[i].type == FIT_RANGING)
			nb_rng_slots++;
	}

	// FIXME: honor nb_dev parameter
	*nb_dev_used = nb_rng_slots;


	// No dev? nothing to do here
	if(*nb_dev_used == 0)
		return;

	nb_devices = *nb_dev_used;
	devices = OSAL_malloc(sizeof(device_t) * nb_devices, ARTLS_ID);

	j=sfi->nb_frames;
	for(i=0; i < nb_devices; i++)
	{
		do
		{
			j++;
			j%= sfi->nb_frames;
		}
		while( !(sfi->frames_desc[j].act == RX  && sfi->frames_desc[j].type == FIT_RANGING));


		OSAL_memset(devices[i].mac,0, UWB_MAC_LEN);
		devices[i].addr = DEFAULT_ADDR;
		devices[i].last_activity = 0;
		devices[i].attach = DAS_DETACHED;
		devices[i].position_loc = sfi->frames_desc[j].slot_index;
		devices[i].loc_rate_ratio = loc_rate;
		devices[i].rngDataType = sfi->frames_desc[j].rngDataType;
		devices[i].option_slotMethod_nb = option_slotMethod_nb;
		devices[i].option_slotMethod_increment = option_slotMethod_increment;
		devices[i].sent_slotMethod_nb = 1;
		devices[i].sent_slotMethod_increment = 0;
		devices[i].friend = NULL;
		devices[i].master = NULL;
	}
}

OSAL_void reset_devices(OSAL_u8 option_slotMethod_nb, OSAL_u8 option_slotMethod_increment)
{
	int i;
	reset_device(NULL,OSAL_true);
	for(i=0; i < nb_devices; i++)
	{
		devices[i].option_slotMethod_nb = option_slotMethod_nb;
		devices[i].option_slotMethod_increment = option_slotMethod_increment;
		devices[i].sent_slotMethod_nb = 1;
		devices[i].sent_slotMethod_increment = 0;
	}
}

artls_pairing_request_t** get_activdevices_list()
{
	//count nb of ready devices, excluding 'freiends ' one
	artls_pairing_request_t** activTable=NULL;
	device_t* devToGet=NULL;
	OSAL_u32 i;
	OSAL_u32 activcnt=0;
	for(i=0; i < nb_devices; i++)
	{
		devToGet = &devices[i];
		if(
			(devToGet->addr != DEFAULT_ADDR)&&
			(devToGet->master == devToGet)
		)
		{
			activcnt++;
		}
	}
	if(activcnt == 0) return NULL;
	activTable=(artls_pairing_request_t**) OSAL_malloc((activcnt+1)*sizeof(artls_pairing_request_t*),SMAC_ID);
	OSAL_memset(activTable,0,(activcnt+1)*sizeof(artls_pairing_request_t*));
	activcnt=0;
	for(i=0; i < nb_devices; i++)
	{
		devToGet = &devices[i];
		if(
			(devToGet->addr != DEFAULT_ADDR)&&
			(devToGet->master == devToGet)
		)
		{
			activTable[activcnt] =(artls_pairing_request_t*) OSAL_malloc(sizeof(artls_pairing_request_t),SMAC_ID);
			activTable[activcnt]->tag_hash=devToGet->addr;
			OSAL_memcpy(activTable[activcnt]->tag_mac,devToGet->mac, UWB_MAC_LEN);
			activcnt++;
		}
	}
	return activTable;
}


OSAL_error_t simple_pairing_request(artls_pairing_request_t* request, artls_simple_pairing_reply_t* reply)
{
	device_t* current_tag = NULL;
	device_t* friend_tag = NULL;

	if(!request || !reply)
		return OSAL_ERROR;

	OSAL_memset(reply, 0, sizeof(artls_simple_pairing_reply_t));

	reply->old_dst_hash = request->tag_hash;

	current_tag = find_free_slot(request->tag_mac,0);

	if(current_tag == NULL)
	{
		OSAL_trace(TRACE_WARN, "PAN at capacity");
		reply->pairing_answer = 1; // PAN at capacity
		return OSAL_OK;
	}
	upgrade_device(current_tag,request->tag_mac,request->tag_hash,DAS_ATTACHING,0);
	OSAL_trace_noend(TRACE_INFO, " TAG %x%x:%x%x:%x%x:%x%x will range at slot(s) ",
					current_tag->mac[0],current_tag->mac[1],current_tag->mac[2],current_tag->mac[3],
					current_tag->mac[4],current_tag->mac[5],current_tag->mac[6],current_tag->mac[7]);
	friend_tag=current_tag;
	do{
		OSAL_trace_noend(TRACE_INFO, "%d,",friend_tag->position_loc);
		friend_tag=friend_tag->friend;
	}while(friend_tag);
	OSAL_trace_noend(TRACE_INFO, "\n");

	reply->new_dst_hash	= current_tag->addr;
	reply->gts_type		= 0; // ranging, the only one supported for now
	reply->slot_first	= current_tag->position_loc;
	reply->slot_inc		= current_tag->sent_slotMethod_increment;
	reply->slot_total	= current_tag->sent_slotMethod_nb;
	reply->zone 		= current_tag->zone;
	reply->channel 		= current_tag->channel;
	reply->rngDataType 	= current_tag->rngDataType;
	reply->gts_rate		= current_tag->loc_rate_ratio;

	return OSAL_OK;
}

OSAL_error_t simple_pairing_request_finished(artls_simple_pairing_reply_t* reply, artls_cmd_return_code_t cmd_status)
{
	device_t* current_tag = NULL;
	OSAL_u32 i;

	if(!reply)
		return OSAL_ERROR;

	for(i=0; i<nb_devices; i++)
	{
		if(devices[i].position_loc == reply->slot_first && devices[i].addr == reply->new_dst_hash)
		{
			current_tag = &devices[i];
			break;
		}
	}

	if(!current_tag)
	{
		OSAL_trace(TRACE_WARN, "Can't find tag %.4X at slot(s) <%d[(inc:%d),total:%d]>",
			reply->new_dst_hash,
			reply->slot_first,
			reply->slot_inc,
			reply->slot_total);
		return OSAL_ERROR;
	}

	if(cmd_status == ACC_SUCCESS)
	{
		upgrade_device(current_tag,current_tag->mac,current_tag->addr,DAS_ATTACHED,0);
		OSAL_trace(TRACE_INFO, "device %.4X attached", current_tag->addr);
	}
	else
	{
		OSAL_trace(TRACE_INFO, "device %.4X disconnected (no answer to SPR)", current_tag->addr);
		reset_device(current_tag,OSAL_false);
	}

	return OSAL_OK;
}

OSAL_error_t notify_device_activity(OSAL_u32 addr)
{
	OSAL_u32 i;
	device_t* tag = NULL;
	if(nb_devices == 0 || devices == NULL)
		return OSAL_OK;

	for(i=0; i<nb_devices; i++)
	{
		tag = &devices[i];
		if(tag->addr != DEFAULT_ADDR && tag->addr == addr)
		{
			upgrade_device(tag,tag->mac,tag->addr,tag->attach,0);
			break;
		}
	}
	return OSAL_OK;
}


OSAL_error_t device_timeout_tick(OSAL_u64 nb_tick)
{
	OSAL_u32 i, dev_loc_rate;
	device_t* tag = NULL;
	if(nb_devices == 0 || devices == NULL)
		return OSAL_OK;

	for(i=0; i<nb_devices; i++)
	{
		tag = &devices[i];

		if(tag->addr == DEFAULT_ADDR)
			continue;

		tag->last_activity+= nb_tick;

		/* in dynamic locrate (case 0) take the maximum as a timeout limit*/
		/* TODO link timeout to SF duration instead of ARTLS_TICK*/
		if(tag->loc_rate_ratio == 0)
			dev_loc_rate = ARTLS_MAX_DYNAMIC_LOCRATE;
		else
			dev_loc_rate = tag->loc_rate_ratio;

		if(tag->last_activity > (dev_loc_rate * DEVICE_TIMEOUT))
		{
			printf("device %.4X disconnected (timeout)\n", tag->addr);
			reset_device(tag,OSAL_false);
		}
	}
	return OSAL_OK;

}


#endif // defined CONFIG_RTLS_FULL_LOC
