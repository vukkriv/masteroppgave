/**
 * @file ranging_data_depayload_common.c
 * @brief definition to function uses to manage data that have been received trough UWB, and must be depayloaded
 * @note THIS FILE IS IMPLEMENTATION INDEPENDANT, DONT EDIT THIS
 * @author ore@bespoon.com
 * @date 24/02/2015
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifdef CONFIG_RTLS_PROTOCOL

#include <osal_stdlib.h>

#include <ranging_data_depayload.h>


OSAL_s32 ranging_data_depayload_dup_hdr(rng_protocol_header_t * inHeader, OSAL_bool_t inHeaderAsLe, rng_protocol_header_t * outHeader, OSAL_bool_t outHeaderAsLe)
{
	//header is always saved as big endian
	if((inHeader==NULL)||(outHeader==NULL)) return -1;
	if( inHeaderAsLe != outHeaderAsLe)//switch endianess
	{
		outHeader->receiver_hash=SWITCH_ENDIANESS_32BITS(inHeader->receiver_hash);
		outHeader->ref_base_hash=SWITCH_ENDIANESS_32BITS(inHeader->ref_base_hash);
		outHeader->mega_frame=SWITCH_ENDIANESS_32BITS(inHeader->mega_frame);
		outHeader->hyper_frame=SWITCH_ENDIANESS_16BITS(inHeader->hyper_frame);
		outHeader->super_frame=inHeader->super_frame;
		outHeader->nb_entry=inHeader->nb_entry;
	}
	else
	{
		outHeader->receiver_hash=inHeader->receiver_hash;
		outHeader->ref_base_hash=inHeader->ref_base_hash;
		outHeader->mega_frame=inHeader->mega_frame;
		outHeader->hyper_frame=inHeader->hyper_frame;
		outHeader->super_frame=inHeader->super_frame;
		outHeader->nb_entry=inHeader->nb_entry;
	}
	return 0;
}

OSAL_s32 ranging_data_depayload_dup_entry(	rng_protocol_entry_t * inEntry,
											OSAL_bool_t inEntryAsLe,
											rng_protocol_entry_t * outEntry,
											OSAL_bool_t outEntryAsLe)
{
	OSAL_s32 ret = 0;
	rng_config_e orngDataTypeLE;
	OSAL_u16 specificSize=0;
	type_trame_t itttype=TT_UNKNOWN;
	type_rdv_t irdvtype=0;
	OSAL_u16 itotalEntrySize=0,ototalEntrySize=0;
	OSAL_u16 dataSize=0;
	loc_toa_t *iltoa=NULL,*oltoa=NULL;
	loc_metrics_t *ilmet=NULL,*olmet=NULL;
	loc_nrj_t *ilnrj=NULL,*olnrj=NULL;
	loc_phy_t *ilphy=NULL,*olphy=NULL;
	loc_estimate_nrj_t *ilesnrj=NULL,*olesnrj=NULL;
	loc_estimate_iq_t *ilesiq=NULL,*olesiq=NULL;
	loc_estimate_complete_t *ilesc=NULL,*olesc=NULL;
	beacon_protocol_entry_basic_t* ib=NULL;
	beacon_protocol_entry_basic_t* ob=NULL;
	beacon1_protocol_entry_basic_t* ib1=NULL;
	beacon1_protocol_entry_basic_t* ob1=NULL;
	beacon2_protocol_entry_basic_t* ib2=NULL;
	beacon2_protocol_entry_basic_t* ob2=NULL;
	rng_protocol_entry_basic_t* irng=NULL;
	rng_protocol_entry_basic_t* orng=NULL;
	rdv_protocol_entry_basic_t* irdv=NULL;
	rdv_protocol_entry_basic_t* ordv=NULL;

	if((inEntry==NULL)&&(outEntry==NULL)) return -1;
	//rngDataType is always saved as big endian
	if(inEntryAsLe) orngDataTypeLE = inEntry->rngDataType;
	else orngDataTypeLE = SWITCH_ENDIANESS_32BITS(inEntry->rngDataType);
	//common copy
	if(inEntryAsLe!=outEntryAsLe) 	outEntry->status = SWITCH_ENDIANESS_16BITS(inEntry->status);
	else outEntry->status = inEntry->status;
	outEntry->slot_index = inEntry->slot_index;
	outEntry->link_quality =  inEntry->link_quality;
	if(outEntryAsLe==OSAL_true) outEntry->rngDataType = orngDataTypeLE;
	else outEntry->rngDataType = SWITCH_ENDIANESS_32BITS(orngDataTypeLE);
	ret = ranging_data_depayload_getLocData(	inEntry,
						inEntryAsLe,
						&itttype,
						&irdvtype,
						&itotalEntrySize,
						&ib,
						&ib1,
						&ib2,
						&irng,
						&irdv,
						&iltoa,
						&ilmet,
						&ilnrj,
						&ilphy,
						&ilesnrj,
						&ilesiq,
						&ilesc);
	if(ret!=0) return ret;
	ret = ranging_data_depayload_getLocData(	outEntry,
						outEntryAsLe,
						NULL,
						NULL,
						&ototalEntrySize,
						&ob,
						&ob1,
						&ob2,
						&orng,
						&ordv,
						&oltoa,
						&olmet,
						&olnrj,
						&olphy,
						&olesnrj,
						&olesiq,
						&olesc);
	if(ret!=0) return ret;

	switch(itttype)
	{
		case TT_BEACON:
			specificSize=sizeof(beacon_protocol_entry_basic_t);
			break;
		case TT_BEACON1:
			specificSize=sizeof(beacon1_protocol_entry_basic_t);
			break;
		case TT_BEACON2:
			specificSize=sizeof(beacon2_protocol_entry_basic_t);
			break;
		case TT_TAG_RANGING:
			specificSize=sizeof(rng_protocol_entry_basic_t);
			break;
		case TT_RDV:
			specificSize=sizeof(rdv_protocol_entry_basic_t);
			break;
		default:return -1;// unused
	}
	//now get data back from input to ouput.If endianness differs, we must switch every u16 and u32 ( and only these)
	//for other u8, the copy will be sufficient
	OSAL_memcpy(&outEntry->data[0],&inEntry->data[0],specificSize);
	if(inEntryAsLe!=outEntryAsLe)
	{
		switch(itttype)
		{
			case TT_BEACON:
				ob->src_hash = SWITCH_ENDIANESS_32BITS(ib->src_hash);
				ob->hyperframe_number = SWITCH_ENDIANESS_16BITS(ib->hyperframe_number);
			break;
			case TT_BEACON1:
				ob1->src_hash = SWITCH_ENDIANESS_32BITS(ib1->src_hash);
				ob1->hyperframe_number = SWITCH_ENDIANESS_16BITS(ib1->hyperframe_number);
				break;
			case TT_BEACON2:
				ob2->src_hash = SWITCH_ENDIANESS_32BITS(ob2->src_hash);
				break;
			case TT_TAG_RANGING:
				orng->src_hash = SWITCH_ENDIANESS_32BITS(irng->src_hash);
				orng->dst_hash = SWITCH_ENDIANESS_32BITS(irng->dst_hash);
				break;
			case TT_RDV:
				switch(ordv->type)
				{
					case RDV_ACK:
						ordv->sub.ack.src_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.ack.src_hash);
						ordv->sub.ack.dst_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.ack.dst_hash);
						break;
					case RDV_PAIRING_REQUEST:
						ordv->sub.pr.dst_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.pr.dst_hash);
						break;
					case RDV_SIMPLE_PAIRING_REPLY:
						ordv->sub.spr.src_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.spr.src_hash);
						ordv->sub.spr.dst_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.spr.dst_hash);
						ordv->sub.spr.new_dst_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.spr.dst_hash);
						break;
					case RDV_DISSOCIATION_REQUEST:
						ordv->sub.dreq.src_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.dreq.src_hash);
						ordv->sub.dreq.dst_hash = SWITCH_ENDIANESS_32BITS(irdv->sub.dreq.dst_hash);
						break;
					//case RDV_DISSOCIATION_NOTIFICATION:
					//case RDV_PAIRING_NACK,
					//case RDV_SLOT_REQUEST,
					//case RDV_SLOT_GRANTED,
					//case RDV_PING,
					//case RDV_INFO_REQUEST,
					//case RDV_DEVICE_INFO_REPLY,
					//case RDV_FRIENDLY_NAME_REPLY,
					default://not yet supported/implemented
						break;
				}
				break;
			default:return -1;
		}
	}

	//copy everything.If endianess is the same, copy is done trough this
	//by the way, it allow to get uwbdata if exist
	//get datasize without basic entry specific to trame and {rng_config,status,slotnb, linkquality}
	dataSize = itotalEntrySize-specificSize-RNG_PROTOCOL_ENTRY_COMMON_SIZE;
	if(dataSize>0)
	{
		OSAL_memcpy(&outEntry->data[specificSize],&inEntry->data[specificSize],dataSize);
		if(inEntryAsLe!=outEntryAsLe) //ok, in this case, all loc structure value u64,u32,u16's endianness must be switched
		{
			if(orngDataTypeLE&RNG_CONF_LOC_TOA_MASK)
			{
				oltoa->toa_le = SWITCH_ENDIANESS_32BITS(iltoa->toa_le);
			}
			if(orngDataTypeLE&RNG_CONF_LOC_METR_MASK)
			{
				olmet->nrj_sum=SWITCH_ENDIANESS_64BITS(ilmet->nrj_sum);
				olmet->med=SWITCH_ENDIANESS_32BITS(ilmet->med);
				olmet->rms=SWITCH_ENDIANESS_32BITS(ilmet->rms);
				//olmet->nb_spl=ilmet->nb_spl;
			}
			if(orngDataTypeLE&RNG_CONF_LOC_NRJ_MASK)
			{
				olnrj->toa_fg1=SWITCH_ENDIANESS_32BITS(ilnrj->toa_fg1);
				olnrj->toa_fg2=SWITCH_ENDIANESS_32BITS(ilnrj->toa_fg2);
				olnrj->toa_fg3=SWITCH_ENDIANESS_32BITS(ilnrj->toa_fg3);
				olnrj->toa_fg4=SWITCH_ENDIANESS_32BITS(ilnrj->toa_fg4);
				olnrj->rxqid_fg1=SWITCH_ENDIANESS_32BITS(ilnrj->rxqid_fg1);
				olnrj->rxqid_fg2=SWITCH_ENDIANESS_32BITS(ilnrj->rxqid_fg2);
				olnrj->rxqid_fg3=SWITCH_ENDIANESS_32BITS(ilnrj->rxqid_fg3);
				olnrj->rxqid_fg4=SWITCH_ENDIANESS_32BITS(ilnrj->rxqid_fg4);
			}
			if(orngDataTypeLE&RNG_CONF_LOC_PHY_MASK)
			{
				olphy->sync_threshold=SWITCH_ENDIANESS_32BITS(ilphy->sync_threshold);
				olphy->rng_threshold=SWITCH_ENDIANESS_32BITS(ilphy->rng_threshold);
				//olphy->channel=ilphy->channel;
				//olphy->zone=ilphy->zone;
			}
			if(orngDataTypeLE&RNG_CONF_LOC_ES_NRJ_MASK)
			{
				OSAL_u8 i;
				//olesnrj->iwle=ilesnrj->iwle;
				//olesnrj->iwfg1=ilesnrj->iwfg1;
				//olesnrj->iwfg2=ilesnrj->iwfg2;
				//olesnrj->iwfg3=ilesnrj->iwfg3;
				//olesnrj->iwfg4=ilesnrj->iwfg4;

				for(i=0;i<EST_SIZE;i++)
				{
					//olesnrj->prp[i]=ilesnrj->prp[i];
					olesnrj->nrj[i]=SWITCH_ENDIANESS_32BITS(ilesnrj->nrj[i]);
				}
			}
			if(orngDataTypeLE&RNG_CONF_LOC_ES_IQ_MASK)
			{
				OSAL_u8 i;
				//olesiq->iwle=ilesiq->iwle;
				//olesiq->iwfg1=ilesiq->iwfg1;
				//olesiq->iwfg2=ilesiq->iwfg2;
				//olesiq->iwfg3=ilesiq->iwfg3;
				//olesiq->iwfg4=ilesiq->iwfg4;

				for(i=0;i<EST_SIZE;i++)
				{
					//olesiq->prp[i]=ilesiq->prp[i];
					olesiq->I[i]=SWITCH_ENDIANESS_16BITS(ilesiq->I[i]);
					olesiq->Q[i]=SWITCH_ENDIANESS_16BITS(ilesiq->Q[i]);
				}
			}
			if(orngDataTypeLE&RNG_CONF_LOC_ES_COMPLETE_MASK)
			{
				OSAL_u8 i;
				//olesc->iwle=ilesc->iwle;
				//olesc->iwfg1=ilesc->iwfg1;
				//olesc->iwfg2=ilesc->iwfg2;
				//olesc->iwfg3=ilesc->iwfg3;
				//olesc->iwfg4=ilesc->iwfg4;

				olesc->arctan_d_ii=SWITCH_ENDIANESS_16BITS(ilesc->arctan_d_ii);
				olesc->arctan_d_iq=SWITCH_ENDIANESS_16BITS(ilesc->arctan_d_iq);
				olesc->arctan_d_qi=SWITCH_ENDIANESS_16BITS(ilesc->arctan_d_qi);
				olesc->arctan_d_qq=SWITCH_ENDIANESS_16BITS(ilesc->arctan_d_qq);

				olesc->dll_rng_nrj_m4dll=SWITCH_ENDIANESS_32BITS(ilesc->dll_rng_nrj_m4dll);
				olesc->dll_rng_nrj_p4dll=SWITCH_ENDIANESS_32BITS(ilesc->dll_rng_nrj_p4dll);
				olesc->dll_rng_nrj_p0dll=SWITCH_ENDIANESS_32BITS(ilesc->dll_rng_nrj_p0dll);

				for(i=0;i<EST_SIZE;i++)
				{
					//oles->prp[i]=ilesc->prp[i];
					olesc->I[i]=SWITCH_ENDIANESS_16BITS(ilesc->I[i]);
					olesc->Q[i]=SWITCH_ENDIANESS_16BITS(ilesc->Q[i]);
					olesc->nrj[i]=SWITCH_ENDIANESS_32BITS(ilesc->nrj[i]);
				}
				olesc->ep_align=SWITCH_ENDIANESS_32BITS(ilesc->ep_align);
				olesc->ts_offset=SWITCH_ENDIANESS_32BITS(ilesc->ts_offset);
				olesc->ntoa_corr=SWITCH_ENDIANESS_32BITS(ilesc->ntoa_corr);
				olesc->hw_offset=SWITCH_ENDIANESS_32BITS(ilesc->hw_offset);
			}
		}
	}
	return 0;
}

OSAL_s32 ranging_data_depayload_getConfig(	rng_config_e *configLE,
											rng_config_e *configBE,
											OSAL_u16* uwbPayloadLenInBits,
											OSAL_u16* totalSizeInBytes,
											OSAL_bool_t sensor0_activated,
											OSAL_bool_t sensor1_activated,
											OSAL_bool_t sensor2_activated,
											OSAL_bool_t sensor3_activated,
											OSAL_bool_t sensor4_activated,
											OSAL_bool_t sensor5_activated,
											OSAL_bool_t stats_activated,
											OSAL_s32 stats_index,
											OSAL_bool_t edata_activated,
											OSAL_bool_t toa_activated,
											OSAL_bool_t measure_activated,
											OSAL_bool_t energy_activated,
											OSAL_bool_t phy_activated,
											OSAL_bool_t lesnrj_activated,
											OSAL_bool_t lesiq_activated,
											OSAL_bool_t lescomplete_activated,
											OSAL_bool_t slot_ignored)
{
	OSAL_s32 ret=0;
	rng_config_e rngDataTypeLE = 0;
	OSAL_u16 locSizeInBytes=0;
	OSAL_u16 uwbDataSizeInBits=0;

	if(configLE) *configLE=0;
	if(configBE) *configBE=0;
	if(uwbPayloadLenInBits) *uwbPayloadLenInBits=0;
	if(totalSizeInBytes) *totalSizeInBytes=0;

	if(sensor0_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_0_MASK;
	if(sensor1_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_1_MASK;
	if(sensor2_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_2_MASK;
	if(sensor3_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_3_MASK;
	if(sensor4_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_4_MASK;
	if(sensor5_activated==OSAL_true)  rngDataTypeLE|=RNG_CONF_SENSOR_5_MASK;
	if(stats_activated==OSAL_true)
	{
		//Stats
		if(stats_index<0)
		{
			rngDataTypeLE |= RNG_CONF_STATS_INDEX2CONF(STATS_ROTATE);
		}
		else
		{
			rngDataTypeLE |= RNG_CONF_STATS_INDEX2CONF(stats_index);
		}
	}
	if(edata_activated==OSAL_true)	rngDataTypeLE|=RNG_CONF_EXTDATA_MASK;

	//LOC PART
	if(toa_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_TOA_MASK;
		locSizeInBytes+=sizeof(loc_toa_t);
	}
	if(measure_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_METR_MASK;
		locSizeInBytes+=sizeof(loc_metrics_t);
	}
	if(energy_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_NRJ_MASK;
		locSizeInBytes+=sizeof(loc_nrj_t);
	}
	if(phy_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_PHY_MASK;
		locSizeInBytes+=sizeof(loc_phy_t);
	}

	#ifndef NO_IQ_SUPPORT // cea style
	if(lesnrj_activated==OSAL_true)
	{
		//manage bad input config
		if(lescomplete_activated==OSAL_true)
		{
			rngDataTypeLE|=RNG_CONF_LOC_ES_COMPLETE_MASK;
			locSizeInBytes+=sizeof(loc_estimate_complete_t);
		}
		else if(lesiq_activated==OSAL_true)
		{
			rngDataTypeLE|=RNG_CONF_LOC_ES_COMPLETE_MASK;
			locSizeInBytes+=sizeof(loc_estimate_complete_t);
		}
		else
		{
			rngDataTypeLE|=RNG_CONF_LOC_ES_NRJ_MASK;
			locSizeInBytes+=sizeof(loc_estimate_nrj_t);
		}
	}
	else if(lesiq_activated==OSAL_true)
	{
		if(lescomplete_activated==OSAL_true)
		{
			rngDataTypeLE|=RNG_CONF_LOC_ES_COMPLETE_MASK;
			locSizeInBytes+=sizeof(loc_estimate_complete_t);
		}
		else
		{
			rngDataTypeLE|=RNG_CONF_LOC_ES_IQ_MASK;
			locSizeInBytes+=sizeof(loc_estimate_iq_t);
		}
	}
	else if(lescomplete_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_ES_COMPLETE_MASK;
		locSizeInBytes+=sizeof(loc_estimate_complete_t);
	}
	#else //#ifndef NO_IQ_SUPPORT // cea style
	if(lescomplete_activated==OSAL_true) //lescomplete_activated cannot pass, change it
	{
		rngDataTypeLE|=RNG_CONF_LOC_ES_NRJ_MASK;
		locSizeInBytes+=sizeof(loc_estimate_nrj_t);
	}
	else if(lesnrj_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_ES_NRJ_MASK;
		locSizeInBytes+=sizeof(loc_estimate_nrj_t);
	}
	else if(lesiq_activated==OSAL_true)
	{
		rngDataTypeLE|=RNG_CONF_LOC_ES_IQ_MASK;
		locSizeInBytes+=sizeof(loc_estimate_iq_t);
	}
	#endif //#ifndef NO_IQ_SUPPORT // cea style
	if(slot_ignored)
	{
		rngDataTypeLE|=RNG_CONF_IGNORE_SLOT_MASK;
	}

	if(configLE) *configLE=rngDataTypeLE;
	if(configBE) *configBE=SWITCH_ENDIANESS_32BITS(rngDataTypeLE);

	ret = ranging_data_depayload_getConfigSizes(&rngDataTypeLE,NULL,&uwbDataSizeInBits,NULL);
	if(ret != 0) return ret;
	if(uwbPayloadLenInBits)
		*uwbPayloadLenInBits = uwbDataSizeInBits;
	if(totalSizeInBytes)
	{
		type_trame_t tttype=TT_UNKNOWN;
		if(uwbDataSizeInBits%32==0)
			*totalSizeInBytes=uwbDataSizeInBits/8;
		else
			*totalSizeInBytes=(uwbDataSizeInBits/32+1)*4;
		*totalSizeInBytes +=locSizeInBytes;
		//dont forget header in total size
		*totalSizeInBytes+=RNG_PROTOCOL_ENTRY_COMMON_SIZE;
		tttype=RNG_CONF_GET_TTTYPE(rngDataTypeLE);
		switch(tttype)
		{
			case TT_BEACON:
				*totalSizeInBytes +=sizeof(beacon_protocol_entry_basic_t);
				break;
			case TT_BEACON1:
				*totalSizeInBytes +=sizeof(beacon1_protocol_entry_basic_t);
				break;
			case TT_BEACON2:
				*totalSizeInBytes +=sizeof(beacon2_protocol_entry_basic_t);
				break;
			case TT_TAG_RANGING:
				*totalSizeInBytes +=sizeof(rng_protocol_entry_basic_t);
				break;
			case TT_RDV:
				*totalSizeInBytes +=sizeof(rdv_protocol_entry_basic_t);
				break;
			default:return -1;

		}
		*totalSizeInBytes +=sizeof(rng_config_e);
	}
	return 0;
}

OSAL_s32 ranging_data_depayload_getConfigSizes(	rng_config_e *configLE,rng_config_e *configBE,OSAL_u16* uwbPayloadLenInBits,OSAL_u16* totalSizeInBytes)
{
	rng_config_e rngDataTypeLE = 0;
	OSAL_u32 locSizeInBytes=0;
	OSAL_u32 uwbDataSizeInBits=0;
	OSAL_u32 totsizeInBytes=0;

	if(configBE)
	{
		rngDataTypeLE = SWITCH_ENDIANESS_32BITS((*configBE));
		if(configLE&&(rngDataTypeLE!=*configLE)) return -1;
	}
	else if(configLE) rngDataTypeLE=*configLE;


	//LOC PART
	if(rngDataTypeLE&RNG_CONF_LOC_TOA_MASK)
		locSizeInBytes+=sizeof(loc_toa_t);
	if(rngDataTypeLE&RNG_CONF_LOC_METR_MASK)
		locSizeInBytes+=sizeof(loc_metrics_t);
	if(rngDataTypeLE&RNG_CONF_LOC_NRJ_MASK)
		locSizeInBytes+=sizeof(loc_nrj_t);
	if(rngDataTypeLE&RNG_CONF_LOC_PHY_MASK)
		locSizeInBytes+=sizeof(loc_phy_t);
	if(rngDataTypeLE&RNG_CONF_LOC_ES_NRJ_MASK)
		locSizeInBytes+=sizeof(loc_estimate_nrj_t);
	if(rngDataTypeLE&RNG_CONF_LOC_ES_IQ_MASK)
		locSizeInBytes+=sizeof(loc_estimate_iq_t);
	if(rngDataTypeLE&RNG_CONF_LOC_ES_COMPLETE_MASK)
		locSizeInBytes+=sizeof(loc_estimate_complete_t);
	uwbDataSizeInBits=sensor0_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=sensor1_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=sensor2_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=sensor3_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=sensor4_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=sensor5_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=bspstat_depayload_getSizeInBitsLE(rngDataTypeLE);
	uwbDataSizeInBits+=extdata_depayload_getSizeInBitsLE(rngDataTypeLE);
	//alwasy 1 byte prepended before ext data
	if(rngDataTypeLE&RNG_CONF_EXTDATA_MASK) uwbDataSizeInBits+=8;
	if(uwbPayloadLenInBits)
		*uwbPayloadLenInBits = (OSAL_u16) uwbDataSizeInBits;
	if(totalSizeInBytes)
	{
		type_trame_t tttype=TT_UNKNOWN;
		if(uwbDataSizeInBits%32==0)//start of ranging info is aligned on 4bytes
			totsizeInBytes=uwbDataSizeInBits/8;
		else
			totsizeInBytes=(uwbDataSizeInBits/32+1)*4;
		totsizeInBytes +=locSizeInBytes;
		//dont forget basic header, depending on trame type + config u32  in total size
		tttype=RNG_CONF_GET_TTTYPE(rngDataTypeLE);
		switch(tttype)
		{
			case TT_BEACON:
				totsizeInBytes +=sizeof(beacon_protocol_entry_basic_t);
			break;
			case TT_BEACON1:
				totsizeInBytes +=sizeof(beacon1_protocol_entry_basic_t);
				break;
			case TT_BEACON2:
				totsizeInBytes +=sizeof(beacon2_protocol_entry_basic_t);
				break;
			case TT_TAG_RANGING:
				totsizeInBytes +=sizeof(rng_protocol_entry_basic_t);
				break;
			case TT_RDV:
				totsizeInBytes +=sizeof(rdv_protocol_entry_basic_t);
				break;
			default:return -1;
		}
		totsizeInBytes +=sizeof(rng_config_e);
		*totalSizeInBytes = (OSAL_u16) totsizeInBytes;
	}
	return 0;
}

OSAL_s32 ranging_data_depayload_getLocData(	rng_protocol_entry_t * entry,
											OSAL_bool_t entryAsLe,
											type_trame_t* typeTrame,
											type_rdv_t* rdvSubType,
											OSAL_u16* totalEntrySize,
											beacon_protocol_entry_basic_t** b,
											beacon1_protocol_entry_basic_t** b1,
											beacon2_protocol_entry_basic_t** b2,
											rng_protocol_entry_basic_t** rng,
											rdv_protocol_entry_basic_t** rdv,
											loc_toa_t** ltoa,
											loc_metrics_t** lmet,
											loc_nrj_t** lnrj,
											loc_phy_t** lphy,
											loc_estimate_nrj_t** lesnrj,
											loc_estimate_iq_t** lesiq,
											loc_estimate_complete_t** lescomplete)
{
	OSAL_u32 uwbDataSizeInBits=0;
	OSAL_u8* baseLocInfo=NULL;
	OSAL_u8* entryData=NULL;
	OSAL_u16 specificSize=0;
	OSAL_u16 entrySize=0;
	rng_config_e rngDataTypeLE;
	type_trame_t tttype=TT_UNKNOWN;

	if(totalEntrySize) *totalEntrySize=0;
	if(typeTrame) *typeTrame=TT_UNKNOWN;
	if(rdvSubType) *rdvSubType=0;
	if(b) *b=NULL;
	if(b1) *b1=NULL;
	if(b2) *b2=NULL;
	if(rng) *rng=NULL;
	if(rdv) *rdv=NULL;
	if(ltoa) *ltoa=NULL;
	if(lmet) *lmet=NULL;
	if(lnrj) *lnrj=NULL;
	if(lphy) *lphy=NULL;
	if(lesnrj) *lesnrj=NULL;
	if(lesiq)  *lesiq=NULL;
	if(lescomplete) *lescomplete=NULL;
	if((entry==NULL)||(entry->data==NULL)) return -1;
	entrySize=RNG_PROTOCOL_ENTRY_COMMON_SIZE;
	rngDataTypeLE = (entryAsLe == OSAL_true)?entry->rngDataType:SWITCH_ENDIANESS_32BITS(entry->rngDataType);
	tttype=RNG_CONF_GET_TTTYPE(rngDataTypeLE);
	if(typeTrame) *typeTrame=tttype;
	switch(tttype)
	{
		case TT_BEACON:
			specificSize=sizeof(beacon_protocol_entry_basic_t);
			if(b) *b=(beacon_protocol_entry_basic_t*) &entry->data[0];
			break;
		case TT_BEACON1:
			specificSize=sizeof(beacon1_protocol_entry_basic_t);
			if(b1) *b1=(beacon1_protocol_entry_basic_t*) &entry->data[0];
			break;
		case TT_BEACON2:
			specificSize=sizeof(beacon2_protocol_entry_basic_t);
			if(b2) *b2=(beacon2_protocol_entry_basic_t*) &entry->data[0];
			break;
		case TT_TAG_RANGING:
			specificSize=sizeof(rng_protocol_entry_basic_t);
			if(rng) *rng=(rng_protocol_entry_basic_t*) &entry->data[0];
			uwbDataSizeInBits=sensor0_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=sensor1_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=sensor2_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=sensor3_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=sensor4_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=sensor5_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=bspstat_depayload_getSizeInBitsLE(rngDataTypeLE);
			uwbDataSizeInBits+=extdata_depayload_getSizeInBitsLE(rngDataTypeLE);
			//alwasy 1 byte prepended before ext data
			if(rngDataTypeLE&RNG_CONF_EXTDATA_MASK) uwbDataSizeInBits+=8;
			if(uwbDataSizeInBits%32==0)
				specificSize+=uwbDataSizeInBits/8;
			else
				specificSize+=(uwbDataSizeInBits/32+1)*4;
			break;
		case TT_RDV:
			specificSize=sizeof(rdv_protocol_entry_basic_t);
			if(rdv) *rdv=(rdv_protocol_entry_basic_t*) &entry->data[0];
			if(rdvSubType)
			{
				rdv_protocol_entry_basic_t* r=(rdv_protocol_entry_basic_t*) &entry->data[0];
				//little or big endfian is equal, type is on u8
				*rdvSubType=r->type;
			}
			break;
		default: return -1;
	}
	entrySize+=specificSize;
	baseLocInfo=&entry->data[specificSize];
	entryData=baseLocInfo;
	//what we have to manage now is local loc info, managed by 16 next bit of rngDataType
	if(rngDataTypeLE&RNG_CONF_LOC_TOA_MASK)
	{
		if(ltoa) *ltoa=(loc_toa_t*) entryData;
		entryData+=sizeof(loc_toa_t);
		entrySize+=sizeof(loc_toa_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_METR_MASK)
	{
		if(lmet) *lmet=(loc_metrics_t*) entryData;
		entryData+=sizeof(loc_metrics_t);
		entrySize+=sizeof(loc_metrics_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_NRJ_MASK)
	{
		if(lnrj) *lnrj=(loc_nrj_t*) entryData;
		entryData+=sizeof(loc_nrj_t);
		entrySize+=sizeof(loc_nrj_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_PHY_MASK)
	{
		if(lphy) *lphy=(loc_phy_t*) entryData;
		entryData+=sizeof(loc_phy_t);
		entrySize+=sizeof(loc_phy_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_ES_NRJ_MASK)
	{
		if(lesnrj) *lesnrj=(loc_estimate_nrj_t*) entryData;
		entryData+=sizeof(loc_estimate_nrj_t);
		entrySize+=sizeof(loc_estimate_nrj_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_ES_IQ_MASK)
	{
		if(lesiq) *lesiq=(loc_estimate_iq_t*) entryData;
		entryData+=sizeof(loc_estimate_iq_t);
		entrySize+=sizeof(loc_estimate_iq_t);
	}
	if(rngDataTypeLE&RNG_CONF_LOC_ES_COMPLETE_MASK)
	{
		if(lescomplete) *lescomplete=(loc_estimate_complete_t*) entryData;
		entryData+=sizeof(loc_estimate_complete_t);
		entrySize+=sizeof(loc_estimate_complete_t);
	}
	if(totalEntrySize) *totalEntrySize=entrySize;
	return 0;
}


#define mask_size(a, size) ((a) & ((1<<(size)) - 1))
#define set_at_pos_size(a, offset, size) (mask_size(a,size) << (offset))
#define get_at_pos_size(a, offset, size) mask_size(((a) >> offset), (size))
static OSAL_void depayload_getUWBData(OSAL_u8* inBuffer,OSAL_u16 outSizeInBits, OSAL_u8* outBuffer,OSAL_u8* inStartByte, OSAL_u8* inStartBit)
{
	OSAL_u16 outSizeInBytes=outSizeInBits/8;
	OSAL_u16 leftBits=outSizeInBits-8*outSizeInBytes;
	OSAL_u8 dStartbyte=*inStartByte;
	OSAL_u8 dStartbit=*inStartBit;
	OSAL_u8 splittedSize=8-dStartbit;
	OSAL_u8 i;
	OSAL_u8 tmp;

	//Copy the data got from databuffer
	//each data byte at data[n], is splitted like:
	//               bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7
	//outbuffer[Xn]                                     i[0]
	//outbuffer[X+1] i[1] i[2] i[3] i[4] i[5] i[6] i[7]


	//               bit0 bit1 bit2 bit3 bit4 bit5 bit6 bit7
	//outbuffer[Xn]                 i[0] i[1] i[2] i[3] i[4]
	//outbuffer[X+1] i[5] i[6] i[7]

	// etc...

	//aka generalistically speaking
	//                            bit0 ...................bit[dStartbit]...........bit7
	//outbuffer[dStartbyte]                               i[0]                 i[7-dStartbit]
	//   count                                                     8-dStartbit
	//outbuffer[dStartbyte+1]     i[8-dStartbit]---i[7]
	//   count                    dStartbit
	if(dStartbit == 0)
	{
		if(outBuffer!=NULL)
		{
			for(i=0;i<outSizeInBytes;i++)
				outBuffer[i] = inBuffer[dStartbyte+i];
		}
	}
	else
	{
		if(outBuffer!=NULL)
		{
			for(i=0;i<outSizeInBytes;i++)
			{
				tmp = get_at_pos_size(inBuffer[dStartbyte+i], dStartbit, splittedSize);
				outBuffer[i] = tmp;
				tmp = get_at_pos_size(inBuffer[dStartbyte+i+1], 0, dStartbit);
				outBuffer[i] |= (tmp << splittedSize);
			}
		}
	}
	//manage rest:
	if(leftBits>0)
	{
		if((leftBits+dStartbit)<8)
		{
			tmp = get_at_pos_size(inBuffer[dStartbyte+outSizeInBytes], dStartbit, leftBits);
			if(outBuffer!=NULL)
				outBuffer[outSizeInBytes] = tmp;
			*inStartByte = dStartbyte+outSizeInBytes;
			*inStartBit  = dStartbit+leftBits;
		}
		else
		{
			leftBits=leftBits-splittedSize;
			if(outBuffer!=NULL)
			{
				tmp = get_at_pos_size(inBuffer[dStartbyte+outSizeInBytes], dStartbit, splittedSize);
				outBuffer[outSizeInBytes] = tmp;
				tmp = get_at_pos_size(inBuffer[dStartbyte+outSizeInBytes+1], 0, leftBits);
				outBuffer[outSizeInBytes] |= (tmp << splittedSize);
			}
			*inStartByte = dStartbyte+outSizeInBytes+1;
			*inStartBit  = leftBits;
		}
	}
	else
	{
		//this function must return the next start byte and start bit
		*inStartByte = dStartbyte+outSizeInBytes;
		*inStartBit  = dStartbit;
	}
}

static OSAL_u32 get_RoundedBytes(OSAL_u32 sizeInBits)
{
	OSAL_u32 roundedSizeInBytes=(sizeInBits/8);
	if((sizeInBits/8)*8 < sizeInBits)
		roundedSizeInBytes+=1;
	return roundedSizeInBytes;
}

static OSAL_s32 check_and_allocate_sensor(sensor_data_t** s,OSAL_u32 sizeinbits,OSAL_u32* isAllocated)
{
	if(isAllocated) *isAllocated = 0;
	if(s)
	{
		sensor_data_t* allocated = (sensor_data_t*) *s;
		if(!allocated)
		{
			allocated = (sensor_data_t*) OSAL_malloc(sizeof(sensor_data_t),0);
			if(!allocated) return -1;
			OSAL_memset(allocated,0,sizeof(sensor_data_t));
			allocated->data=(OSAL_u8*) OSAL_malloc(get_RoundedBytes(sizeinbits),0);
			if(!allocated->data)
			{
				OSAL_free(allocated,0);
				return -1; 
			}
			if(isAllocated) *isAllocated = 1;
			*s=allocated;
		}
		allocated->sizeInBits=sizeinbits;
		allocated->roundedSizeInBytes=get_RoundedBytes(sizeinbits);
		OSAL_memset(allocated->data,0,allocated->roundedSizeInBytes);
	}
	return 0;
}

static OSAL_s32 check_and_allocate_ed(ext_data_t** s,OSAL_u32 sizeinbits,OSAL_u32* isAllocated)
{
	if(isAllocated) *isAllocated = 0;
	if(s)
	{
		ext_data_t* allocated = (ext_data_t*) *s;
		if(!allocated)
		{
			allocated = (ext_data_t*) OSAL_malloc(sizeof(ext_data_t),0);
			if(!allocated) return -1;
			OSAL_memset(allocated,0,sizeof(ext_data_t));
			allocated->data=(OSAL_u8*) OSAL_malloc(get_RoundedBytes(sizeinbits),0);
			if(!allocated->data)
			{
				OSAL_free(allocated,0);
				return -1; 
			}
			if(isAllocated) *isAllocated = 1;
			*s=allocated;
		}
		allocated->ctx_index = 0;
		allocated->sizeInBits=sizeinbits;
		allocated->roundedSizeInBytes=get_RoundedBytes(sizeinbits);
		OSAL_memset(allocated->data,0,allocated->roundedSizeInBytes);
	}
	return 0;
}

static OSAL_s32 check_and_allocate_stats(stats_data_t** s,OSAL_u32* isAllocated)
{
	if(isAllocated) *isAllocated = 0;
	if(s)
	{
		stats_data_t* allocated = (stats_data_t*) *s;
		if(!allocated)
		{
			allocated = (stats_data_t*) OSAL_malloc(sizeof(stats_data_t),0);
			if(!allocated) return -1;
			if(isAllocated) *isAllocated = 1;
			*s = allocated;
		}
		OSAL_memset(allocated,0,sizeof(stats_data_t));
	}
	return 0;
}

OSAL_void free_stats_data_t(stats_data_t** stats)
{
	if(!stats) return;
	if(!*stats) return;
	OSAL_free((*stats),0),(*stats)=NULL;
}

OSAL_void free_sensor_data_t(sensor_data_t** sensor)
{
	if(!sensor) return;
	if(!*sensor) return;
	if((*sensor)->data) OSAL_free((*sensor)->data,0),(*sensor)->data=NULL;
	OSAL_free((*sensor),0),(*sensor)=NULL;
}

OSAL_void free_ext_data_t(ext_data_t** ed)
{
	if(!ed) return;
	if(!*ed) return;
	if((*ed)->data) OSAL_free((*ed)->data,0),(*ed)->data=NULL;
	OSAL_free((*ed),0),(*ed)=NULL;
}

OSAL_s32 dup_stats_data_t(stats_data_t* in,stats_data_t** out)
{
	if(in==NULL || out == NULL) return -1;
	if(check_and_allocate_stats(out,NULL)!=0) return -1;
	(*out)->val = in->val;
	(*out)->type = in->type;
	return 0;
}

OSAL_s32 dup_sensor_data_t(sensor_data_t* in,sensor_data_t** out)
{
	if(in==NULL || out == NULL) return -1;
	if(check_and_allocate_sensor(out,in->sizeInBits,NULL)!=0) return -1;
	OSAL_memcpy((*out)->data,in->data,in->roundedSizeInBytes);
	return 0;
}

OSAL_s32 dup_ext_data_t(ext_data_t* in,ext_data_t** out)
{
	if(in==NULL || out == NULL) return -1;
	if(check_and_allocate_ed(out,in->sizeInBits,NULL)!=0) return -1;
	(*out)->ctx_index  = in->ctx_index;
	OSAL_memcpy((*out)->data,in->data,in->roundedSizeInBytes);
	return 0;
}

OSAL_s32 ranging_data_depayload_getUWBData(	rng_protocol_entry_t * entry,
						OSAL_bool_t entryAsLe,
						sensor_data_t** s0, OSAL_bool_t* s0Activ,
						sensor_data_t** s1, OSAL_bool_t* s1Activ, 
						sensor_data_t** s2, OSAL_bool_t* s2Activ,
						sensor_data_t** s3, OSAL_bool_t* s3Activ,
						sensor_data_t** s4, OSAL_bool_t* s4Activ,
						sensor_data_t** s5, OSAL_bool_t *s5Activ,
						stats_data_t** stats, OSAL_bool_t* statsActiv,
						ext_data_t** ed, OSAL_bool_t* edActiv)
{
	OSAL_u8 curbit=0;
	OSAL_u8 curbyte=0;
	rng_config_e rngDataTypeLE;
	type_trame_t tttype=TT_UNKNOWN;
	OSAL_u32 sizeInBits=0;
	OSAL_u32 s0Allocated=0, s1Allocated=0, s2Allocated=0;
	OSAL_u32 s3Allocated=0, s4Allocated=0, s5Allocated=0;
	OSAL_u32 statsallocated=0;
	OSAL_u32 edsallocated=0;

	if(s0Activ) *s0Activ=OSAL_false;
	if(s1Activ) *s1Activ=OSAL_false;
	if(s2Activ) *s2Activ=OSAL_false;
	if(s3Activ) *s3Activ=OSAL_false;
	if(s4Activ) *s4Activ=OSAL_false;
	if(s5Activ) *s5Activ=OSAL_false;
	if(statsActiv) *statsActiv=OSAL_false;
	if(edActiv) *edActiv=OSAL_false;

	if((entry==NULL)||(entry->data==NULL)) return -1;

	if(entryAsLe==OSAL_false) rngDataTypeLE = SWITCH_ENDIANESS_32BITS(entry->rngDataType);
	else rngDataTypeLE = entry->rngDataType;
	//check where to start
	tttype=RNG_CONF_GET_TTTYPE(rngDataTypeLE);
	switch(tttype)
	{
		case TT_BEACON:
			curbyte=sizeof(beacon_protocol_entry_basic_t);
			break;
		case TT_BEACON1:
			curbyte=sizeof(beacon1_protocol_entry_basic_t);
			break;
		case TT_BEACON2:
			curbyte=sizeof(beacon2_protocol_entry_basic_t);
			break;
		case TT_TAG_RANGING:
			curbyte=sizeof(rng_protocol_entry_basic_t);
			break;
		case TT_RDV:
			curbyte=sizeof(rdv_protocol_entry_basic_t);
			break;
		default:return -1;
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_0_MASK)
	{
		if(s0Activ) *s0Activ=OSAL_true;
		sizeInBits = sensor0_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s0,sizeInBits,&s0Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s0)?(*s0)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_1_MASK)
	{
		if(s1Activ) *s1Activ=OSAL_true;
		sizeInBits = sensor1_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s1,sizeInBits,&s1Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s1)?(*s1)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_2_MASK)
	{
		if(s2Activ) *s2Activ=OSAL_true;
		sizeInBits = sensor2_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s2,sizeInBits,&s2Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s2)?(*s2)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_3_MASK)
	{
		if(s3Activ) *s3Activ=OSAL_true;
		sizeInBits = sensor3_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s3,sizeInBits,&s3Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s3)?(*s3)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_4_MASK)
	{
		if(s4Activ) *s4Activ=OSAL_true;
		sizeInBits = sensor4_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s4,sizeInBits,&s4Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s4)?(*s4)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_SENSOR_5_MASK)
	{
		if(s5Activ) *s5Activ=OSAL_true;
		sizeInBits = sensor5_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_sensor(s5,sizeInBits,&s5Allocated)!=0) goto error;
		depayload_getUWBData(entry->data,sizeInBits,(s5)?(*s5)->data:NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_STATS_MASK)
	{
		if(statsActiv) *statsActiv=OSAL_true;
		sizeInBits = bspstat_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_stats(stats,&statsallocated)!=0) goto error;
		if(stats)
		{
			OSAL_u8* data=(OSAL_u8*) &((*stats)->val);
			(*stats)->type = (protocol_stats_type_t) (RNG_CONF_STATS_INDEX(rngDataTypeLE)-1);
			depayload_getUWBData(entry->data,sizeInBits,data,&curbyte,&curbit);//becasue its in same order 
		}
		else 
			depayload_getUWBData(entry->data,sizeInBits,NULL,&curbyte,&curbit);
	}
	if(rngDataTypeLE&RNG_CONF_EXTDATA_MASK)
	{
		OSAL_u8 prepend[1];
		if(edActiv) *edActiv=OSAL_true;
		sizeInBits = extdata_depayload_getSizeInBitsLE(rngDataTypeLE);
		if(check_and_allocate_ed(ed,sizeInBits,&edsallocated)!=0) goto error;
		//there is always 1 bytes prepended:
		depayload_getUWBData(entry->data,8,(ed)?prepend:NULL,&curbyte,&curbit);
		if(ed)  (*ed)->ctx_index = prepend[0];
		depayload_getUWBData(entry->data,sizeInBits,(ed)?(*ed)->data:NULL,&curbyte,&curbit);
	}

	return 0;
error:
	if(s0Allocated) free_sensor_data_t(s0);
	if(s1Allocated) free_sensor_data_t(s1);
	if(s2Allocated) free_sensor_data_t(s2);
	if(s3Allocated) free_sensor_data_t(s3);
	if(s4Allocated) free_sensor_data_t(s4);
	if(s5Allocated) free_sensor_data_t(s5);
	if(statsallocated) free_stats_data_t(stats);
	if(edsallocated) free_ext_data_t(ed);
	return -1;
}


/* #################################################################################################### */
/* BESPOON INTERNAL SENSOR MANAGEMENT */
/* #################################################################################################### */

/* #################################################################################################### */
/* EXTERNAL SENSOR MANAGEMENT */
/* #################################################################################################### */
/* Bespon reference implementation : this slot is used to store b1,b2 pulse informations */
/* worst case : pass 664 bytes * 2 beacons through 15 slots = 1328bytes/16 bytes per slots =88.53 bytes => use 90 bytes  */
/* This should pass with 128TS, 256TS */
/* But this can be modified on implementation condition */

int MAX_EXTDATA_SIZE_IN_BITS = UWBDATA_EXTDATA_SIZE_IN_BITS_BASE;
OSAL_u32 extdata_depayload_getSizeInBitsLE(OSAL_u32 _rngDataTypeLE)
{
	return (_rngDataTypeLE&RNG_CONF_EXTDATA_MASK)?MAX_EXTDATA_SIZE_IN_BITS:0;
}


/* #################################################################################################### */
/* INIT FUNCTION */
/* #################################################################################################### */
OSAL_s32 ranging_data_depayload_init(OSAL_u32 ext_data_size_in_bits)
{
	if(ext_data_size_in_bits > (RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE * 8) ) return -1;
	if(ext_data_size_in_bits < 1) return -2;
	MAX_EXTDATA_SIZE_IN_BITS = ext_data_size_in_bits;
	return 0;
}


#endif /*#ifdef CONFIG_RTLS_PROTOCOL*/

