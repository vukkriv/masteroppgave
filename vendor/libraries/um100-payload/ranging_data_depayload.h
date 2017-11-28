/**
 * @file ranging_data_depayload.h
 * @brief definition to function uses to manage packet that have been received trough UWB, and must be depayloaded
 * @author ore@bespoon.com
 * @date 24/02/2015
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#ifndef _RANGING_DATA_DEPAYLOAD_H_
#define _RANGING_DATA_DEPAYLOAD_H_

#include <osal_type.h>
#include <pinpointer_common_def.h>


/** @page ranging_specific Managing specificity
 * 
 * 
 * 
 * 
 * TODO
 * 
 * 
 * 
 * 
*/ 
/************************************** COMMON ROUTINE *************/

#ifdef __cplusplus
extern "C" {
#endif

/**
 @brief stats_data_t
 @note structure for statistic coming from a device
*/
typedef struct{
    OSAL_u32 val;               /**< value for this statistic */
    protocol_stats_type_t type; /**< type of statistic */
}stats_data_t;

/**
 @brief sensor_data_t
 @note structure for a sensor informations coming from a device
*/
typedef struct{
    OSAL_u32 sizeInBits;        /**< size in bits of sensor data */
    OSAL_u32 roundedSizeInBytes;/**< neareset upper size in bytes of sensor data, which is the size of data buffer below*/
    OSAL_u8* data;        /**< buffer containing sensor data (with sizeInBits usefull bits), of size roundedSizeInBytes*/
}sensor_data_t;

/**
 @brief ext_data_t
 @note structure for external data coming from a device
*/
typedef struct{
    OSAL_u32 sizeInBits;        /**< size in bits of external data */
    OSAL_u32 roundedSizeInBytes;/**< neareset upper size in bytes of external data, which is the size of data buffer below*/
    OSAL_u8 ctx_index;          /**< context index */
    OSAL_u8 rsvd[3];            /**< 32 bits alignement */
    OSAL_u8* data;              /**< buffer containing external data (with sizeInBits usefull bits), of size roundedSizeInBytes*/
}ext_data_t;

/**
 * @brief ranging_data_depayload_init
 * @note (necessary ) call to initialise ranging data depayload function
 * @param ext_data_size_in_bits
 * @return <0 on error, 0 on success
 */
OSAL_s32 ranging_data_depayload_init(OSAL_u32 ext_data_size_in_bits);

/**
 * @brief ranging_data_depayload_dup_hdr
 * @note duplicate the header from a ranging data frame, with change of endianness eventually
 * @param inHeader input ranging data frame header.necessarily as big endian
 * @param inHeaderAsLe if true, all 32/16 bits fields of in header are stored as little endian. Remember that a header coming from lower layer is in big endian
 * @param outHeader output ranging data frame header
 * @param outHeaderAsLe if true, all 32/16 bits fields of out header will be stored as little endian. Remember that a header coming from lower layer is in big endian
 * @return <0 on error, 0 on success
 */
OSAL_s32 ranging_data_depayload_dup_hdr(rng_protocol_header_t * inHeader, OSAL_bool_t inHeaderAsLe, rng_protocol_header_t * outHeader, OSAL_bool_t outHeaderAsLe);

/**
 * @brief ranging_data_depayload_dup_entry
 * @note duplicate an entry from a ranging data frame, with change of endianness eventually
 * @param inEntry input ranging data frame entry
 * @param inEntryAsLe if true, all 32/16 bits fields of sub struct of in entry are stored as little endian. Remember that an entry coming from lower layer is in big endian
 * @param outEntry output ranging data frame entry
 * @param outEntryAsLe if true, all 32/16 bits fields of sub struct of out entry will be stored as little endian. Remember that an entry coming from lower layer is in big endian
 * @return <0 on error, 0 on success
 */
OSAL_s32 ranging_data_depayload_dup_entry(	rng_protocol_entry_t * inEntry,OSAL_bool_t inEntryAsLe,rng_protocol_entry_t * outEntry,OSAL_bool_t outEntryAsLe);
/**
 * @brief ranging_data_depayload_getConfig
 * @param configLE
 * @param configBE
 * @param uwbPayloadLenInBits
 * @param totalSizeInBytes
 * @param sensor0_activated
 * @param sensor1_activated
 * @param sensor2_activated
 * @param sensor3_activated
 * @param sensor4_activated
 * @param sensor5_activated
 * @param stats_activated
 * @param stats_index
 * @param edata_activated
 * @param toa_activated
 * @param measure_activated
 * @param energy_activated
 * @param phy_activated
 * @param iw_activated
 * @param littleEndian_activated
 * @param slot_ignored
 * @return <0 on error, 0 on success
 */
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
						OSAL_bool_t slot_ignored);
/**
 * @brief ranging_data_depayload_getConfigSizes
 * @note get a size corresponding to a ranging config (LE (or BE). If both are given and LE!=BE , return -1)
 * @param configLE the config to analyze, Little endianess
 * @param configBE the config to analyze, Big endianess
 * @param uwbPayloadLenInBits if given, return size of uwb Payload in bits
 * @param totalSizeInBytes if given, return size of {uwb Payload+ranging data} in bytes
 * @return  <0 on error, 0 on success
 */
OSAL_s32 ranging_data_depayload_getConfigSizes(	rng_config_e *configLE,
							rng_config_e *configBE,
							OSAL_u16* uwbPayloadLenInBits,
							OSAL_u16* totalSizeInBytes);
/**
 * @brief ranging_data_depayload_getLocData
 * @note parse and retrieve from an entry all ranging information, and fill corresponding field
 * @param entry[in] pointer to input entry to pars
 * @param entryAsLe if true, all 32/16 bits fields of sub struct of in entry are stored as little endian. Remember that an entry coming from lower layer is in big endian
 * @param typeTrame[out] type of trame of this entry
 * @param rdvSubType[out] type of subtrame of this entry (rdv only)
 * @param totalEntrySize[out] entrysize in bytes
 * @param b[out] pointer on, beacon unified basics info for this entry, if this entry is a beacon unified
 * @param b1[out] pointer on, beacon 1 basics info for this entry, if this entry is a beacon 1
 * @param b2[out] pointer on, beacon 2 basics info for this entry, if this entry is a beacon 2
 * @param rng[out] pointer on, ranging basics info for this entry, if this entry is a ranging
 * @param rdv[out] pointer on, RDV basics info for this entry, if this entry is a RDV
 * @param ltoa[out] pointer on, loc toa info, if this entry rngDataType has been configured to get loc toa
 * @param lmet[out] pointer on, metrics info, if this entry rngDataType has been configured to get metrics
 * @param lnrj[out] pointer on, nrj info, if this entry rngDataType has been configured to get nrj
 * @param lphy[out] pointer on, phy info, if this entry rngDataType has been configured to get phy
 * @param les[out] pointer on, estimates info, if this entry rngDataType has been configured to get estimates
 * @return  <0 on error, 0 on success. 0 is return if data are valid but overloaded. So  dataoverloaded is important
 */
OSAL_s32 ranging_data_depayload_getLocData(rng_protocol_entry_t * entry,
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
						loc_estimate_complete_t** lescomplete);


/************************************** UWB DATA PART IMPLEMENTATION *************/

/**
 * @brief free_stats_data_t
 * @note free a stats data structure allocated through ranging_data_depayload_getUWBData
 * @param sensor memory to be freed
 */
OSAL_void free_stats_data_t(stats_data_t** stats);

/**
 * @brief free_sensor_data_t
 * @note free a sensor data structure allocated through ranging_data_depayload_getUWBData
 * @param sensor memory to be freed
 */
OSAL_void free_sensor_data_t(sensor_data_t** sensor);

/**
 * @brief free_ext_data_t
 * @note free an ext data structure allocated through ranging_data_depayload_getUWBData
 * @param ed memory to be freed
 */
OSAL_void free_ext_data_t(ext_data_t** ed);

/**
 * @brief dup_stats_data_t
 * @note duplicate a stats data structure
 * @note Since data size of stats are FIXED during session, if *, including (struct)->data for ext_data_t or sensor_data_t is not NULL, previous buffer will be used and erased.
 * @note If *out is NULL, function will allocate it. It's this function caller responsability to free data
 * @param in the stats to duplicate
 * @param out the stats to be created/changed
 * @return  <0 on error, 0 on success. 0 is return if data are valid but overloaded. So  dataoverloaded is important
 */
OSAL_s32 dup_stats_data_t(stats_data_t* in,stats_data_t** out);

/**
 * @brief dup_sensor_data_t
 * @note duplicate a stats data structure
 * @note Since data size of sensor are FIXED during session, if *out is not NULL, previous buffer will be used and erased.
 * @note If *out is NULL, function will allocate it. It's this function caller responsability to free data, including (out)->data
 * @param in the sensor to duplicate
 * @param out the sensor to be created/changed
 * @return  <0 on error, 0 on success. 0 is return if data are valid but overloaded. So  dataoverloaded is important
 */
OSAL_s32 dup_sensor_data_t(sensor_data_t* in,sensor_data_t** out);

/**
 * @brief dup_ext_data_t
 * @note duplicate a stats data structure
 * @note Since data size of ext data are FIXED during session(see ranging_data_depayload_init), if *out is not NULL, previous buffer will be used and erased.
 * @note If *out is NULL, function will allocate it. It's this function caller responsability to free data, including (out)->data
 * @note this can be done through free_ext_data_t
 * @param in the ext data to duplicate
 * @param out the ext data to be created/changed
 * @return  <0 on error, 0 on success. 0 is return if data are valid but overloaded. So  dataoverloaded is important
 */
OSAL_s32 dup_ext_data_t(ext_data_t* in,ext_data_t** out);

/**
 * @brief extdata_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for external data.
 * @note this size will be the same than size set through ranging_data_depayload_init
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
OSAL_u32 extdata_depayload_getSizeInBitsLE(OSAL_u32 _rngDataTypeLE);

/************************************** Get them all  *************/
/** if sensor_x_data_t** sX, stats_data_t** stats, extdata_data_t** ed are given, it will be allocated by this function, if data of this kind are available
 * Its this function 's caller responsability to free it after having used 
*/
/**
 * @brief ranging_data_depayload_getUWBData
 * @note for any member of this function,  If member is NULL, data won't be retrieved.
 * @note Since data size of sensor/stats are FIXED during session, if *member is not NULL, previous buffer will be used and erased.
 * @note If *member is NULL, function will allocate it. It's this function caller responsability to free data, including (struct)->data for ext_data_t or sensor_data_t
 * @param entry[in] pointer to input entry to parse
 * @param entryAsLe if true, all 32/16 bits fields of sub struct of in entry are stored as little endian. Remember that an entry coming from lower layer is in big endian
 * @param s0[out] sensor 0 data information filled.See note above for allocation
 * @param s0Activ[out] true if sensor 0 is activated within current config of this entry, false otherwise
 * @param s1[out] sensor 1 data information filled.See note above for allocation
 * @param s1Activ[out] true if sensor 1 is activated within current config of this entry, false otherwise
 * @param s2[out] sensor 2 data information filled.See note above for allocation
 * @param s2Activ[out] true if sensor 2 is activated within current config of this entry, false otherwise
 * @param s3[out] sensor 3 data information filled.See note above for allocation
 * @param s3Activ[out] true if sensor 3 is activated within current config of this entry, false otherwise
 * @param s4[out] sensor 4 data information filled.See note above for allocation
 * @param s4Activ[out] true if sensor 4 is activated within current config of this entry, false otherwise
 * @param s5[out] sensor 5 data information filled.See note above for allocation
 * @param s5Activ[out] true if sensor 5 is activated within current config of this entry, false otherwise
 * @param stats[out] stats information filled.See note above for allocation
 * @param statsActiv[out] true if statistics is activated within current config of this entry, false otherwise
 * @param ed[out] external data information filled.See note above for allocation
 * @param edActiv[out] true if external data is activated within current config of this entry, false otherwise
 * @return  <0 on error, 0 on success. 0 is return if data are valid but overloaded. So  dataoverloaded is important. In case of error, no memory has been allocated
 */
OSAL_s32 ranging_data_depayload_getUWBData(	rng_protocol_entry_t * entry,
						OSAL_bool_t entryAsLe,
						sensor_data_t** s0, OSAL_bool_t* s0Activ,
						sensor_data_t** s1, OSAL_bool_t* s1Activ, 
						sensor_data_t** s2, OSAL_bool_t* s2Activ,
						sensor_data_t** s3, OSAL_bool_t* s3Activ,
						sensor_data_t** s4, OSAL_bool_t* s4Activ,
						sensor_data_t** s5, OSAL_bool_t *s5Activ,
						stats_data_t** stats, OSAL_bool_t* statsActiv,
						ext_data_t** ed, OSAL_bool_t* edActiv);



#ifdef __cplusplus
}
#endif


#endif //#ifndef _RANGING_DATA_DEPAYLOAD_H_

