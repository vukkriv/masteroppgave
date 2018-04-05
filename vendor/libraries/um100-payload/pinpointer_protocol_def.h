/**
 * @file pinpointer_protocol_def.h
 * @brief enum and structure definition, used by UWB stack specific to bespoon protocol
 * @author www.bespoon.com
 * @date Bespoon 2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */


#ifndef PINPOINTER_PROTOCOL_DEF_H
#define PINPOINTER_PROTOCOL_DEF_H

//#################################
// DEFINE SPECIFIC TO BSP PROTOCOL
//#################################

/**
* @brief MAX_FRAME_PER_SUPERFRAME number max of frame per a superframe
*/
#define MAX_FRAME_PER_SUPERFRAME	32

/**
 * @brief MAX_LISTENER_NUMBER : number or potential listener for rng data. 
 * @note a listener is an instance in server, bridge, viewer, whatever that will be interested in a certain part of data in rng data
 * @note rng data are stored in listener order, mean rng data 
 */
#define MAX_LISTENER_NUMBER		8

//#################################
// MACRO SPECIFIC TO BSP PROTOCOL
//#################################

/**
 * @brief RGN_INFO_SEQ_STATUS : Extract info from status field of	the rng_data structure
 */
#define RGN_INFO_SEQ_STATUS(error) ((error) & 0xFFFF)		/**< last 16bits are reserved for sequencer status */

/**
 * @brief RGN_INFO_DEFRAME_STATUS : Extract info from status field of	the rng_data structure
 */
#define RGN_INFO_DEFRAME_STATUS(error) (((error) >> 16) & 0x01) /**< bit 16 is deframing status: 1 ok, 1 ko */

/**
 * @brief RGN_INFO_SIGN_STATUS : Extract info from status field of	the rng_data structure
 */
#define RGN_INFO_SIGN_STATUS(error) (((error) >> 18) & 0x01) 	/**< bit 18 is positive status: 1 ranging positive, 
									 0 ranging negative */
/**
 * @brief RQI_GET_PARASIZE : Extract parabola size used for MED from RQI (Ranging QualIty)
 */
#define RQI_GET_PARASIZE(rqi) ((rqi) & 0xF)

/**
* @brief RQI_GET_LOSINDICATOR : Extract LOS indicator from RQI (Ranging QualIty)
*/
#define RQI_GET_LOSINDICATOR(rqi) (((rqi) >> 4) & 0xF)

/**
 * @brief RQI_SET_PARASIZE : Set parabola size in RQI (Ranging QualIty)
 */
#define RQI_SET_PARASIZE(rqi, parasize) ((rqi) |= ((parasize) & 0xF))

/**
* @brief RQI_SET_LOSINDICATOR : Set LOS indicator in RQI (Ranging QualIty)
*/
#define RQI_SET_LOSINDICATOR(rqi, losindicator) ((rqi) |= (((losindicator) << 4) & 0xF))


//#################################
// MACRO SPECIFIC TO UWB_DATA
//#################################
/**
 * @brief UWBDATA_SENSOR0_SIZE_IN_BITS : The size used by sensor 0, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR0_SIZE_IN_BITS 1
/**
 * @brief UWBDATA_SENSOR0_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 0, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR0_ROUNDEDSIZE_IN_BYTES 1
/**
 * @brief UWBDATA_SENSOR1_SIZE_IN_BITS : The size used by sensor 1, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR1_SIZE_IN_BITS 1
/**
 * @brief UWBDATA_SENSOR1_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 1, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR1_ROUNDEDSIZE_IN_BYTES 1
/**
 * @brief UWBDATA_SENSOR2_SIZE_IN_BITS : The size used by sensor 2, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR2_SIZE_IN_BITS 20
/**
 * @brief UWBDATA_SENSOR2_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 2, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR2_ROUNDEDSIZE_IN_BYTES 3
/**
 * @brief UWBDATA_SENSOR3_SIZE_IN_BITS : The size used by sensor 3, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR3_SIZE_IN_BITS 48
/**
 * @brief UWBDATA_SENSOR3_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 3, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR3_ROUNDEDSIZE_IN_BYTES 6
/**
 * @brief UWBDATA_SENSOR4_SIZE_IN_BITS : The size used by sensor 4, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR4_SIZE_IN_BITS 16
/**
 * @brief UWBDATA_SENSOR4_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 4, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR4_ROUNDEDSIZE_IN_BYTES 2
/**
 * @brief UWBDATA_SENSOR5_SIZE_IN_BITS : The size used by sensor 5, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR5_SIZE_IN_BITS 16
/**
 * @brief UWBDATA_SENSOR5_ROUNDEDSIZE_IN_BYTES : The rounded size in bytes used by sensor 5, fixed/managed by bespoon
 */
#define UWBDATA_SENSOR5_ROUNDEDSIZE_IN_BYTES 2
/**
 * @brief UWBDATA_STATS_SIZE_IN_BITS_FIXED : The size used by stats, fixed/managed by bespoon, case of fixed stat
 */
#define UWBDATA_STATS_SIZE_IN_BITS_FIXED 32
/**
 * @brief UWBDATA_STATS_ROUNDEDSIZE_IN_BYTES_FIXED : The rounded size in bytes used by stats, fixed/managed by bespoon, case of fixed stat
 */
#define UWBDATA_STATS_ROUNDEDSIZE_IN_BYTES_FIXED 4
/**
 * @brief UWBDATA_STATS_SIZE_IN_BITS_FIXED : The size used by stats, fixed/managed by bespoon, case of rotate stats
 */
#define UWBDATA_STATS_SIZE_IN_BITS_ROTATE 40
/**
 * @brief UWBDATA_STATS_ROUNDEDSIZE_IN_BYTES_FIXED : The rounded size in bytes used by stats, fixed/managed by bespoon, case of rotate stats
 */
#define UWBDATA_STATS_ROUNDEDSIZE_IN_BYTES_ROTATE 5
/**
* @brief UWBDATA_EXTDATA_SIZE_IN_BITS_BASE : The size used by ext data (tuned to 94 bytes to be sent). Base size is fixed/managed by bespoon, but value can differs on implementation
*/
#define UWBDATA_EXTDATA_SIZE_IN_BITS_BASE 752
/**
* @brief UWBDATA_EXTDATA_ROUNDEDSIZE_IN_BYTES_BASE : The rounded size in bytes used by ext data (tuned to 94 bytes to be sent). Base size is fixed/managed by bespoon, but value can differs on implementation
*/
#define UWBDATA_EXTDATA_ROUNDEDSIZE_IN_BYTES_BASE 94

/**
 * @brief sensor0_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 0
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor0_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_0_MASK)?UWBDATA_SENSOR0_SIZE_IN_BITS:0)
/**
 * @brief sensor1_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 1
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor1_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_1_MASK)?UWBDATA_SENSOR1_SIZE_IN_BITS:0)
/**
 * @brief sensor2_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 2
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor2_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_2_MASK)?UWBDATA_SENSOR2_SIZE_IN_BITS:0)
/**
 * @brief sensor3_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 3
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor3_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_3_MASK)?UWBDATA_SENSOR3_SIZE_IN_BITS:0)
/**
 * @brief sensor4_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 4
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor4_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_4_MASK)?UWBDATA_SENSOR4_SIZE_IN_BITS:0)
/**
 * @brief sensor5_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 5
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define sensor5_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_SENSOR_5_MASK)?UWBDATA_SENSOR5_SIZE_IN_BITS:0)
/**
 * @brief sensor5_depayload_getSizeInBits
 * @note get size used by BeSpoon protocol for sensor 5
 * @param _rngDataTypeLE rangingDataType configuration (  LE only )
 * @return the size of usefull bits for this sensor
 */
#define bspstat_depayload_getSizeInBitsLE(_rngDataTypeLE) ((_rngDataTypeLE&RNG_CONF_STATS_MASK)?\
														(((RNG_CONF_STATS_INDEX(_rngDataTypeLE)-1)>=STATS_ROTATE)?\
															UWBDATA_STATS_SIZE_IN_BITS_ROTATE:UWBDATA_STATS_SIZE_IN_BITS_FIXED):\
														0)

//#################################
// ENUMS SPECIFIC TO BSP PROTOCOL
//#################################

/**
 *	@brief afc_config_e : contains afc settings
 */
typedef enum
{
	AFC_CLOCKTRACK_FLAG	= (1<<0),	/**< Do AFC with clock track values */
	AFC_BEACONS_FLAG	= (1<<1)	/**< Do AFC with measures between beacon 1 & beacon 2 */
} OSAL_PACKED_ENUM afc_config_e;

/**
 *	@brief devcaps_bitfield_e : Device Capabilities
*/
typedef enum{
	DEVCAPS_UWB_SLAVE			= 0x00000001,	/**< Device should find a coordinator and ask him for the configuration to use via the RDV slot*/
	DEVCAPS_UWB_MASTER			= 0x00000002,	/**< Device should answer request on RDV slot */
	DEVCAPS_ROOT				= 0x00000004,	/**< Device is synchronisation root device (e.g. do not use other beacon to sync its clock, instead other devices sync on it) */ 
	DEVCAPS_TYPE_MASK			= 0x00000007,	/**< MAsk on type of device */ 
	DEVCAPS_FILTER_NULL			= 0x00001000,	/**< Device wont notify for ranging wiche src and dst are =0*/
	DEVCAPS_AGILE				= 0x00010000,	/**< Device agile mode */
	DEVCAPS_FAKEADDR			= 0x00020000	/**< Instead of using its own hash in ranging frame, tag use the slot number as a hash. This is used for stress tests */
} OSAL_PACKED_ENUM devcaps_bitfield_e;

/**
 * @brief type_trame_t Trame typeof frame enumeration
*/
typedef enum {
	TT_UNUSED 		= 0,		/**< Unused type. This allow to detect for unalignement with server */
	TT_BEACON  		= 1,		/**< Unified Beacon (send by coord) */
	TT_BEACON1 		= 2,		/**< Beacon 1 (send by coord) */
	TT_BEACON2 		= 3,		/**< Beacon 2 (send by coord) */
	TT_TAG_RANGING	= 4,		/**< Ranging data (send by tag) */
	TT_DATA			= 5,		/**< Data slot  */
	TT_RDV 			= 6,		/**< Rendez Vous */
	TT_UNKNOWN		= 0xFF  	/**< unknown frame */
} OSAL_PACKED_ENUM type_trame_t;

/**
 * @brief type_rdv_t subtype for RDV slot enumeration
*/

/**
 * @brief type_rdv_t subtype for RDV slot enumeration
*/
typedef enum {
	RDV_ACK = 0,						/**< RDV type is ACK*/
	RDV_RSVD1 = 1,						/**< Reserved */
	RDV_PAIRING_REQUEST = 2,			/**< RDV type is pairing request*/
	RDV_SIMPLE_PAIRING_REPLY = 3,		/**< RDV type is pairing reply*/
	RDV_DISSOCIATION_REQUEST = 4,		/**< RDV type is dissociation request*/
	RDV_DISSOCIATION_NOTIFICATION = 5,	/**< RDV type is dissociation notification*/
	RDV_PAIRING_NACK,					/**< RDV type is ACK*/
	RDV_SLOT_REQUEST,					/**< RDV type is request for a SLOT */
	RDV_SLOT_GRANTED,					/**< RDV type is SLOT has been granted*/
	RDV_PING,							/**< RDV type is PING */
	RDV_INFO_REQUEST,					/**< RDV type is request for INFO */
	RDV_DEVICE_INFO_REPLY,				/**< RDV type is request for INFO reply */
	RDV_FRIENDLY_NAME_REPLY,			/**< RDV type is request for Name reply */
	RDV_RSVD13,							/**< Reserved */
	RDV_RSVD14,							/**< Reserved */
	RDV_ZEROCONF,						/**< Used by device to discover the PAN infrastructure + may be used also for clock calibration */
	RDV_RSVD16,							/**< Reserved */
	RDV_RSVD17,							/**< Reserved */
	RDV_RSVD18,							/**< Reserved */
	RDV_RSVD19,							/**< Reserved */
	RDV_RSVD20,							/**< Reserved */
	RDV_RSVD21,							/**< Reserved */
	RDV_RSVD22,							/**< Reserved */
	RDV_RSVD23,							/**< Reserved */
	RDV_RSVD24,							/**< Reserved */
	RDV_RSVD25,							/**< Reserved */
	RDV_RSVD26,							/**< Reserved */
	RDV_RSVD27,							/**< Reserved */
	RDV_RSVD28,							/**< Reserved */
	RDV_RSVD29,							/**< Reserved */
	RDV_RSVD30,							/**< Reserved */
	RDV_RSVD31,							/**< Reserved */
	RDV_LAST=RDV_RSVD31,				/**< Indicat last possible rdv on 5 bits */
	RDV_UNKNOWN = 255					/**< Special internal value when scheduling a RX of RDV frame but of unknown size */
} OSAL_PACKED_ENUM type_rdv_t;

/**
 * @brief framer_data_slot_t 
*/
typedef enum {
	DS_BROADCAST_TRANSMITING = 0,	/**< Master has something to say */
	DS_BROADCAST_LISTENING = 1,		/**< Slave is listenning */
	DS_DEDICATED_TRANSMITTING = 2,	/**< Master is in transaction */
	DS_DEDICATED_LISTENING = 3		/**< Slave is in transaction */
} OSAL_PACKED_ENUM framer_data_slot_t;

/*
 * //brief type_rng_t
*/
typedef enum {
	RNGT_BASIC = 0,			// please coment me 
	RNGT_THRESHOLD = 1,		// please coment me 
	RNGT_UNKNOWN = 255		// please coment me 
} OSAL_PACKED_ENUM type_rng_t;

/*
 * //brief framer_auth_type_request_t 
*/
typedef enum {
	ATR_SHORT_OPEN = 0,		// please coment me
	ATR_EXT_OPEN   = 1,		// please coment me
	ATR_SECURE     = 2		// please coment me
} OSAL_PACKED_ENUM framer_auth_type_request_t;

/*
 * //brief framer_loc_type_request_t 
*/
typedef enum {
	LTR_SLOWEST = 1,		// please coment me 
	LTR_STANDARD = 2,		// please coment me 
	LTR_FASTEST = 3		// please coment me 
} OSAL_PACKED_ENUM framer_loc_type_request_t;


/**
 * @brief rng_config_e Type of ranging data
 * This field represents a slot configuration. 
 * All field returned in a rngData are in big endian order
 * Interpretation of this bitfield will differ depending on frame type : \n
 * 
 * <small><tt><pre> bit     | 31        ...     12  | 11            ...         0 |</pre></tt></small>
 * <small><tt><pre> Indicate| common configuration  | type specific configuration |</pre></tt></small>
 * 
 * 
 * The common part will indicate to data retriever some common configuration:
 * - 'bit12'  : Activate toa_le retrieving, info in loc_toa_t structure.
 * - 'bit13'  : Activate nrj summary information retrieving, info in loc_metrics_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit14'  : Activate finger summary, info in loc_nrj_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit15'  : Activate phy config basic summary, info in loc_phy_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit16'  : Activate iw, prp and nrj for pulse reconstruction, info in loc_estimate_nrj_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit17'  : Activate iw, prp, I and Q for pulse reconstruction wiht IQ, info in loc_estimate_iq_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit18'  : Activate iw, prp, nrj I,Q , arctan for pulse reconstruction, info in loc_estimate_complete_t structure. 32bits and 16bits field in this structure are in big endian order.
 * - 'bit19 to 25'  : reserved
 * - 'bit26'  : If 0(default), slot considered. If 1, slot is ignored ( not reported from protocol to outer world )
 * - 'bit27 to bit30': Frame Type : see #type_trame_t
 * - 'bit30: reserved
 *
 * <b>Note</b>: 
 * <i> For More information retrievieng, either bit16,bit17 or bit18 should be selected. If bit16,bit17 or bit16,bit17,bit18 are selected, on data push, only bit 18 will be set.
 * On product with smaller memory, bit18 cannot be used. Then, if bit18 is set, bit16 will be set on data push. </i>
 *
 * <b> INTERPRETATION FOR BEACON FRAME </b>
 * In this case, bit0 to bit11 are unused. Thus, 'bit24 to bit31' can be ignored too.

 * <b> INTERPRETATION FOR BEACON1 FRAME </b>
 * In this case, bit0 to bit11 are unused. Thus, 'bit24 to bit31' can be ignored too.
 *
 * <b> INTERPRETATION FOR BEACON2 FRAME </b>
 * In this case, bit0 to bit11 are unused. Thus, 'bit24 to bit31' can be ignored too.
 *
 * <b> INTERPRETATION FOR RANGING FRAME </b>
 * Bits will indicate which information is returned in output buffer:
 * - 'bit00'  : Sensor 0 (Battery on 1 bit in reference design). 1 if activated,0 otherwise.
 * - 'bit01'  : Sensor 1 (Moving on 1 bit in reference design). 1 if activated,0 otherwise
 * - 'bit02'  : Sensor 2 (TSA Confidence on 7 bits in reference design). 1 if activated,0 otherwise.
 * - 'bit03'  : Sensor 3 (Accelerometer data on 180 bits in reference design). 1 if activated,0 otherwise.
 * - 'bit04'  : Sensor 4. 1 if activated,0 otherwise.
 * - 'bit05'  : Sensor 5. 1 if activated,0 otherwise.
 * - 'bit0610': Bespoon Stats activated index. Deactivated if 0. Value-1 indicate index of returned Stats (see #protocol_stats_type_t). Rotate if >= STATS_ROTATE
 * - 'bit11'  : External Data. 1 if activated,0 otherwise. 
 *
 * <b> INTERPRETATION FOR DATA FRAME </b>
 * Data frame act like ranging frame. There are under specification for now. Currently, in this case, bit0 to bit11
 * are unused. Thus, 'bit24 to bit31' can be ignored too.
 *
 * <b> INTERPRETATION FOR RDV FRAME </b>
 * In this case, bit0 to bit11 are unused. Thus, 'bit24 to bit31' can be ignored too.
 *
 * <b> Form outter world point of view : </b>
 * Each listener (server, cleint, ...) can treat and remove the data it is interrested in.
 * Once its done, and before forwarding data to above listener, if data are not needed by above listener, stage can reconcanate rng data, and decrease total size.
 * In this case, rngDataType must be changed to match the new config
 * Example:Assume  All section are used,
 * Buffer contain data corresponding to Section bit0,bit1,bit2,bit3,bit4,bit5,bit6
 *   listener0 want section bit0,    ,bit2,    ,bit4,bit5
 *   listener1 want section bit1,bit2,bit3,bit4,bit6
 *   listener2 want section bit0,bit1,bit3
 *   listener3 want section bit1,bit2,bit3,bit4
 *   listener4 want section bit0,bit1,bit3
 *   listener5 want section bit1
 * 
 *  - listener 0 can remove data corresponding to Section bit5, and reset bit5 in rng config to 0
 *    => Buffer contain data corresponding to Section bit0,bit1,bit2,bit3,bit4,bit6
 *  - listener 1 can remove data corresponding to Section bit6, and reset bit6 in rng config to 0
 *    => Buffer contain data corresponding to Section bit0,bit1,bit2,bit3,bit4
 *  - listener 2 can't remove anything
 *    => Buffer contain data corresponding to Section bit0,bit1,bit2,bit3,bit4
 *  - listener 3 can remove data corresponding to Section bit2 and bit4, and reset bit2 and bit4 in rng config to 0
 *    => Buffer contain data corresponding to Section bit0,bit1,bit3
 *  - listener 4 can remove data corresponding to Section bit0 and bit3, and reset bit0 and bit3 in rng config to 0
 *    => Buffer contain data corresponding to Section bit1
 *  - listener 5 treat last data
 */
 
typedef enum {
	//################### COMMON CONFIG ######################################

	/* BIT 12 - 25 : All loc mask */
	RNG_CONF_LOC_MASK	= (0x03FFF000),		/**< All loc mask, including reserved one*/
	
	/* BIT 12 : Activate toa_le retrieving for beacon unified, beacon1 or beacon 2 */
	RNG_CONF_LOC_TOA_MASK	= (1<<12),		/**< Mask for Activate toa_le retrieving*/

	/* BIT 13 : Activate metrics summary information retrieving, info in loc_metrics_t structure */
	RNG_CONF_LOC_METR_MASK	= (1<<13),	/**< Mask for Activate nrj summary information retrieving */

	/* BIT 14 : Activate finger summary, info in loc_nrj_t structure*/
	RNG_CONF_LOC_NRJ_MASK	= (1<<14),	/**< Mask for Activate finger summary */

	/* BIT 15 : Activate phy config basic summary, info in loc_phy_t structure*/
	RNG_CONF_LOC_PHY_MASK	= (1<<15),	/**< Mask for Activate phy config basic summary */

	/* BIT 16 : Activate iw, prp and nrj for pulse reconstruction, info in loc_estimate_nrj_t structure */
	RNG_CONF_LOC_ES_NRJ_MASK	= (1<<16),	/**< Mask for Activate iw, prp and nrj*/

	/* BIT 17 : Activate iw, prp and I,Q for pulse reconstruction, info in loc_estimate_iq_t structure */
	RNG_CONF_LOC_ES_IQ_MASK	= (1<<17),	/**< Mask for Activate iw, prp and I,Q*/

	/* BIT 18 : Activate iw, prp, nrj, I,Q arctan for pulse reconstruction, info in loc_estimate_complete_t structure */
	RNG_CONF_LOC_ES_COMPLETE_MASK	= (1<<18),	/**< Mask for Activate iw, prp, nrj, IQ arctan*/

	/* BIT 19 - BIT 25 : Reserved for future loc conf mask*/
	RNG_CONF_RSVD_BIT19_25_MASK	= (0x003F80000) , /**< reserved bits  */

	/* BIT 26: Indicate this slot will be ignored */ 
	RNG_CONF_IGNORE_SLOT_MASK	= (1<<26),		/**< Mask indicating this slot's metrics wont be send to user space. By default, all frame are sent(if FULL LOC activated)*/

	/* BIT 27 to 30 : indicate frame type */
	RNG_CONF_FTYPE_RNGMASK 		= (0xF<<27),	/**< Mask for TT type conf, on 32bits: see type_trame_t*/
			#define RNG_CONF_GET_TTTYPE(_conf)		(((unsigned) (_conf)&RNG_CONF_FTYPE_RNGMASK)>>27)
			#define RNG_CONF_FTYPE_IS_BU(_conf)		(RNG_CONF_GET_TTTYPE(_conf)==TT_BEACON) /**< Macro to check if rngdatatype concern a beacon unifed slot*/
			#define RNG_CONF_FTYPE_IS_B1(_conf)		(RNG_CONF_GET_TTTYPE(_conf)==TT_BEACON1) /**< Macro to check if rngdatatype concern beacon 1 slot*/
			#define RNG_CONF_FTYPE_IS_B2(_conf)		(RNG_CONF_GET_TTTYPE(_conf)==TT_BEACON2) /**< Macro to check if rngdatatype concern beacon 2  slot*/
			#define RNG_CONF_FTYPE_IS_RNG(_conf)	(RNG_CONF_GET_TTTYPE(_conf)==TT_TAG_RANGING) /**< Macro to check if rngdatatype concern a ranging  slot*/
			#define RNG_CONF_FTYPE_IS_DATA(_conf)	(RNG_CONF_GET_TTTYPE(_conf)==TT_DATA) /**< Macro to check if rngdatatype concern a data slot*/
			#define RNG_CONF_FTYPE_IS_RDV(_conf)	(RNG_CONF_GET_TTTYPE(_conf)==TT_RDV) /**< Macro to check if rngdatatype concern a rdv slot*/
			#define RNG_CONF_SET_FTYPE_X(_conf,_x) do{\
					_conf &= (~RNG_CONF_FTYPE_RNGMASK);\
					_conf |= ((_x&0xF)<<27);\
				}while(0)																/**< Macro to set any slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_B(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_BEACON)	/**< Macro to set beacon unifed as slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_B1(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_BEACON1)	/**< Macro to set beacon 1 as slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_B2(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_BEACON2)	/**< Macro to set beacon 2 as slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_RNG(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_TAG_RANGING)	/**< Macro to set ranging as slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_DATA(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_DATA)	/**< Macro to set data as slot type in rngdatatype*/
			#define RNG_CONF_SET_FTYPE_RDV(_conf) RNG_CONF_SET_FTYPE_X(_conf,TT_RDV)	/**< Macro to set rdv as slot type in rngdatatype*/

	/* BIT 31  : Reserved */
	RNG_CONF_RSVD_BIT31_MASK	= (0x80000000) , /**< reserved bits  */


	//################### BEACON SPECIFIC ######################################
	/* BIT 0 - BIT 11 : Reserved */
	BEA_CONF_RSVD_BIT0_11_MASK	= (0xFFF) , /**< reserved bits for Beacon (unifed,1 or 2) */

	//################### RANGING SPECIFIC ######################################
	/* BIT0 : Sensor 0*/
	RNG_CONF_SENSOR_0_MASK	= (1<<0),	/**< Mask for Sensor 0*/
	/* BIT1 : Sensor 0*/
	RNG_CONF_SENSOR_1_MASK	= (1<<1),	/**< Mask for Sensor 1*/
	/* BIT2 : Sensor 0*/
	RNG_CONF_SENSOR_2_MASK	= (1<<2),	/**< Mask for Sensor 2*/
	/* BIT3 : Sensor 0*/
	RNG_CONF_SENSOR_3_MASK	= (1<<3),	/**< Mask for Sensor 3*/
	/* BIT4 : Sensor 0*/
	RNG_CONF_SENSOR_4_MASK	= (1<<4),	/**< Mask for Sensor 4*/
	/* BIT5 : Sensor 0*/
	RNG_CONF_SENSOR_5_MASK	= (1<<5),	/**< Mask for Sensor 5*/
	#define RNG_CONF_SENSOR_NUMBER 6		 /**< total number of sensor*/
	/* BIT6 to BIT10 : Stats*/
	RNG_CONF_STATS_MASK		= (0x1F<<6),	/**< Mask for BeSpoon stats*/
			#define RNG_CONF_STATS_INDEX(_conf) 		(((unsigned) (_conf)&RNG_CONF_STATS_MASK)>>6) /**< macro to get stats index */
			#define RNG_CONF_STATS_INDEX2CONF(_index)	(((unsigned) ((_index+1)&0x3F)<<6)) /**< macro to set stats into a rngdatatype */
	/* BIT11 : External data*/
	RNG_CONF_EXTDATA_MASK	= (1<<11),		/**< Mask for External data*/
	RNG_CONF_UWB_DATA_MASK	= 0x00000FFF,	/**< Mask indicating something is configured on uwb data part*/

	//################### DATA SPECIFIC ######################################
	/* BIT 0 - BIT 11 : Reserved */
	DATA_CONF_RSVD_BIT0_11_MASK	= (0xFFF) , /**< reserved bits for Beacon (unifed,1 or 2) */

	//################### RDV SPECIFIC ######################################
	/* BIT 0 - BIT 11 : Reserved */
	RDV_CONF_RSVD_BIT0_11_MASK	= (0xFFF)  /**< reserved bits for Beacon (unifed,1 or 2) */

} OSAL_PACKED_ENUM rng_config_e;

/**
 * @brief fct_payload_init : function pointer for payload initialisation prototype 
 */
typedef OSAL_void (*fct_payload_init) (OSAL_void*);
/**
 * @brief fct_payload_deinit : function pointer for payload deinitialisation prototype 
 */
typedef OSAL_void (*fct_payload_deinit) (OSAL_void*);
/**
 * @brief fct_payload_getData : function pointer for payload get data prototype 
 */
typedef OSAL_void (*fct_payload_getData) (OSAL_void*,OSAL_u8**,OSAL_u16*);
/**
 * @brief fct_payload_updateData : function pointer for payload update data prototype 
 */
typedef OSAL_void (*fct_payload_updateData) (OSAL_void*);

/**
 * @brief cb_key_type_t : Type of value to set on a callback
 */
typedef enum cb_payload_key_type {
	SCB_INIT	= 0,	/**< Set init callback.If this key is used, value must be an fct_payload_init. If not pass, NULL will be assumed */
	SCB_DEINIT 	= 1,	/**< Set deinit callback.If this key is used, value must be an fct_payload_deinit. If not pass, NULL will be assumed */
	SCB_GETDATA = 2,	/**< Set get data callback.If this key is used, value must be an fct_payload_getData. If not pass, NULL will be assumed */
	SCB_UPDATE 	= 3,	/**< Set update callback.If this key is used, value must be an fct_payload_updateData. If not pass, NULL will be assumed */
	SCB_UDATA 	= 4	/**< Set user data of callback.If this key is used, value must be an OSAL_void*. If not pass, NULL will be assumed */
} OSAL_PACKED_ENUM cb_payload_key_type_t;


/**
 * @brief frame_info_type_t : Type of slot that can be scheduled in a superframe schema
 */
typedef enum frame_info_type {
	FIT_SCAN	= 0,	/**< Schedule a SCAN */
	FIT_BEACON1 	= 1,	/**< Schedule a BEACON 1 */
	FIT_BEACON2 	= 2,	/**< Schedule a BEACON 2 */
	FIT_RANGING 	= 3,	/**< Schedule a RANGING */
	FIT_RDV		= 4,	/**< Schedule a frame on annoncement slot */
	FIT_RACH 	= FIT_RDV	/**< same as FIT_RDV (for backward compatibility) */
} OSAL_PACKED_ENUM frame_info_type_t;

/**
 * @brief dev_status_t device state enumeration
*/
typedef enum {
	DS_DETACHED = 0,	/**< Device detached */
	DS_ATTACH_REQUEST,	/**< Device ask for attaching */
	DS_ATTACHED		/**< Device attached */
} OSAL_PACKED_ENUM dev_status_t;

/**
 * artls_lostatus_t artls Local Status enumeration
*/
typedef enum {
	LOSTATUS_STOPED = 0,	/**< stopped */
	LOSTATUS_SCAN_RF,	/**< scanning */
	LOSTATUS_CALIBRATING,	/**< calibrating */
	LOSTATUS_RANGING,	/**< ranging */
	LOSTATUS_ON_ERROR,	/**< error */
	LOSTATUS_NB		/**< nb of possible state */
} OSAL_PACKED_ENUM artls_lostatus_t;

/**
 *	@brief protocol_stats_type_t Statistics
*/
typedef enum {
	STATS_RX=0,		/**< Number of scheduled RX*/
	STATS_TX,		/**< Number of scheduled TX*/
	STATS_SFD_TO,		/**< Number of SFT timeout*/
	STATS_NOISE_LEVEL,	/**< Noise reference level*/
	STATS_CORRUPTED_FRAME,	/**< Number of FEC error (or bad Rx size)*/
	STATS_CANCELED_SEQ_FRAME,	/**< Number of canceled frames*/
	STATS_DEFRAME_ERROR,	/**< Number of deframe error*/
	STATS_SEQUENCER_ERROR,  /**< Number of error in sequencer*/
	STATS_SEQUENCER_TSA,	/**< time slot adjustment (in ps)*/
	STATS_SCAN,		/**< Number of time the device do scan_rf*/
	STATS_AFC_ERROR,	/**< Number of error in AFC*/
	STATS_AFC_FILTER,	/**< AFC filter output (in ppb)*/
	STATS_RDV_RX,		/**< Number of received frame on RDV slot*/
	STATS_RDV_MISSING_RX,	/**< Number of missing RDV frame*/
	STATS_RDV_TX,		/**< Number of RDV frame sent*/
	STATS_LOCRATE_INFO,	/**< Loc rate info: dynamic flag, SF start and actual loc rate (tag only)*/
	STATS_BEACON_SCORE, 	/**< ref base current score*/
	STATS_RSVD,		/**< reserved for future use or debug */
	STATS_ROTATE,		/**< Rotate upon all stat above*/
	STATS_NB
} OSAL_PACKED_ENUM protocol_stats_type_t;

/**
 * @brief artls_up_cmd_e - tell which field is set in artls_up_t payload
 */
typedef enum {
	ARTLS_LOSTATUS_UP = 0,		/**< Field local_status is filled*/
	ARTLS_LO_REF_BASE_UP,		/**< Field local_ref_base is filled */
	ARTLS_PAIRING_REQUEST_UP,	/**< Field pairing_req is filled */
	ARTLS_INFO_UP,			/**< Field device_info is filled */
	ARTLS_DISSOCIATION_REQUEST_UP,  /**< Field dreq is filled */
	ARTLS_CMD_STATUS_UP		/**< Field cmd_status is filled */
}OSAL_PACKED_ENUM artls_up_cmd_e;

/**
 * @brief artls_down_cmd_e - tell which field is set in artls_down_t payload
 */
typedef enum {
	ARTLS_SIMPLE_PAIRING_REPLY_DOWN = 0,	/**< Send a Simple Pairing Reply frame to a remote tag */
	ARTLS_DISSOCIATION_REQUEST_DOWN = 1,	/**< Send a Dissociation Request frame to a remote tag */
	ARTLS_SET_EXT_DATA_SIZE = 2,			/**< Set the size of external data */
	ARTLS_SET_DEV_ID_DOWN = 10,		/**< Set the device id (a.k.a short address a.k.a hash) */
	ARTLS_SET_PREFERED_REF_BASE_DOWN = 11,	/**< Force the device to bind to a specific ref base */
	ARTLS_SET_THRESHOLD_COEFF_DOWN = 12,	/**< Apply a multiplicative coefficient to all synchronisation threshold */
	ARTLS_SET_ANTENNA_OFFSET_DOWN = 13		/**< Apply a specific hardware offset */
}OSAL_PACKED_ENUM artls_down_cmd_e;

/**
 * @brief artls_command_return_code_t - return code of a artls_down_t command
 */
typedef enum {
	ACC_SUCCESS = 0,	/**< Command finished sucessfully */
	ACC_NO_ACK	= 1,	/**< Remote peer do not ack the command */
	ACC_INTERNAL_ERROR = 2 /**< Command not finished due to internal error */
	// TODO: add timeout, canceled, ...
} OSAL_PACKED_ENUM artls_cmd_return_code_t;

/**
 * @brief artls_tracking_t - define how user want to track an artls_down_t command
 */
typedef enum {
	AT_NO_TRACKING = 0,	/**< Fire and forget, do not notify anything about the end of the command */
	AT_NOTIFY_END		/**< Notify at the end of the command */
} OSAL_PACKED_ENUM artls_tracking_t;

/**
 * @brief asset_tracking_config_e - define how user want to configure asset tracking
 */
typedef enum {
	ASSETTRACKING_ENABLE 		= 0x0001,		/**< Enable / disable sleep between ranging */
	ASSETTRACKING_RACH		= 0x0002,		/**< Enable / disable rach on wake up. If disabled rach data will be restored from first attachment */
	ASSETTRACKING_RANDOM_SLOT	= 0x0004,		/**< Enable / disable random ranging slot. Overwrite ASSETTRACKING_RACH if enabled */
	ASSETTRACKING_AFC		= 0x0008		/**< Enable / disable AFC on each wake up */
} OSAL_PACKED_ENUM asset_tracking_config_e;


//#################################
// STRUCT SPECIFIC TO BSP PROTOCOL
//#################################

/**
 *	@brief protocol_stats_t : contains stat info
 */
typedef struct protocol_stats {
	OSAL_u8  rsvd[4];		/**< Reserved fields. In case of spi transmission, it will be used for ack */
	OSAL_u32 megaframe;	/**< Current mega frame */
	OSAL_u16 hyperframe;	/**< Current hyper frame */
	OSAL_u8	 superframe;	/**< Current super frame */
	artls_lostatus_t	local_status;	/**< Ranging status of the local pinpointer */
	OSAL_u32 entries[STATS_NB];	/**< entries for stats */
} protocol_stats_t;

/**
 *	@brief rng_info : contains information about the ranging
 *	@note This structure is aligned and sized 8 bytes
 */
typedef struct rng_time
{
	OSAL_u32 mega_frame;	/**< Counter incremented each 32 hyperframe*/
	OSAL_u16 hyper_frame;	/**< Counter incremented each 6 superframe */
	OSAL_u8	super_frame;	/**< Superframe counter */
	OSAL_u8	slot;		/**< Number of slot */
} OSAL_PACKED_STRUCTURE rng_time_t; 

/**
 *	@brief rng_protocol_header_t header of nb of packet to be sent 
 *	@note This structure is aligned and is 16 bytes.
 *      @note all field in this structure are filled in big endian
 */
typedef struct rng_protocol_header
{
	OSAL_u32 receiver_hash;		/**< Short ID of ME THAT IM HERE */
	OSAL_u32 ref_base_hash;		/**< Short ID of base on which I am sync */
	OSAL_u32 mega_frame;		/**< Counter incremented each 32 hyperframe ?*/
	OSAL_u16 hyper_frame;		/**< Counter incremented each 6 superframe */
	OSAL_u8	super_frame;		/**< Superframe counter */
	OSAL_u8 nb_entry;		/**< Nb of entry in burst */	
} OSAL_PACKED_STRUCTURE rng_protocol_header_t;

/**
 *	@brief beacon_protocol_entry_basic_t : entry for each slot of type unified beacon
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 12 bytes.
 * 	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct beacon_protocol_entry_basic
{
	OSAL_u32 src_hash;				/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u16 hyperframe_number;		/**< beacon hf number*/
	OSAL_u8 beacon_id;				/**< beacon id */
	OSAL_u8 data_slot;				/**< framer data slot  */
	OSAL_bool_t moving;				/**< moving or not*/
	OSAL_u8 superframe_number;		/**< beacon sf number */
	OSAL_u8 rsvd[2];				/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE beacon_protocol_entry_basic_t;

/**
 *	@brief beacon1_protocol_entry_basic_t : entry for each slot of type BEACON1
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 8 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct beacon1_protocol_entry_basic
{
	OSAL_u32 src_hash;				/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u16 hyperframe_number;		/**< beacon hf number*/
	OSAL_u8 beacon_id;				/**< beacon id */
	OSAL_u8 rsvd;					/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE beacon1_protocol_entry_basic_t;

/**
 *	@brief beacon2_protocol_entry_basic_t : entry for each slot of type BEACON2
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 8 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct beacon2_protocol_entry_basic
{
	OSAL_u32 src_hash;			/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u8 data_slot;			/**< framer data slot  */
	OSAL_bool_t moving;			/**< moving or not*/
	OSAL_u8 superframe_number;	/**< beacon sf number */
	OSAL_u8 rsvd;				/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE beacon2_protocol_entry_basic_t;

/**
 *	@brief rng_protocol_entry_basic : entry for each slot of type RANGING
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 8 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rng_protocol_entry_basic
{
	OSAL_u32 src_hash;		/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u32 dst_hash;		/**< Short ID of device destination.Endianess indicated in rngDataType */
} OSAL_PACKED_STRUCTURE rng_protocol_entry_basic_t;

/**
 *	@brief rdv_ack_protocol_entry_basic_t : entry for each slot of type RDV ACK 
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 12 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rdv_ack_protocol_entry_basic
{
	OSAL_u32 src_hash;		/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u32 dst_hash;		/**< Short ID of device destination.Endianess indicated in rngDataType */
	OSAL_u8 rsvd[4];		/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE rdv_ack_protocol_entry_basic_t;

/**
 *	@brief rdv_pr_protocol_entry_basic : entry for each slot of type RDV PAIRING REQUEST
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 16 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rdv_pr_protocol_entry_basic
{
	OSAL_u32 dst_hash;			/**< Short ID of device destination.Endianess indicated in rngDataType */
	OSAL_u8	src_mac[UWB_MAC_LEN];		/**< UWB Mac address of the device that request pairing */
	OSAL_u8 auth_type_request;	/**< reserved for future use */
	OSAL_u8 caps;				/**< Bitfield describing capabilities of device requesting pairing */
	OSAL_u8 rsvd[2];				/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE rdv_pr_protocol_entry_basic_t;

/**
 *	@brief rdv_spr_protocol_entry_basic_t : entry for each slot of type RDV SIMPLE PAIRING REPLY
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 28 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rdv_spr_protocol_entry_basic
{
	OSAL_u32 src_hash;			/**< please coment me */
	OSAL_u32 dst_hash;	/**< Previous short ID (aka hash) of UWB slave */
	OSAL_u32 new_dst_hash;	/**< Short ID (aka hash) that UWB slave now use */
	rng_config_e rngDataType;/**< data configuration of the allocated slot */
	OSAL_u8 pairing_answer;	/**< Was a slot allocated successfully */
	OSAL_u8 channel;		/**< channel, 1,2,3,4 or	8,9,10,11 */
	OSAL_u8 zone;			/**< UWB zone that will be used on specified slot (1 to 5) */
	OSAL_u8 gts_type;		/**< Direction of frame, from UWB slave to UWB master or vice-versa */
	OSAL_u8 it_start;		/**< At which superframe number the device should start ranging */
	OSAL_u8 slot_first;		/**< First slot to be allocated in the superframe  (0->127)*/
	OSAL_u8 slot_inc;		/**< increment between each allocated slot in the superframe  (0->127)*/
	OSAL_u8 slot_total;		/**< total slot(s) to be allocated in the superframe  (0->127)*/
	OSAL_u8 gts_rate;		/**< At which rate the device should range (0->127) */
	OSAL_u8 rsvd[3];		/**< Reserved:32 bits alignement */
} OSAL_PACKED_STRUCTURE rdv_spr_protocol_entry_basic_t;

/**
 *	@brief rdv_dreq_protocol_entry_basic : entry for each slot of type RDV DISSOCIATION REQUEST
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 12 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rdv_dreq_protocol_entry_basic
{
	OSAL_u32 src_hash;		/**< Short ID of device emitting the TX.Endianess indicated in rngDataType */
	OSAL_u32 dst_hash;		/**< Short ID of device destination.Endianess indicated in rngDataType */
	OSAL_bool_t reboot;		/**< Should the requested device reboot after detaching */
	OSAL_u8 rsvd[3];		/**< reserved for alignement */
} OSAL_PACKED_STRUCTURE rdv_dreq_protocol_entry_basic_t;


/**
 *	@brief rdv_protocol_entry_basic : entry for each slot of type RDV ACK 
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since its 32 bytes.
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct rdv_protocol_entry_basic
{
	type_rdv_t type;		/**< rdv Type */
	OSAL_u8 rsvd[3];		/**< reserved for alignement */
	union{ //max size is the one of rdv_spr_protocol_entry_basic_t = 28 bytes
		rdv_ack_protocol_entry_basic_t ack;		/**< data in case RDV is an ACK */
		rdv_pr_protocol_entry_basic_t pr; 		/**< data in case RDV is a PAIRING REQUEST */
		rdv_spr_protocol_entry_basic_t spr; 	/**< data in case RDV is a SIMPLE PAIRING REPLY */
		rdv_dreq_protocol_entry_basic_t dreq;	/**< data in case RDV is a DISSOCIATION REQUEST */
	}sub;
} OSAL_PACKED_STRUCTURE rdv_protocol_entry_basic_t;


/**
 *	@brief protocol_entry_basic : entry for each slot of any type
 *	@note This structure MUST BE ALIGNED ON 32 bits, which is the case since the worst case entry is 32 bytes. (rdv_protocol_entry_basic)
 *	@note must be always placed at [frame]_protocol_entry_t->data[0]
 */
typedef struct protocol_entry_basic
{
	union{
		beacon_protocol_entry_basic_t b;	/**< basic info for a beacon unified */
		beacon1_protocol_entry_basic_t b1;	/**< basic info for a beacon 1 */
		beacon2_protocol_entry_basic_t b2;	/**< basic info for a beacon 2 */
		rng_protocol_entry_basic_t rng;		/**< basic info for a ranging*/
		rdv_protocol_entry_basic_t rdv;		/**< basic info for a rdv */
	}basic;
} OSAL_PACKED_STRUCTURE protocol_entry_basic_t;


/**
 *	@brief rng_protocol_entry_t : entry for each slot.
 *	@note This structure is aligned and is max 684 bytes / 384 bytes without iq support
 */
typedef struct rng_protocol_entry
{
	#define RNG_PROTOCOL_ENTRY_COMMON_SIZE        8 /**< the common size is sizeof(rng_config_e)+sizeof(u16)+2xsizeof(u8) = 8*/
	#define RNG_PROTOCOL_ENTRY_BASE_SIZE         (sizeof(protocol_entry_basic_t)) /**< the basic entry for a header of any type*/
	#define RNG_PROTOCOL_ENTRY_MAX_PP_DATASIZE  128 /**< maximum size for UWB data is the size in pp buffer */
	#define RNG_PROTOCOL_ENTRY_UWBDATA_BASE_SIZE  4 /**< complete data pushed in tX buffer will have a prefix of [basic rng info(frame(3)+hash emitter(12)+hash receiver(12) = 27], rounded to 4bytes */
	#define RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_THEORIC (RNG_PROTOCOL_ENTRY_MAX_PP_DATASIZE-RNG_PROTOCOL_ENTRY_UWBDATA_BASE_SIZE) /**< maximum size for UWB data = round(127 bytes -round([basic rng info(frame(3)+hash emitter(12)+hash receiver(12) = 27]=4bytes) */
	#define RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_LIMITED 108 /**< max limit used for whole uwbdata */
	#define RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE ((RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_LIMITED<RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_THEORIC)?RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_LIMITED:RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE_THEORIC) /**< maximum size for UWB data */

	#define RNG_PROTOCOL_ENTRY_MAX_EXTDATA_SIZE (RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE - 1 ) /**< External data will always been prepended by 1 bytes, a counter. Upper layer can then filter and not send data through these index*/
	#ifndef NO_IQ_SUPPORT // cea style
		#define RNG_PROTOCOL_ENTRY_MAX_INFO_SIZE    (sizeof(loc_toa_t)+sizeof(loc_metrics_t)+sizeof(loc_nrj_t)+sizeof(loc_phy_t)+sizeof(loc_estimate_complete_t)) /**< all loc info usable  */
	#else
		#define RNG_PROTOCOL_ENTRY_MAX_INFO_SIZE    (sizeof(loc_toa_t)+sizeof(loc_metrics_t)+sizeof(loc_nrj_t)+sizeof(loc_phy_t)+sizeof(loc_estimate_nrj_t)) /**< all loc info usable  */
	#endif


	//define here the max size we can authorize, until overload. 
	//to opimize memory, we can asssume that worst case is RNG_PROTOCOL_ENTRY_MAX_INFO_SIZE(loc_toa, phy, estimate +  14 u32 of datas)
	//scalable until a certain point then
	#define PROTOCOL_ENTRY_MAX_DATA_SIZE        (RNG_PROTOCOL_ENTRY_BASE_SIZE+RNG_PROTOCOL_ENTRY_MAX_UWBDATA_SIZE+RNG_PROTOCOL_ENTRY_MAX_INFO_SIZE)
	#define PROTOCOL_ENTRY_MAX_SIZE             (sizeof(rng_protocol_entry_t))

	rng_config_e rngDataType;	/**< Type of ranging data, used to interpret next data, on 4bytes.This filed is always in BIG ENDIAN*/
	OSAL_u16 status;		/**< status */
	OSAL_u8 slot_index;		/**< This ranging slot index */
	OSAL_u8 link_quality;	/**< link quality */
	OSAL_u8	data[PROTOCOL_ENTRY_MAX_DATA_SIZE]; /**< 	Ranging Data up to 128bytes, LOC info up to RNG_PROTOCOL_ENTRY_MAX_INFO_SIZE
												This data always begin with basic entry which can be rng_protocol_entry_basic,beacon1_protocol_entry_basic,beacon2_protocol_entry_basic,rdv_protocol_entry_basic
												There is always a padidng between Ranigng Data, and LOC data,
												in order to let LOC data begin at multiple of 32bits
												Exemple :  if ranging data is 1023 bits, loc data will begin at data[128]
												* If ranging data is 1015 bits, loc data will begin at data[128]
												* If ranging data is 546 bits, loc data will being at data[72]
												* More Generally, loc[index], with index = (DATA sizeinbits%32==0)?DATA sizeinbits/8:((DATA sizeinbits/32)+1)*4
												*/
	
} OSAL_PACKED_STRUCTURE rng_protocol_entry_t;

/**
 *  @brief average_rng_measure_t : - groups metrics for average TOA, average min and max nrj
 *  @note This structure is aligned and is 20 bytes
 */
typedef struct average_rng_measure
{
	OSAL_u32 rangingNB;	/**<total number of ranging */
	OSAL_s32 averageTOA;	/**<average toa got */
	OSAL_s32 averageNRJ;	/**<average fg1 NRJ */
	OSAL_s32 minNRJ;	/**<min of fg1 NRJ got */
	OSAL_s32 maxNRJ;	/**<max of fg1 NRJ got */
} OSAL_PACKED_STRUCTURE average_rng_measure_t;

/**
 *	@brief frame_info : - Describe a single slot in a superframe schema
 *	@note This structure is aligned and sized 16 bytes
 */
typedef struct frame_info {
	OSAL_u8	slot_index;		/**< Slot Index, 0 to PSN. LSB is one PSS*/
	frame_info_type_t type;		/**< Type of slot */
	action_t act;			/**< Type of action */
	OSAL_u8	beacon_id;		/**< BeaconId (only used if type is BEACON1 or BEACON2 )*/
	OSAL_u8 zone;			/**< From 1 to 5 (inclusive) */
	OSAL_u8 channel;		/**< ETSI Channel: 1, 2, 3, 4	or 8, 9, 10, 11 */
	OSAL_u8 rf_path;		/**< Select different rf path (eg with multiple antennas) */
	OSAL_u8 rsvd1[1];		/**< 32 bits alignement */
	rng_config_e rngDataType;	/**< Type of ranging data, used to interpret next data, on 4bytes*/
} OSAL_PACKED_STRUCTURE frame_info_t;

/**
 *	@brief superframe_info : to store information concerning the superframe
 *	@note This structure is aligned and sized 532 bytes
 */
typedef struct superframe_info
{
	OSAL_u32 phs;			/**< Protocol Hyperframe Size : number of superframe per Hyperframe */
	OSAL_u32 psn;			/**< Protocol Slot Number: Numbers of slot in a superframe */
	OSAL_u32 pss;			/**< Protocol Slot Size: Size of a slot, in TS */
	OSAL_u16 nb_frames;		/**< Number of slots described in this frames_desc */	
	OSAL_u8	beacon_distance;	/**< Delay between a BEACON1 and a BEACON2, in PSS */
	OSAL_u8  sync_code_ID;		/**< Spreading code ID for synchro */
	OSAL_u8  demod_code_ID;		/**< Spreading code ID for demod */
	OSAL_u8  rsvd[3];		/**< Reserved:32 bits alignement */      
	frame_info_t frames_desc[MAX_FRAME_PER_SUPERFRAME]; /**<slot description */
} OSAL_PACKED_STRUCTURE superframe_info_t;

/**
 *	@brief superframe_info_summary_t : a structure with summary for superfrmae info
 *	@note This structure is aligned and sized 52 bytes.
 *  @note for internal purpose, phs,psn,pss, beacon_distance, must be at the same place than same field in superframe_info_t
 *  @note All fields above sync_code_ID are mapped like defined in userSectionStruct default_sfi
 */
typedef struct superframe_info_summary
{
	OSAL_u32 phs;							/**< Protocol Hyperframe Size : number of superframe per Hyperframe */
	OSAL_u32 psn;							/**< Protocol Slot Number: Numbers of slot in a superframe */
	OSAL_u32 pss;							/**< Protocol Slot Size: Size of a slot, in TS */
	OSAL_u16 rsvd1[1];						/**< alignement on superframe_info_t first field */
	OSAL_u8	nbRelay;						/**< Delay between a BEACON1 and a BEACON2, in PSS */
	OSAL_u8 zoneBeacon;						/**< default zone used on Beacon slots*/
	OSAL_u8 zoneRanging;					/**< default zone used on Ranging slots*/
	OSAL_u8 zoneRdv;						/**< default zone used on Rdv slots*/
	OSAL_u8 rsvd2[2];						/**< alignement on superframe_info_t first field */
	OSAL_u16 firstRangingSlot;				/**< default first ranging slot*/
	OSAL_s16 rng_slot_inc;					/**< ranging slot increment. Negative if unused (protocol will adapt) */
	OSAL_s16 rng_slot_total;				/**< ranging slot total. Negative if unused (protocol will adapt)*/
	OSAL_s16 rdvSlot;						/**< default rdv slot. Negative if unused*/
	rng_config_e b1_rngDataType;			/**< rngDataType used by beacon1 */
	rng_config_e b2_rngDataType;			/**< rngDataType used by beacon2 */
	rng_config_e rng_rngDataType;			/**< rngDataType used by ranging */
	rng_config_e rdv_rngDataType;			/**< rngDataType used by rdv */
	OSAL_u8 sync_code_ID;				/**< Spreading code ID for synchro */
	OSAL_u8 demod_code_ID;				/**< Spreading code ID for demod */
	OSAL_u8 channel;				/**< Channel used for each slot of sfi */
	OSAL_u8 rsvd3;					/**< 32 bits alignement/reserved */
} OSAL_PACKED_STRUCTURE superframe_info_summary_t;

/**
 *	@brief dev_info : contains information of an UWB device
 *	@note This structure is aligned and sized 32 bytes
 */
typedef struct dev_info
{
	OSAL_u8		address[UWB_MAC_LEN];	/**< UWB Mac Address of the device, aka long id */	
	OSAL_u32 	hash_address;		/**< Hash of mac address, aka short id*/	
	dev_status_t	status;			/**< Device status */
	OSAL_u8		movement;		/**< Tell if the device is moving */
	OSAL_u8		level_bat;		/**< Tell the battery level*/
	OSAL_u8		loc_rate;		/**< LSR rate type	*/
	OSAL_u8		hwVersion;		/**< Device hardware version */
	OSAL_u8		swVersion;		/**< Device software version */
	OSAL_u8		sensor;			/**< Sensor value (unused for the moment)*/	
	OSAL_u8		rsvd;			/**< Reserved:32 bits alignement */
	OSAL_u32	secureKey;		/**< Device secure key (unused for the moment)*/
	OSAL_u32	MacCtrl1;		/**< SF position. */
    //bit            |31| ... |24|23| ... |16|15| ... | 8| 7| ... |0|
    //MacCtrl2       |slot_first | slot_total| slot_inc  | reserved |
    #define MACCTRL2_SFIRST_SHIFT 24
    #define MACCTRL2_SFIRST_MASK  (((uint32_t) 0xFF)<<MACCTRL2_SFIRST_SHIFT)
    #define MACCTRL2_STOTAL_SHIFT 16
    #define MACCTRL2_STOTAL_MASK  (((uint32_t) 0xFF)<<MACCTRL2_STOTAL_SHIFT)
    #define MACCTRL2_SINC_SHIFT   8
    #define MACCTRL2_SINC_MASK    (((uint32_t) 0xFF)<<MACCTRL2_SINC_SHIFT)
	OSAL_u32	MacCtrl2;		/**< Slots organisation: 8bits(31->24):rsvd, 8bits(23->16):slot_firstposition, (15->8):slot_total, 8bits(7->0): slot_increment. */
} OSAL_PACKED_STRUCTURE artls_dev_info_update_t;

/**
 *	@brief artls_pairing_request : contains pairing request
 *	Pairing request are sent from a UWB Slave device to a UWB Master
 *	@note This structure is aligned and sized 16 bytes
 */
typedef struct artls_pairing_request {
	OSAL_u8 tag_mac[UWB_MAC_LEN];	/**< UWB Mac address of the device that request pairing */
	OSAL_u32 tag_hash;				/**< Short ID (aka hash) of the device requesting pairing */
	OSAL_u8 tag_profile;			/**< Bitfield describing capabilities of device requesting pairing */
	OSAL_u8 rsvd[3];				/**< Reserved:32 bits alignement */
} OSAL_PACKED_STRUCTURE artls_pairing_request_t;


/**
 *	@brief artls_pairing_request : contains pairing reply
 *	Pairing reply are sent from a UWB Master device to a UWB Slave
 *	@note This structure is aligned and sized 24 bytes
 */
typedef struct artls_simple_pairing_reply {
	OSAL_u32 old_dst_hash;	/**< Previous short ID (aka hash) of UWB slave */
	OSAL_u32 new_dst_hash;	/**< Short ID (aka hash) that UWB slave now use */
	rng_config_e rngDataType;/**< data configuration of the allocated slot */
	OSAL_u8 pairing_answer;	/**< Was a slot allocated successfully */
	OSAL_u8 channel;		/**< channel, 1,2,3,4 or	8,9,10,11 */
	OSAL_u8 zone;			/**< UWB zone that will be used on specified slot (1 to 5) */
	OSAL_u8 gts_type;		/**< Direction of frame, from UWB slave to UWB master or vice-versa */
	OSAL_u8 it_start;		/**< At which superframe number the device should start ranging */
	OSAL_u8 slot_first;		/**< First slot to be allocated in the superframe  (0->127)*/
	OSAL_u8 slot_inc;		/**< increment between each allocated slot in the superframe  (0->127)*/
	OSAL_u8 slot_total;		/**< total slot(s) to be allocated in the superframe  (0->127)*/
	OSAL_u8 gts_rate;		/**< At which rate the device should range (0->127) */
	OSAL_u8 rsvd[3];		/**< Reserved:32 bits alignement */
} OSAL_PACKED_STRUCTURE artls_simple_pairing_reply_t;

/**
 *	@brief artls_dissociation_request: contains dissociation request
 *	Dissocatiation request are sent from a UWB Master device to a UWB Slave
 *	@note This structure is aligned and sized 8 bytes
 */
typedef struct artls_dissociation_request {
	OSAL_u32 dst_hash;	/**< Short ID (aka hash) of device that should dissociate */
	OSAL_bool_t reboot;	/**< Should the requested device reboot after detaching */
	OSAL_u8 rsvd[3];	/**< Reserved:32 bits alignement */      
} OSAL_PACKED_STRUCTURE artls_dissociation_request_t;

/**
 *	@brief artls_pairing_request : contains pairing reply
 *	@note This structure is unaligned and sized 5 bytes 
 */
typedef struct artls_cmd_status {
	OSAL_u32 user_data;				/**< Internal defined data that will be sent back when command will be finished */
	artls_cmd_return_code_t status;	/**< status of sent ARTLS command */
} OSAL_PACKED_STRUCTURE artls_cmd_status_t;

/**
 *	@brief artls_up : contains artls notification - send from driver to userspace
*	@note This structure is UNALIGNED and sized 37 bytes.
 */
typedef struct artls_up
{
	OSAL_u8			reserved[4];	 	/**< Reserved data. Usefull for ack in case of Ext_com communication */
	artls_up_cmd_e		notif_type;	 	/**< signal which fields are updated */
	union {
		OSAL_u32		local_ref_base;	/**< Reference base of the local pinpointer */
		artls_lostatus_t	local_status;	/**< Ranging status of the local pinpointer */
		artls_pairing_request_t pairing_req;	/**< Pairing request from a remote peer */
		artls_dev_info_update_t	device_info;	/**< Got Information of a remote peer */
		artls_dissociation_request_t dreq;	/**< Got a dissoctiation request from an UWB Master */
		artls_cmd_status_t	cmd_status;	/**< A command artls_down_t has finished */
	} payload;
} OSAL_PACKED_STRUCTURE artls_up_t;

/**
 *	@brief artls_down : contains artls notification - send from userspace to driver
*	@note This structure is aligned and sized 36 bytes.
 */
typedef struct artls_down
{
	artls_down_cmd_e cmd_type;			/**< type of artls (could be command defined data that will be sent back when command will be finished*/
	OSAL_u8 cmd_tracking;				/**< Define how the command status should be tracked*/
	OSAL_u8 rsvd[2];				/**< Reserved:32 bits alignement */
	OSAL_u32 user_data;				/**< User defined data that will be sent back when command will be finished*/
	OSAL_u32 sf_start;				/**< Superframe number at which the artls command must be run */
	OSAL_u32 sf_period;				/**< If command failed, repeat the command every sf_period */
	union {
		artls_simple_pairing_reply_t spr;	/**< used if cmd_type == ARTLS_SIMPLE_PAIRING_REPLY_DOWN*/
		artls_dissociation_request_t dreq;	/**< used if cmd_type == ARTLS_DISSOCIATION_REQUEST_DOWN */
		OSAL_u16 extDataLenInBits;		/**< used if cmd_type == ARTLS_SET_EXT_DATA_SIZE*/
		OSAL_u32 dev_id;			/**< used if cmd_type == ARTLS_SET_DEV_ID_DOWN*/
		OSAL_u32 prefered_ref_base;		/**< used if cmd_type == ARTLS_SET_PREFERED_REF_BASE_DOWN*/
		#define RFPATH_NB 4
		OSAL_s16 antenna_offset[RFPATH_NB];	/**< used if cmd_type == ARTLS_SET_ANTENNA_OFFSET */
		OSAL_u16 threshold_coeff;		/**< used if cmd_type == ARTLS_SET_THRESHOLD_COEFF */
		OSAL_u8 rsvd[2];			/**< Reserved:32 bits alignement */
	} payload;
} OSAL_PACKED_STRUCTURE artls_down_t;

#endif //PINPOINTER_PROTOCOL_DEF_H

