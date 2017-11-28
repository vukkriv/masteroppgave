/**
 * @file pinpointer_common_def.h
 * @brief enum and structure definition used by UWB stack
 * @author www.bespoon.com
 * @date Bespoon 2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */
 
/*FIXME : merge pm_types with pinpointer_drv_api.h for PM handling via kernel*/
//need to be aligned with /env/include/pm_str.h

 
#include <osal_type.h>

#ifndef PINPOINTER_COMMON_DEF_H
#define PINPOINTER_COMMON_DEF_H

//#################################
// COMMON DEFINE
//#################################

/**
* @brief UWB_MAC_LEN length of UWB MAC Address, in bytes
*/
#define UWB_MAC_LEN	8

/**
* @brief SHORT_ADDR_LEN length of a UWB Short address (a.k.a hash a.k.a short ID) , in bits
*/
#define SHORT_ADDR_LEN 12


/**
* @brief EST_SIZE Size of estimation
*/
#define EST_SIZE	64

/**
* @brief MAX_CHANNEL_ESM Size of estimation
*/
#define MAX_CHANNEL_ESM			EST_SIZE

//#################################
// COMMON MACRO
//#################################

/**
* @brief SWITCH_ENDIANESS_64BITS Change a 64bits value endiannes (FROM be->le OR le->be)
*/
#define SWITCH_ENDIANESS_64BITS(_u64) (\
					 ((((OSAL_u64) _u64)<<56)&0xFF00000000000000)| \
					 ((((OSAL_u64) _u64)<<40)&0x00FF000000000000)| \
					 ((((OSAL_u64) _u64)<<24)&0x0000FF0000000000)| \
					 ((((OSAL_u64) _u64)<<8 )&0x000000FF00000000)| \
					 ((((OSAL_u64) _u64)>>8 )&0x00000000FF000000)| \
					 ((((OSAL_u64) _u64)>>24)&0x0000000000FF0000)| \
					 ((((OSAL_u64) _u64)>>40)&0x000000000000FF00)| \
					 ((((OSAL_u64) _u64)>>56)&0x00000000000000FF)\
				)

/**
* @brief SWITCH_ENDIANESS_32BITS Change a 32bits value endiannes (FROM be->le OR le->be)
*/
#define SWITCH_ENDIANESS_32BITS(_u32) (\
					 ((((OSAL_u32) _u32)<<24)&0xFF000000)| \
					 ((((OSAL_u32) _u32)<<8 )&0x00FF0000)| \
					 ((((OSAL_u32) _u32)>>8 )&0x0000FF00)| \
					 ((((OSAL_u32) _u32)>>24)&0x000000FF)\
					)

/**
* @brief SWITCH_ENDIANESS_16BITS Change a 16bits value endiannes (FROM be->le OR le->be)
*/
#define SWITCH_ENDIANESS_16BITS(_u16) (\
					 ((((OSAL_u16) _u16)<<8)&0xFF00)| \
					 ((((OSAL_u16) _u16)>>8)&0x00FF)\
					)

/**
* @brief NO_SWITCH_ENDIANESS Identity Macro
*/
#define NO_SWITCH_ENDIANESS(_u16) (_u16)
//#################################
// COMMON ENUMS
//#################################

/** 
 * @brief action_t action type enumeration
*/
typedef enum {
	TX=0,			/**< Action is TX.*/
	RX,			/**< Action is RX.*/
	WAIT,			/**< Action is WAIT.*/
	NO_ACTION,		/**< Action is None.*/
} OSAL_PACKED_ENUM action_t;

/** 
 *	@brief pm_state_t Enumeration of available Power Managment states
*/
typedef enum {
	PM_3=0,		/**< Power management state 3.*/
	PM_2,		/**< Power management state 2.*/
	PM_0,		/**< Power management state 0.*/ 
	PM_STDBY,	/**< Power management standby.*/
	PM_RX_IDLE,	/**< Power management Rx idle.*/
	PM_TX_IDLE,	/**< Power management TX_IDLE.*/
	PM_TXRX_IDLE,	/**< Power management TXRX_IDLE.*/
	POWER_MODE_NB,	/**< Power management number of mode */
} OSAL_PACKED_ENUM pm_state_t;

/**
 * @brief description of RF paths
 *  12 means : internal path1  and external path2 
*/
 typedef enum rf_path_rx
 {
	RF_PATH_NONE	 = 0,   /**< No RF Path set (no rx or tx configured) */
 	RF_PATH1_1       = 11,	/**< Internal path 1, external path 1 */
 	RF_PATH1_2       = 12,  /**< Internal path 1, external path 2 */
 	RF_PATH2_1       = 21,  /**< Internal path 2, external path 1 */
 	RF_PATH2_2       = 22	/**< Internal path 2, external path 2 */
 } OSAL_PACKED_ENUM rf_path_rx_t;


//#################################
// COMMON STRUCTURES
//#################################
/**
 *	@brief config_phy_t : contains all radio params that can be get/set by userspace
 *	@note This structure is aligned and sized 28 bytes
 */
typedef struct _config_phy
{
	OSAL_u8 reserved[1];		/**< Reserved data. Usefull for ack in case of Ext_com communication */
	OSAL_u8 manual_threshold;	/**< If 1, sync_threshold and rng threshold must be set manually. If 0, filed will be ignored ( automaticaaly calculated*/
	OSAL_u8 sync_code_ID;		/**< Spreading code to use for synchro */
	OSAL_u8 demod_code_ID;		/**< Spreading code to use for demod  */
	OSAL_u8 channel;		/**< Channel definition:Low band : channels 1, 2/4 and 3, High band: channels 8, 9/11 */
	OSAL_u8 sync_code;		/**< Code length for preamble + SFD:7,15,31,63,127*/
	OSAL_u8 demod_code;		/**< Code length for PHR + PSDU:7,15,31,63,127*/
	OSAL_u8 sync_rep;		/**< Symbol length for preambule + SFD: 1,2,4,8,16,32,64 */
	OSAL_u8 demo_rep;		/**< Symbol length for PSDU: 1,2,4,8,16,32,64*/
	OSAL_u8 clock_track;		/**< Enable / Disable the clock tracking functionality. Contains cktrack settings (1st bit) and precomp settings (2nd bit) */
	OSAL_u8 framer_config;		/**< Frame config: GOLAY CODEC ON/OFF, PHR CRC ON/OFF,PSDU CRC ON/OFF,PSDU CRC type,FRAMING_STYLE */
	rf_path_rx_t rf_path;		/**< RF Path, used to select different antennas for example */
	OSAL_u32 sync_threshold;	/**< Thresholds used for preamble detection.0 mean automatic */
	OSAL_u32 rng_threshold;		/**< Thresholds used for leading edge detection.0 mean automatic */
	OSAL_u8 tx_gain;		/**< Extra tx gain (unused for the moment:for Future use = PA extern?) */
	OSAL_u8 rx_gain;		/**< Extra rx gain (unused for the moment:for Future use = LNA variable?) */
	pm_state_t power_mode;		/**< Current power management state */
	action_t act;			/**< Type of action */
	OSAL_u32 preamble_timeout;	/**< Time-out for preamble detection, LSB is 16ns */
} OSAL_PACKED_STRUCTURE config_phy_t;


/**
 *	@brief rng_data_generic_t : - groups both TOAs, Rxqids and estimates values for a ranging
 *	@note This structure is aligned and is 448 bytes / 8 bytes w/o CONFIG_RTLS_FULL_LOC
 */
typedef struct rng_data_generic
{
	OSAL_u8 rsvd0[12];		/**<compatibility with SDK<=1.4 */
	OSAL_u32 toa_le_raw;		/**< Time of arrival( nano) */
	OSAL_s32 toa_le_corr;		/**< Time of arrival correction in picosecond ( -5000 ps <=x<=5000ps, real toa = toa_le_raw + toa_le_corr ) */

	OSAL_u32 toa_fg1;		/**< Finger 1 toa */
	OSAL_u32 toa_fg2;		/**< Finger 2 toa */
	OSAL_u32 toa_fg3;		/**< Finger 3 toa */
	OSAL_u32 toa_fg4;		/**< Finger 4 toa */

	OSAL_u8 rsvd2[8];		/**<compatibility with SDK<=1.4 */
	OSAL_s16 clock_drift;		/**< Clock drift in pbb */
	OSAL_u8 link_quality;		/**< Link quality in pcent*/
	OSAL_u8 rsvd3[1];		/**<compatibility with SDK<=1.4 */

	OSAL_u8 iwle;			/**< IW Time of arrival */
	OSAL_u8 iwfat;			/**< IW First Above Thresold */
	OSAL_u8 rsvd4[2];		/**<compatibility with SDK<=1.4 */
	OSAL_u8 iwfg1;			/**< IW Finger 1 toa	*/
	OSAL_u8 iwfg2;			/**< IW Finger 2 toa	*/
	OSAL_u8 iwfg3;			/**< IW Finger 3 toa	*/
	OSAL_u8 iwfg4;			/**< IW Finger 4 toa	*/
	
	OSAL_u32 rxqid_le_raw;		/**< RSSID Time of arrival */
	OSAL_u8 rsvd5[4];		/**<compatibility with SDK<=1.4 */
	OSAL_u32 rxqid_fg1;		/**< RSSID Finger 1 toa	*/
	OSAL_u32 rxqid_fg2;		/**< RSSID Finger 2 toa	*/
	OSAL_u32 rxqid_fg3;		/**< RSSID Finger 3 toa	*/
	OSAL_u32 rxqid_fg4;		/**< RSSID Finger 4 toa	*/
	OSAL_u8	prp[EST_SIZE];		/**< Channel estimation memory, PRP index*/
	OSAL_u32 nrj[EST_SIZE];		/**< Channel estimation memory, cumulated energy values */
	OSAL_u8 startPhyConf;		/**< Start of phy config part in this structure */
	OSAL_u8 rsvd6[3];		/**< compatibility with SDK<=1.4 */
	OSAL_u8 channel;		/**< Channel definition:Low band : channels 1, 2/4 and 3, High band: channels 8, 9/11 */
	OSAL_u8 sync_code;		/**< Code length for preamble + SFD:7,15,31,63,127*/
	OSAL_u8 demod_code;		/**< Code length for PHR + PSDU:7,15,31,63,127*/
	OSAL_u8 sync_rep;		/**< Symbol length for preambule + SFD: 1,2,4,8,16,32,64 */
	OSAL_u8 demo_rep;		/**< Symbol length for PSDU: 1,2,4,8,16,32,64*/
	OSAL_u8 clock_track;		/**< Enable / Disable the clock tracking functionality. Contains cktrack settings (1st bit) and precomp settings (2nd bit) */
	OSAL_u8 framer_config;		/**< Frame config: GOLAY CODEC ON/OFF, PHR CRC ON/OFF,PSDU CRC ON/OFF,PSDU CRC type,FRAMING_STYLE */
	OSAL_u8 rsvd7[1];		/**<compatibility with SDK<=1.4 */
	OSAL_u32 sync_threshold;	/**< Thresholds used for preamble detection.0 mean automatic */
	OSAL_u32 rng_threshold;		/**< Thresholds used for leading edge detection.0 mean automatic */
	OSAL_u8 rsvd8[2];		/**<compatibility with SDK<=1.4 */
	OSAL_u8 powerMode;		/**< phyConfigPower mode*/
	OSAL_u8 rsvd9[1];		/**<compatibility with SDK<=1.4 */
	OSAL_u32 preamble_timeout;	/**< Time-out for preamble detection, LSB is 16ns */
	OSAL_u64 nrj_sum;		/**< Sum of values of estimate.nrj */
	OSAL_u32 med;			/**< Mean Excess Delay of energy */
	OSAL_u32 rms;			/**< RMS Delta Spread of energy */
	OSAL_u8 rsvd10[4];		/**<compatibility with SDK<=1.4 */
} OSAL_PACKED_STRUCTURE rng_data_generic_t;


/**
 *	@brief reduced_rng_data_generic_t : - groups both TOAs, and clock drift for a ranging
 *	@note This structure is aligned and is 32 bytes
 */
typedef struct reduced_rng_data_generic
{
	OSAL_u8 rsvd0[12];		/**<compatibility with SDK<=1.4 */
	OSAL_u32 toa_le_raw;		/**< Time of arrival( in nano) */
	OSAL_s32 toa_le_corr;		/**< Time of arrival correction in picosecond ( -5000 ps <=x<=5000ps, real toa = toa_le_raw + toa_le_corr ) */
	OSAL_u8 rsvd2[8];		/**<compatibility with SDK<=1.4 */
	OSAL_s16 clock_drift;		/**< Clock drift in pbb */
	OSAL_u8 link_quality;		/**< Link quality in pcent*/
	OSAL_u8 rsvd3[1];		/**<compatibility with SDK<=1.4 */
} OSAL_PACKED_STRUCTURE reduced_rng_data_generic_t;

/**
 *	@brief loc_toa_t : contains toa in iw and ntoa
 *	@note This structure is aligned and sized 16 bytes
 */ 
typedef struct relative_toa
{
	OSAL_s32 iw;   /**<  in IW (~1ns) */
	OSAL_s32 ntoa; /**<  in NTOA (~125ps) */
}OSAL_PACKED_STRUCTURE rtoa_t;

/**
 *	@brief loc_toa_t : contains toa le
 *	@note This structure is aligned and sized 8 bytes
 */ 
typedef struct loc_toa
{
	OSAL_u32 toa_le;	/**< Time of arrival( nano) */
	OSAL_u8 para_size;	/**< Parabola size used for MED: bigger is better */
	OSAL_u8 los_indicator; 	/**< LOS indicator: bigger is better */
	OSAL_u8	rsvd3[2];		/**< 32 bits alignement	*/
} OSAL_PACKED_STRUCTURE loc_toa_t;

/**
 *	@brief loc_metrics_t : contains nrj summary for channel
 *	@note This structure is aligned and sized 20 bytes.
 */ 
typedef struct loc_metrics
{
	OSAL_u64 nrj_sum;		/**< Sum of values of estimate.nrj */
	OSAL_u32 med;			/**< Mean Excess Delay of energy */
	OSAL_u32 rms;			/**< RMS Delta Spread of energy*/	
	OSAL_u8	nb_spl;			/**< Number of valid sample to compute the nrj sum */			
	OSAL_u8	rsvd3[3];		/**< 32 bits alignement	*/
} OSAL_PACKED_STRUCTURE loc_metrics_t;

/**
 *	@brief loc_nrj_t : contains finger information
 *	@note This structure is aligned and sized 32 bytes.
 */ 
typedef struct loc_nrj
{
	OSAL_u32 toa_fg1;		/**< Finger 1 toa */
	OSAL_u32 toa_fg2;		/**< Finger 2 toa */
	OSAL_u32 toa_fg3;		/**< Finger 3 toa */
	OSAL_u32 toa_fg4;		/**< Finger 4 toa */
	OSAL_u32 rxqid_fg1;		/**< RSSID Finger 1 toa	*/
	OSAL_u32 rxqid_fg2;		/**< RSSID Finger 2 toa	*/
	OSAL_u32 rxqid_fg3;		/**< RSSID Finger 3 toa	*/
	OSAL_u32 rxqid_fg4;		/**< RSSID Finger 4 toa	*/
} OSAL_PACKED_STRUCTURE loc_nrj_t;

/**
 *	@brief loc_phy_t : contains most important value for phy config retruned trough ranging
 *	@note This structure is aligned and sized 12 bytes
 */ 
typedef struct loc_phy
{
	OSAL_u32 sync_threshold;	/**< Thresholds used for preamble detection.0 mean automatic */
	OSAL_u32 rng_threshold;		/**< Thresholds used for leading edge detection.0 mean automatic */
	OSAL_u8	channel;		/**< Channel definition for this ranging:Low band : channels 1, 2/4 and 3, High band: channels 8, 9/11 */
	OSAL_u8	zone;			/**< Zone definition for this ranging */
	OSAL_u8	rsvd[2];		/**< 32bits alignement */
} OSAL_PACKED_STRUCTURE loc_phy_t;

/**
 *	@brief loc_estimate_nrj_t : groups all info needed to reconstruct pulse, through nrj only
 *	@note This structure is aligned and is 328
 */
typedef struct loc_estimate_nrj
{
	OSAL_u8	iwle;			/**< IW Time of arrival */
	OSAL_u8	iwfg1;			/**< IW Finger 1 toa	*/
	OSAL_u8	iwfg2;			/**< IW Finger 2 toa	*/
	OSAL_u8	iwfg3;			/**< IW Finger 3 toa	*/
	OSAL_u8	iwfg4;			/**< IW Finger 4 toa	*/
	OSAL_u8 iwfat;			/**< IW First Above Thresold */
	OSAL_u8 fine_fat;		/**< Offset in NTOA between iwfat and refined FAT */
	OSAL_u8 fine_paramed;		/**< Offset in NTOA between iwfat and PARAMED */
	OSAL_u8	prp[EST_SIZE];		/**< Channel estimation memory, PRP index 64*/
	OSAL_u32 nrj[EST_SIZE];		/**< Channel estimation memory, cumulated energy values256*/
} OSAL_PACKED_STRUCTURE loc_estimate_nrj_t;

/**
 *	@brief loc_estimate_iq_t : groups all info needed to reconstruct pulse, through i and Q only
 *	@note This structure is aligned and is 328.
 */
typedef struct loc_estimate_iq
{
	OSAL_u8	iwle;			/**< IW Time of arrival */
	OSAL_u8	iwfg1;			/**< IW Finger 1 toa	*/
	OSAL_u8	iwfg2;			/**< IW Finger 2 toa	*/
	OSAL_u8	iwfg3;			/**< IW Finger 3 toa	*/
	OSAL_u8	iwfg4;			/**< IW Finger 4 toa	*/
	OSAL_u8 iwfat;			/**< IW First Above Thresold */
	OSAL_u8 fine_fat;		/**< Offset in NTOA between iwfat and refined FAT */
	OSAL_u8 fine_paramed;		/**< Offset in NTOA between iwfat and PARAMED */
	OSAL_u8	prp[EST_SIZE];		/**< Channel estimation memory, PRP index 64*/
	OSAL_s16 I[EST_SIZE];		/**< Channel estimation memory, I 64 signed values*/
	OSAL_s16 Q[EST_SIZE];		/**< Channel estimation memory, Q 64 signed values*/
} OSAL_PACKED_STRUCTURE loc_estimate_iq_t;


/**
 *	@brief loc_estimate_complete_t : groups all info needed to reconstruct pulse
 *	@note This structure is aligned and is 628 bytes  cannot be more than 658bytes. This structure first member must match loc_estimate_nrj structure
 */
typedef struct loc_estimate_complete
{
	OSAL_u8	iwle;			/**< IW Time of arrival */
	OSAL_u8	iwfg1;			/**< IW Finger 1 toa	*/
	OSAL_u8	iwfg2;			/**< IW Finger 2 toa	*/
	OSAL_u8	iwfg3;			/**< IW Finger 3 toa	*/
	OSAL_u8	iwfg4;			/**< IW Finger 4 toa	*/
	OSAL_u8 iwfat;			/**< IW First Above Thresold */
	OSAL_u8 fine_fat;		/**< Offset in NTOA between iwfat and refined FAT */
	OSAL_u8 fine_paramed;		/**< Offset in NTOA between iwfat and PARAMED */
	OSAL_u8	prp[EST_SIZE];		/**< Channel estimation memory, PRP index 64*/
	OSAL_u32 nrj[EST_SIZE];		/**< Channel estimation memory, cumulated energy values*/
	OSAL_s16 I[EST_SIZE];		/**< Channel estimation memory, I 64 signed values*/
	OSAL_s16 Q[EST_SIZE];		/**< Channel estimation memory, Q 64 signed values*/
	OSAL_s32 dll_rng_nrj_m4dll;	/**< DLL Ranging energy result 4 DLL steps before current position */
	OSAL_s32 dll_rng_nrj_p4dll;	/**< DLL Ranging energy result 4 DLL steps after current position */
	OSAL_s32 dll_rng_nrj_p0dll;	/**< DLL Ranging energy result for current position */
	OSAL_s16 arctan_d_ii;		/**< Arctan ranging correlation result for II */
	OSAL_s16 arctan_d_iq;		/**< Arctan ranging correlation result for IQ */
	OSAL_s16 arctan_d_qi;		/**< Arctan ranging correlation result for QI */
	OSAL_s16 arctan_d_qq;		/**< Arctan ranging correlation result for QQ */
	OSAL_u8 arctan_status;		/**< Arctan associated data enable */	
	OSAL_u8 rsvd[3];			/**< 32bits alignement */
	OSAL_s32 ep_align;				/**< EP */
	OSAL_u32 ts_offset;			/**< ts_offset */
	OSAL_u32 ntoa_corr;			/**< ntoa correction: time in ntoa */
	OSAL_u32 hw_offset;			/**< hw_offset */
	OSAL_u32 padding;		/**< 2 more u32 for future informations*/
} OSAL_PACKED_STRUCTURE loc_estimate_complete_t;

#endif //PINPOINTER_COMMON_DEF_H



#ifdef CONFIG_RTLS_PROTOCOL
#include "pinpointer_protocol_def.h"
#endif

