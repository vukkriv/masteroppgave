/**
 * @file default_sfi.h
 * @brief Default superframe schema for normal rate (e.g. superframe at 250 ms)
 * @author cca@bespoon.com
 * @date 17/09/2013
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#include <pinpointer_drv_api.h>
#ifndef DEFAULT_SFI_H
#define DEFAULT_SFI_H

#define DEFAULT_SFI_PHS 6                                                                                         
#define DEFAULT_SFI_PSN 32                                                                                         
#define DEFAULT_SFI_PSS 256                                                                                        

#define DEFAULT_SFI_SLOT_B1 5                                                                                      
#define DEFAULT_SFI_SLOT_B2 11                                                                                     
#define DEFAULT_SFI_SLOT_RANGING 13 //not really used anyway      
#define DEFAULT_SFI_ZONE_BEACON 4
#define DEFAULT_SFI_ZONE_RANGING 4
#define DEFAULT_SFI_ZONE_RDV 3

#define DEFAULT_SFI_CHANNEL 2
#define DEFAULT_SFI_RX_RFPATH RF_PATH2_1 
#define DEFAULT_SFI_TX_RFPATH RF_PATH1_1 


#ifdef CONFIG_RTLS_FULL_LOC
static frame_info_t Coord_Slot_Desc[] = {
	{ .type = FIT_BEACON1,	.act = TX,		.slot_index = 5, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = TX,		.slot_index = 11,	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 13, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 14, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 15, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 16, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 17, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 18, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 19, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 20, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 21, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 22, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 23, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 24, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 25, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 26, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 27, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 28, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 29, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 30, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RDV,	.act = NO_ACTION,	.slot_index = 31, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};
static frame_info_t Base_Slot_Desc[] = {
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 0, 	.beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 1, 	.beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 2, 	.beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 3, 	.beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 4, 	.beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 5, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 6, 	.beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 7, 	.beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 8, 	.beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 9, 	.beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 10, 	.beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 11, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 13, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 14, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 15, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 16, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 17, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 18, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 19, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 20, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 21, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 22, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 23, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 24, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 25, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 26, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 27, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 28, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 29, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = RX,		.slot_index = 30, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RDV,	.act = NO_ACTION,	.slot_index = 31, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};

static frame_info_t Tag_Slot_Desc[] = {
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 0, 	.beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 1, 	.beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 2, 	.beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 3, 	.beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 4, 	.beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON1,	.act = RX,		.slot_index = 5, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 6, 	.beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 7, 	.beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 8, 	.beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 9, 	.beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 10, 	.beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_BEACON2,	.act = RX,		.slot_index = 11, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 13, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 14, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 15, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 16, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 17, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 18, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 19, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 20, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 21, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 22, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 23, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 24, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 25, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 26, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 27, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 28, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 29, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RANGING,	.act = TX,		.slot_index = 30, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
	{ .type = FIT_RDV,	.act = NO_ACTION,	.slot_index = 31, 	.beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};

#endif // defined CONFIG_RTLS_FULL_LOC
#endif // defined DEFAULT_SFI_H
