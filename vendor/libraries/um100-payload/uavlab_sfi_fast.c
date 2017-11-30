/*
 * uavlab_sfi_fast.c
 *
 *  Created on: Nov 30, 2017
 *      Author: krisklau
 */




#include "uavlab_sfi_fast.h"

int nCoord_Slot_Desc = 19;
static frame_info_t Coord_Slot_Desc[] = {
  { .type = FIT_BEACON1,  .act = TX,    .slot_index = 5,  .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = TX,    .slot_index = 11, .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 15,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 16,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 17,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 18,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 19,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 20,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 21,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 22,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 23,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 24,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 25,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 26,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 27,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 28,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 29,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 30,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RDV,  .act = NO_ACTION, .slot_index = 31,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};

int nBase_Slot_Desc = 29;
static frame_info_t Base_Slot_Desc[] = {
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 0,  .beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 1,  .beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 2,  .beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 3,  .beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 4,  .beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 5,  .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 6,  .beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 7,  .beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 8,  .beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 9,  .beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 10,   .beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 11,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 15,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 16,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 17,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 18,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 19,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 20,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 21,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 22,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 23,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 24,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 25,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 26,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 27,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 28,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 29,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = RX,    .slot_index = 30,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RDV,  .act = NO_ACTION, .slot_index = 31,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};

int nTag_Slot_Desc = 29;
static frame_info_t Tag_Slot_Desc[] = {
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 0,  .beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 1,  .beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 2,  .beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 3,  .beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 4,  .beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON1,  .act = RX,    .slot_index = 5,  .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 6,  .beacon_id = 5, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 7,  .beacon_id = 4, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 8,  .beacon_id = 3, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 9,  .beacon_id = 2, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 10,   .beacon_id = 1, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_BEACON2,  .act = RX,    .slot_index = 11,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_BEACON,  .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 15,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 16,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 17,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 18,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 19,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 20,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 21,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 22,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 23,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 24,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 25,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 26,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 27,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 28,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 29,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RANGING,  .act = TX,    .slot_index = 30,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RANGING, .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_TX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK},
  { .type = FIT_RDV,  .act = NO_ACTION, .slot_index = 31,   .beacon_id = 0, .zone = DEFAULT_SFI_ZONE_RDV,    .channel = DEFAULT_SFI_CHANNEL, .rf_path = DEFAULT_SFI_RX_RFPATH, .rngDataType=RNG_CONF_LOC_NRJ_MASK|RNG_CONF_LOC_TOA_MASK|RNG_CONF_LOC_METR_MASK}
};
