/**
 * @file default_sfi_fast_rate.h
 * @brief Example file for Default superframe schema when doing Fast Rate (e.g. a superframe at 62.5ms)
 * @author cca@bespoon.com
 * @date 17/09/2013
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */


// HACK:
#ifndef CONFIG_RTLS_PROTOCOL
#define CONFIG_RTLS_PROTOCOL
#endif

#ifndef CONFIG_RTLS_FULL_LOC
#define CONFIG_RTLS_FULL_LOC
#endif

#ifndef ARM
#define ARM
#endif


#include "pinpointer_drv_api.h"

#ifndef DEFAULT_SFI_H
#define DEFAULT_SFI_H

#define DEFAULT_SFI_PHS 6
#define DEFAULT_SFI_PSN 32
#define DEFAULT_SFI_PSS   64

#define DEFAULT_SFI_SLOT_B1 5
#define DEFAULT_SFI_SLOT_B2 11
#define DEFAULT_SFI_SLOT_RANGING 15//not really used anyway
#define DEFAULT_SFI_ZONE_BEACON 2
#define DEFAULT_SFI_ZONE_RANGING 2
#define DEFAULT_SFI_ZONE_RDV 2

#define DEFAULT_SFI_CHANNEL 2
#define DEFAULT_SFI_RX_RFPATH RF_PATH2_1
#define DEFAULT_SFI_TX_RFPATH RF_PATH1_1

extern frame_info_t Coord_Slot_Desc[];
extern frame_info_t Base_Slot_Desc[];
extern frame_info_t Tag_Slot_Desc[];

extern int nCoord_Slot_Desc;
extern int nBase_Slot_Desc;
extern int nTag_Slot_Desc;


#endif
