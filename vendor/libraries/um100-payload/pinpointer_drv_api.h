/**
 * @file pinpointer_drv_api.h
 * @brief kernel/userspace pinpointer driver interface
 * @author www.bespoon.com
 * @date Bespoon 2014
 */
/*
 * Copyright (C) BeSpoon 2012 - 2016. No Part of the source code shall be
 * reproduced, transmitted, distributed, used nor modified in any way without
 * the express prior written authorization of Bespoon.
 */

#include "pinpointer_common_def.h"

#ifndef PINPOINTER_DRV_API_H
#define PINPOINTER_DRV_API_H
#include <osal_type.h>

/*from userspace device name is ppdevx.x*/
#define PINPOINTER_DEV_NAME	 "ppdev"
#define PINPOINTER_MAJOR     	51  /* Borrow Baycom radio modem major*/
#define N_PP_MINORS		32	/* ... up to 256 */

#ifdef ARM

#define PP_DEV_NODE_BASE       "/dev/"
#define PP_DEV_NODE_DEV1_ID   "2.0"
#define PP_DEV_NODE_DEV2_ID   "2.1"
#define PP_DEV_NODE_DEV3_ID   "2.2"
#define PP_PRODFS_PATH "/dev/mtd5ro"
#define PP_PRODFS_SIZE 4095 /*FIXME get the real size from kernel*/
#elif defined X86
#warning "Pinpointer dev file not defined on x86!"
#define PP_DEV_NODE_BASE      ""
#define PP_DEV_NODE_DEV1_ID   ""
#define PP_DEV_NODE_DEV2_ID   ""
#define PP_DEV_NODE_DEV3_ID   ""
#define PP_PRODFS_PATH ""
#define PP_PRODFS_SIZE 0
#elif defined ANDROID
#define PP_DEV_NODE_BASE      "/dev/"
#define PP_DEV_NODE_DEV1_ID   "0.0"
#define PP_DEV_NODE_DEV2_ID   "0.0"
#define PP_DEV_NODE_DEV3_ID   "0.0"
#define PP_PRODFS_PATH "/dev/block/mmcblk0p33"
#define PP_PRODFS_SIZE 4095 /*FIXME get the real size from kernel*/
#else
#warning Unknown MACHINE/ARCH in Pinpointer dev file
#endif

#define PP_DEV_NODE_DEV1   PP_DEV_NODE_BASE PINPOINTER_DEV_NAME PP_DEV_NODE_DEV1_ID
#define PP_DEV_NODE_DEV2   PP_DEV_NODE_BASE PINPOINTER_DEV_NAME PP_DEV_NODE_DEV2_ID
#define PP_DEV_NODE_DEV3   PP_DEV_NODE_BASE PINPOINTER_DEV_NAME PP_DEV_NODE_DEV3_ID
#define PP_DEV_NODE        PP_DEV_NODE_DEV1


/*********************** IO CONTROL *****************************************/

/**
 * struct pp_spi_transfer -borrowed from SPIDEV driver describes a single SPI transfer
 * @tx_buf: Holds pointer to userspace buffer with transmit data, or null.
 * @rx_buf: Holds pointer to userspace buffer for receive data, or null.
 * @len: Length of tx and rx buffers, in bytes.
 */
typedef struct pp_spi_transfer {
	OSAL_u8* tx_buf;
	OSAL_u8* rx_buf;
	OSAL_u32 len;
} pp_spi_transfer_t;

/**
 * struct pp_hw_gpio - change Pinpointer reset/pwr lines level
 * @rst_level : electric level of RST line
 * @rstg_level: electric level of RSTG line (analog reset)
 * @pwr_level: electric level of daughter power line (2.5V)
 */
typedef struct pp_hw_gpio {
	OSAL_u8 rst_level;
	OSAL_u8 rstg_level;
	OSAL_u8 pwr_level;
} pp_hw_gpio_t;


/**
 * struct pp_api_call - direct configuration for module API communication
 */
typedef struct pp_api_call {
	OSAL_u8   commandId; /**<[in] command to send to module (uwb_com_system_cmd_e,uwb_com_radio_cmd_e,uwb_com_seq_cmd_e or uwb_com_protocol_cmd_e)*/
	OSAL_u8   subCommand;/**<[in] sub command needed to precise action, can be a value or UWB_SC_NONE=0xFF*/
	OSAL_u8   rsvd[2];/**<Reserved and alignement*/
	OSAL_u32  parameter; /**<[in] parameter for this command (can be 8bits or 16bits, or UWB_P_NO_PARAMETER = 0xFFFFFFFF) */
	OSAL_u8*  additionalData; /**<[in] Additionnal Data to sent to the module (command that have a UWB_CS_COMMAND_N_DATA scheme only).Can be NULL*/
	OSAL_u32  additionalDataLen; /**<[in] Additionnal Data length, can be 0*/
	OSAL_u8*  outData; /**<[out] data sent by the module ( commands that have a UWB_CS_COMMAND_N_ANSWER scheme only), can be NULL*/
	OSAL_u32  outDataLen; /**<[out] data sent length, can be 0*/
} pp_api_call_t;

/**
 * struct pp_hw_bootrst_config - Boot and RESET pin configuration ( 0/0 (down, down), 1/1 (up/up), etc...). Module specific
 */
typedef struct pp_hw_pin_config {
    OSAL_u8 val;/**<[in/out] 0 if pin is set to DOWN, 1 for UP.*/
    OSAL_u8 rsvd[3];/**<Reserved and alignement*/
} pp_hw_pin_config_t;


#define PP_IOC_SPI_ACCESS 	_IOW ('z',1,  pp_spi_transfer_t)
#define PP_IOC_GPIO_ACCESS 	_IOW ('z',2,  pp_hw_gpio_t)

#define PP_IOC_API_CALL		_IOW  ('z',3, pp_api_call_t)

#define PP_IOC_S_CONFIG_PHY	_IOW ('z',4,  config_phy_t)
#define PP_IOC_G_CONFIG_PHY	_IOW ('z',5,  config_phy_t)

#define PP_IOC_FORBIDDEN6	_IO  ('z',6)
#define PP_IOC_FORBIDDEN7	_IO  ('z',7)
#define PP_IOC_FORBIDDEN8	_IO  ('z',8)
#define PP_IOC_FORBIDDEN9	_IO  ('z',9)
#define PP_IOC_FORBIDDEN10	_IO  ('z',10)
#define PP_IOC_FORBIDDEN11	_IO  ('z',11)
#define PP_IOC_FORBIDDEN12	_IO  ('z',12)
#define PP_IOC_FORBIDDEN13	_IO  ('z',13)
#define PP_IOC_FORBIDDEN14	_IO  ('z',14)

#define PP_IOC_OPEN             _IO  ('z',15)
#define PP_IOC_CLOSE            _IO  ('z',16)
#define PP_IOC_UNITTESTS        _IO  ('z',17)

#define PP_IOC_FORBIDDEN18	    _IO  ('z',18)
#define PP_IOC_FORBIDDEN19  	_IO  ('z',19)

#define PP_IOC_MODULE_SET_RESET		_IOW ('z',20, pp_hw_pin_config_t)
#define PP_IOC_MODULE_GET_RESET		_IOW ('z',21, pp_hw_pin_config_t)
#define PP_IOC_MODULE_SET_BOOTMODE	_IOW ('z',22, pp_hw_pin_config_t)
#define PP_IOC_MODULE_GET_BOOTMODE	_IOW ('z',23, pp_hw_pin_config_t)
#define PP_IOC_RESERVED24	    _IO  ('z',24)

#endif //PINPOINTER_DRV_API_H

#ifdef CONFIG_RTLS_PROTOCOL
#include "pinpointer_drv_api_protocol.h"
#endif//#ifdef CONFIG_RTLS_PROTOCOL
