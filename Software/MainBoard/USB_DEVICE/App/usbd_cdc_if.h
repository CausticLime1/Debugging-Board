/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.h
  * @brief          : Header for usbd_cdc_if.c file.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USBD_CDC_IF_H__
#define __USBD_CDC_IF_H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @defgroup USBD_CDC_IF
  * @brief Usb VCP device module
  * @{
  */

/** @defgroup USBD_CDC_IF_Exported_Defines USBD_CDC_IF_Exported_Defines
  * @brief Defines.
  * @{
  */
/* USER CODE BEGIN EXPORTED_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048
/* USER CODE END EXPORTED_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Types USBD_CDC_IF_Exported_Types
  * @brief Types.
  * @{
  */

/* USER CODE BEGIN EXPORTED_TYPES */
/* REMOVED: USBD_CDC_LineCodingTypeDef definition (already in usbd_cdc.h) */
/* USER CODE END EXPORTED_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Macros USBD_CDC_IF_Exported_Macros
  * @brief Aliases.
  * @{
  */

/* USER CODE BEGIN EXPORTED_MACROS */

/* USER CODE END EXPORTED_MACROS */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

/* USER CODE BEGIN EXPORTED_VARIABLES */
/* Shared variables for main.c */
extern USBD_CDC_LineCodingTypeDef LineCoding;
extern uint8_t UserRxBufferHS[APP_RX_DATA_SIZE];
extern uint8_t UserTxBufferHS[APP_TX_DATA_SIZE];
extern uint32_t UserRxBufPtrIn;
extern uint32_t UserRxBufPtrOut;

/* Expose the Interface Structure so usb_device.c can see it */
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_FunctionsPrototypes USBD_CDC_IF_Exported_FunctionsPrototypes
  * @brief Public functions declaration.
  * @{
  */

uint8_t CDC_Transmit_HS(uint8_t* Buf, uint16_t Len);

/* USER CODE BEGIN EXPORTED_FUNCTIONS */

/* USER CODE END EXPORTED_FUNCTIONS */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CDC_IF_H__ */
