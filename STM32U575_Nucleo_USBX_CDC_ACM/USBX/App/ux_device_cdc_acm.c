/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    ux_device_cdc_acm.c
  * @author  MCD Application Team
  * @brief   USBX Device applicative file
  ******************************************************************************
    * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "ux_device_cdc_acm.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define APP_RX_DATA_SIZE   2048
#define APP_TX_DATA_SIZE   2048

/* Rx/TX flag */
#define RX_NEW_RECEIVED_DATA      0x01
#define TX_NEW_TRANSMITTED_DATA   0x02

/* Data length for vcp */
#define VCP_WORDLENGTH8  8
#define VCP_WORDLENGTH9  9

/* the minimum baudrate */
#define MIN_BAUDRATE     9600

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

UX_SLAVE_CLASS_CDC_ACM  *cdc_acm;

/* Data to send over USB CDC are stored in this buffer   */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint32_t UserTxBufPtrIn;
uint32_t UserTxBufPtrOut;

extern TX_EVENT_FLAGS_GROUP EventFlag;

UX_SLAVE_CLASS_CDC_ACM_LINE_CODING_PARAMETER CDC_VCP_LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  USBD_CDC_ACM_Activate
  *         This function is called when insertion of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Activate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Activate */

  /* Save the CDC instance */
  cdc_acm = (UX_SLAVE_CLASS_CDC_ACM*) cdc_acm_instance;

  /* USER CODE END USBD_CDC_ACM_Activate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_Deactivate
  *         This function is called when extraction of a CDC ACM device.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_Deactivate(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_Deactivate */

  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  /* Reset the cdc acm instance */
  cdc_acm = UX_NULL;

  /* USER CODE END USBD_CDC_ACM_Deactivate */

  return;
}

/**
  * @brief  USBD_CDC_ACM_ParameterChange
  *         This function is invoked to manage the CDC ACM class requests.
  * @param  cdc_acm_instance: Pointer to the cdc acm class instance.
  * @retval none
  */
VOID USBD_CDC_ACM_ParameterChange(VOID *cdc_acm_instance)
{
  /* USER CODE BEGIN USBD_CDC_ACM_ParameterChange */

  UX_PARAMETER_NOT_USED(cdc_acm_instance);

  ULONG request;
  UX_SLAVE_TRANSFER *transfer_request;
  UX_SLAVE_DEVICE *device;

  /* Get the pointer to the device.  */
  device = &_ux_system_slave -> ux_system_slave_device;

  /* Get the pointer to the transfer request associated with the control endpoint. */
  transfer_request = &device -> ux_slave_device_control_endpoint.ux_slave_endpoint_transfer_request;

  request = *(transfer_request -> ux_slave_transfer_request_setup + UX_SETUP_REQUEST);

  switch (request)
  {
    case UX_SLAVE_CLASS_CDC_ACM_SET_LINE_CODING :

      /* Get the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_GET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_GET_LINE_CODING :

      /* Set the Line Coding parameters */
      if (ux_device_class_cdc_acm_ioctl(cdc_acm, UX_SLAVE_CLASS_CDC_ACM_IOCTL_SET_LINE_CODING,
                                        &CDC_VCP_LineCoding) != UX_SUCCESS)
      {
        Error_Handler();
      }

      break;

    case UX_SLAVE_CLASS_CDC_ACM_SET_CONTROL_LINE_STATE :
    default :
      break;
  }

  /* USER CODE END USBD_CDC_ACM_ParameterChange */

  return;
}

/* USER CODE BEGIN 1 */

/**
  * @brief  Function implementing usbx_cdc_acm_thread_entry.
  * @param arg: Not used
  * @retval None
  */
void usbx_cdc_acm_read_write_thread_entry(ULONG arg)
{
  UX_SLAVE_DEVICE *device;
  UX_SLAVE_INTERFACE *data_interface;
  UX_SLAVE_CLASS_CDC_ACM *cdc_acm;
  ULONG actual_length;
  ULONG ux_status = UX_SUCCESS;

  /* Get device */
  device = &_ux_system_slave->ux_system_slave_device;

  while (1)
  {
    /* Check if device is configured */
    if (device->ux_slave_device_state == UX_DEVICE_CONFIGURED)
    {
      /* Get Data interface */
      data_interface = device->ux_slave_device_first_interface->ux_slave_interface_next_interface;

      /* Compares two memory blocks ux_slave_class_name and _ux_system_slave_class_cdc_acm_name */
      ux_status = ux_utility_memory_compare(data_interface->ux_slave_interface_class->ux_slave_class_name,
                                            _ux_system_slave_class_cdc_acm_name,
                                            ux_utility_string_length_get(_ux_system_slave_class_cdc_acm_name));

      /* Check Compares success */
      if (ux_status == UX_SUCCESS)
      {
        cdc_acm =  data_interface->ux_slave_interface_class_instance;

#ifndef UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE
        /* Set transmission_status to UX_FALSE for the first time */
        cdc_acm -> ux_slave_class_cdc_acm_transmission_status = UX_FALSE;
#endif /* UX_DEVICE_CLASS_CDC_ACM_TRANSMISSION_DISABLE */

        /* Read the received data in blocking mode */
        ux_device_class_cdc_acm_read(cdc_acm, (UCHAR *)UserRxBufferFS, 1,
                                     &actual_length);
        if (actual_length != 0)
        {
          ux_status = ux_device_class_cdc_acm_write(cdc_acm, (UCHAR*)(UserRxBufferFS), 1, &actual_length);
        }
      }
    }
    else
    {
      tx_thread_sleep(MS_TO_TICK(10));
    }
  }
}

/* USER CODE END 1 */
