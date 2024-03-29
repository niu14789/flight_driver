/*!
    \file  cdc_acm_core.h
    \brief the header file of IAP driver
*/

/*
    Copyright (C) 2017 GigaDevice

    2017-02-10, V1.0.0, firmware for GD32F30x
*/

#ifndef CDC_ACM_CORE_H
#define CDC_ACM_CORE_H

#include "usbd_std.h"

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif

#define PACKET_SEND_FINISH          0x00000002
#define PACKET_IN_SENDING           0xfffffffd
#define PACKET_RECEIVE_FINISH       0x00000001
#define PACKET_IN_RECEIVING         0xfffffffe
#define PACKET_IDLE                 0x00000003


#define USB_DESCTYPE_CS_INTERFACE               0x24
#define USB_CDC_ACM_CONFIG_DESC_SIZE            0x43

#define CDC_ACM_DESC_SIZE                       0x3A

#define CDC_ACM_DESC_TYPE                       0x21

#define SEND_ENCAPSULATED_COMMAND               0x00
#define GET_ENCAPSULATED_RESPONSE               0x01
#define SET_COMM_FEATURE                        0x02
#define GET_COMM_FEATURE                        0x03
#define CLEAR_COMM_FEATURE                      0x04
#define SET_LINE_CODING                         0x20
#define GET_LINE_CODING                         0x21
#define SET_CONTROL_LINE_STATE                  0x22
#define SEND_BREAK                              0x23
#define NO_CMD                                  0xFF

#pragma pack(1)

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size. */
    uint8_t  bDescriptorSubtype;          /*!< bDescriptorSubtype: header function descriptor */
    uint16_t  bcdCDC;                     /*!< bcdCDC: low byte of spec release number (CDC1.10) */
} usb_descriptor_header_function_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size. */
    uint8_t  bDescriptorSubtype;          /*!< bDescriptorSubtype:  call management function descriptor */
    uint8_t  bmCapabilities;              /*!< bmCapabilities: D0 is reset, D1 is ignored */
    uint8_t  bDataInterface;              /*!< bDataInterface: 1 interface used for call management */
} usb_descriptor_call_managment_function_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size. */
    uint8_t  bDescriptorSubtype;          /*!< bDescriptorSubtype: abstract control management desc */
    uint8_t  bmCapabilities;              /*!< bmCapabilities: D1 */
} usb_descriptor_acm_function_struct;

typedef struct
{
    usb_descriptor_header_struct Header;  /*!< descriptor header, including type and size. */
    uint8_t  bDescriptorSubtype;          /*!< bDescriptorSubtype: union func desc */
    uint8_t  bMasterInterface;            /*!< bMasterInterface: communication class interface */
    uint8_t  bSlaveInterface0;            /*!< bSlaveInterface0: data class interface */
} usb_descriptor_union_function_struct;

#pragma pack()

typedef struct
{
    usb_descriptor_configuration_struct               config;
    usb_descriptor_interface_struct                   cdc_loopback_interface;
    usb_descriptor_header_function_struct             cdc_loopback_header;
    usb_descriptor_call_managment_function_struct     cdc_loopback_call_managment;
    usb_descriptor_acm_function_struct                cdc_loopback_acm;
    usb_descriptor_union_function_struct              cdc_loopback_union;
    usb_descriptor_endpoint_struct                    cdc_loopback_cmd_endpoint;
    usb_descriptor_interface_struct                   cdc_loopback_data_interface;
    usb_descriptor_endpoint_struct                    cdc_loopback_out_endpoint;
    usb_descriptor_endpoint_struct                    cdc_loopback_in_endpoint;
} usb_descriptor_configuration_set_struct;

extern void* const usbd_strings[USB_STRING_COUNT];
extern const usb_descriptor_device_struct device_descriptor;
extern usb_descriptor_configuration_set_struct configuration_descriptor;

/* function declarations */
/* initialize the CDC ACM device */
usbd_status_enum cdc_acm_init(void *pudev, uint8_t config_index);
/* de-initialize the CDC ACM device */
usbd_status_enum cdc_acm_deinit(void *pudev, uint8_t config_index);
/* handle the CDC ACM class-specific requests */
usbd_status_enum cdc_acm_req_handler(void *pudev, usb_device_req_struct *req);
/* handle CDC ACM data */
usbd_status_enum cdc_acm_data_handler(void *pudev, usbd_dir_enum rx_tx, uint8_t ep_id);

/* receive CDC ACM data */
void cdc_acm_data_receive(void *pudev, U8 *pbyReceiveBuffer, int iReceiveSize);
/* send CDC ACM data */
void cdc_acm_data_send(void *pudev, U8 *pbySendBuffer, int iSendLen);
/* command data received on control endpoint */
usbd_status_enum cdc_acm_EP0_RxReady(void  *pudev);

int usb_init(void);

#endif  /* CDC_ACM_CORE_H */
