#include "usbd_ctlreq.h"

#define CDC_SEND_ENCAPSULATED_COMMAND               0x00U
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01U
#define CDC_SET_COMM_FEATURE                        0x02U
#define CDC_GET_COMM_FEATURE                        0x03U
#define CDC_CLEAR_COMM_FEATURE                      0x04U
#define CDC_SET_LINE_CODING                         0x20U
#define CDC_GET_LINE_CODING                         0x21U
#define CDC_SET_CONTROL_LINE_STATE                  0x22U
#define CDC_SEND_BREAK                              0x23U

typedef struct _USBD_CDC_Itf
{
    int8_t (* Init)(void);
    int8_t (* DeInit)(void);
    int8_t (* Control)(uint8_t cmd, uint8_t *pbuf, uint16_t length);
    int8_t (* Receive)(uint8_t *Buf, uint32_t *Len);
    int8_t (* TransmitCplt)(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
} USBD_CDC_ItfTypeDef;


typedef struct
{
    uint32_t data[64U / 4U];      /* Force 32bits alignment */
    uint8_t  CmdOpCode;
    uint8_t  CmdLength;
    uint8_t  *RxBuffer;
    uint8_t  *TxBuffer;
    uint32_t RxLength;
    uint32_t TxLength;

    __IO uint32_t TxState;
    __IO uint32_t RxState;
}
USBD_CDC_HandleTypeDef;

typedef struct
{
    uint32_t bitrate;
    uint8_t format;
    uint8_t paritytype;
    uint8_t datatype;
} USBD_CDC_LineCodingTypeDef;
