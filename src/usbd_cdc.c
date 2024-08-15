#include "usbd_cdc.h"
#include "gs_usb.h"
#include "usbd_core.h"


extern USBD_CDC_ItfTypeDef cdc_if;
static USBD_CDC_HandleTypeDef hcdc;
extern USBD_HandleTypeDef hUSB;

static uint8_t UserRxBufferFS[1024];
static uint8_t UserTxBufferFS[1024];


uint8_t  USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    USBD_LL_OpenEP(pdev, GSUSB_CDC_ENDPOINT_IN, USBD_EP_TYPE_BULK, 64U);
    pdev->ep_in[GSUSB_CDC_ENDPOINT_IN & 0xFU].is_used = 1U;

    USBD_LL_OpenEP(pdev, GSUSB_CDC_ENDPOINT_OUT, USBD_EP_TYPE_BULK, 64U);
    pdev->ep_out[GSUSB_CDC_ENDPOINT_OUT & 0xFU].is_used = 1U;

    USBD_LL_OpenEP(pdev, GSUSB_CDC_ENDPOINT_CMD, USBD_EP_TYPE_INTR, 8U);
    pdev->ep_in[GSUSB_CDC_ENDPOINT_CMD & 0xFU].is_used = 1U;

    cdc_if.Init();

    hcdc.TxState = 0U;
    hcdc.RxState = 0U;

    USBD_LL_PrepareReceive(pdev, GSUSB_CDC_ENDPOINT_OUT, hcdc.RxBuffer, 64U);

    return USBD_OK;
}

uint8_t  USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx)
{
    UNUSED(cfgidx);

    /* Close EP IN */
    USBD_LL_CloseEP(pdev, GSUSB_CDC_ENDPOINT_IN);
    pdev->ep_in[GSUSB_CDC_ENDPOINT_IN & 0xFU].is_used = 0U;

    /* Close EP OUT */
    USBD_LL_CloseEP(pdev, GSUSB_CDC_ENDPOINT_OUT);
    pdev->ep_out[GSUSB_CDC_ENDPOINT_OUT & 0xFU].is_used = 0U;

    /* Close Command IN EP */
    USBD_LL_CloseEP(pdev, GSUSB_CDC_ENDPOINT_CMD);
    pdev->ep_in[GSUSB_CDC_ENDPOINT_CMD & 0xFU].is_used = 0U;

    /* DeInit  physical Interface components */
    cdc_if.DeInit();

    return USBD_OK;
}

uint8_t  USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req)
{
    uint16_t len;
    uint8_t ifalt = 0U;
    uint16_t status_info = 0U;
    uint8_t ret = USBD_OK;

    switch (req->bmRequest & USB_REQ_TYPE_MASK)
    {
        case USB_REQ_TYPE_CLASS:
            if (req->wLength)
            {
                if (req->bmRequest & 0x80U)
                {
                    cdc_if.Control(req->bRequest, (uint8_t *)hcdc.data, req->wLength);
                    len = MIN(7U, req->wLength);
                    USBD_CtlSendData(pdev, (uint8_t *)hcdc.data, len);
                }
                else
                {
                    hcdc.CmdOpCode = req->bRequest;
                    hcdc.CmdLength = req->wLength;

                    USBD_CtlPrepareRx(pdev, (uint8_t *)hcdc.data, req->wLength);
                }
            }
            else
            {
                cdc_if.Control(req->bRequest, (uint8_t *)req, 0U);
            }
            break;

        case USB_REQ_TYPE_STANDARD:
            switch (req->bRequest)
            {
                case USB_REQ_GET_STATUS:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED)
                    {
                        USBD_CtlSendData(pdev, (uint8_t *)&status_info, 2U);
                    }
                    else
                    {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_GET_INTERFACE:
                    if (pdev->dev_state == USBD_STATE_CONFIGURED)
                    {
                        USBD_CtlSendData(pdev, &ifalt, 1U);
                    }
                    else
                    {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_SET_INTERFACE:
                    if (pdev->dev_state != USBD_STATE_CONFIGURED)
                    {
                        USBD_CtlError(pdev, req);
                        ret = USBD_FAIL;
                    }
                    break;

                case USB_REQ_CLEAR_FEATURE:
                    break;

                default:
                    USBD_CtlError(pdev, req);
                    ret = USBD_FAIL;
                    break;
            }
            break;

        default:
            USBD_CtlError(pdev, req);
            ret = USBD_FAIL;
            break;
    }
    return ret;
}

uint8_t  USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    PCD_HandleTypeDef *hpcd = pdev->pData;
    if ((pdev->ep_in[epnum].total_length > 0U) && ((pdev->ep_in[epnum].total_length % hpcd->IN_ep[epnum].maxpacket) == 0U))
    {
        /* Update the packet total length */
        pdev->ep_in[epnum].total_length = 0U;

        /* Send ZLP */
        USBD_LL_Transmit(pdev, epnum, NULL, 0U);
    }
    else
    {
        hcdc.TxState = 0U;
        cdc_if.TransmitCplt(hcdc.TxBuffer, &hcdc.TxLength, epnum);
    }
    return USBD_OK;
}

uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    /* Get the received data length */
    hcdc.RxLength = USBD_LL_GetRxDataSize(pdev, epnum);

    /* USB data will be immediately processed, this allow next USB traffic being
    NAKed till the end of the application Xfer */
    cdc_if.Receive(hcdc.RxBuffer, &hcdc.RxLength);

    return USBD_OK;
}

uint8_t  USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev)
{
    UNUSED(pdev);
    cdc_if.Control(hcdc.CmdOpCode,
                  (uint8_t *)(void *)hcdc.data,
                  (uint16_t)hcdc.CmdLength);
    if (hcdc.CmdOpCode != 0xFFU)
    {
        cdc_if.Control(hcdc.CmdOpCode,
                       (uint8_t *)hcdc.data,
                       (uint16_t)hcdc.CmdLength);
        hcdc.CmdOpCode = 0xFFU;
    }
    return USBD_OK;
}

/**
  * @brief  USBD_CDC_SetTxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Tx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetTxBuffer(uint8_t  *pbuff,
                              uint16_t length)
{
    hcdc.TxBuffer = pbuff;
    hcdc.TxLength = length;

    return USBD_OK;
}

/**
  * @brief  USBD_CDC_SetRxBuffer
  * @param  pdev: device instance
  * @param  pbuff: Rx Buffer
  * @retval status
  */
uint8_t  USBD_CDC_SetRxBuffer(uint8_t  *pbuff)
{
    hcdc.RxBuffer = pbuff;

    return USBD_OK;
}

/**
  * @brief  USBD_CDC_TransmitPacket
  *         Transmit packet on IN endpoint
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev)
{
    if (hcdc.TxState == 0U)
    {
        /* Tx Transfer in progress */
        hcdc.TxState = 1U;

        /* Update the packet total length */
        pdev->ep_in[GSUSB_CDC_ENDPOINT_IN & 0xFU].total_length = hcdc.TxLength;

        /* Transmit next packet */
        USBD_LL_Transmit(pdev, GSUSB_CDC_ENDPOINT_IN, hcdc.TxBuffer,
                        (uint16_t)hcdc.TxLength);

        return USBD_OK;
    }
    else
    {
        return USBD_BUSY;
    }
}

/**
  * @brief  USBD_CDC_ReceivePacket
  *         prepare OUT Endpoint for reception
  * @param  pdev: device instance
  * @retval status
  */
uint8_t  USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev)
{
    /* Prepare Out endpoint to receive next packet */
    USBD_LL_PrepareReceive(pdev,
                        GSUSB_CDC_ENDPOINT_OUT,
                        hcdc.RxBuffer,
                        64U);

    return USBD_OK;
}

static int8_t CDC_Init(void)
{
    /* Set Application Buffers */
    USBD_CDC_SetTxBuffer(UserTxBufferFS, 0);
    USBD_CDC_SetRxBuffer(UserRxBufferFS);

    return (USBD_OK);
}

static int8_t CDC_DeInit(void)
{
    return (USBD_OK);   
}

static int8_t CDC_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    UNUSED(length);
    static USBD_CDC_LineCodingTypeDef line_coding;

    switch (cmd) {
    case CDC_SEND_ENCAPSULATED_COMMAND:
    case CDC_GET_ENCAPSULATED_RESPONSE:
    case CDC_SET_COMM_FEATURE:
    case CDC_GET_COMM_FEATURE:
    case CDC_CLEAR_COMM_FEATURE:
        break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
    case CDC_SET_LINE_CODING:
        line_coding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
        line_coding.format = pbuf[4];
        line_coding.paritytype = pbuf[5];
        line_coding.datatype = pbuf[6];

        //Change_UART_Setting(cdc_ch);
        break;

    case CDC_GET_LINE_CODING:
        pbuf[0] = (uint8_t)(line_coding.bitrate);
        pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
        pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
        pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
        pbuf[4] = line_coding.format;
        pbuf[5] = line_coding.paritytype;
        pbuf[6] = line_coding.datatype;
        break;

    case CDC_SET_CONTROL_LINE_STATE:
    case CDC_SEND_BREAK:
        break;

    default:
        break;
    }

    return (USBD_OK);
}

uint8_t CDC_Transmit(uint8_t* Buf, uint16_t Len)
{
    uint8_t result = USBD_OK;

    if (hcdc.TxState != 0)
    {
        return USBD_BUSY;
    }
    USBD_CDC_SetTxBuffer(Buf, Len);
    result = USBD_CDC_TransmitPacket(&hUSB);
    return result;
}

static int8_t CDC_Receive(uint8_t* Buf, uint32_t *Len)
{
    // Echo back the received data.
    CDC_Transmit(Buf, *Len);

    USBD_CDC_SetRxBuffer(&Buf[0]);
    USBD_CDC_ReceivePacket(&hUSB);
    return (USBD_OK);
}

int8_t CDC_TransmitCplt(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    UNUSED(Buf);
    UNUSED(Len);
    UNUSED(epnum);
    return (USBD_OK);
}

USBD_CDC_ItfTypeDef cdc_if = 
{
    CDC_Init,
    CDC_DeInit,
    CDC_Control,
    CDC_Receive,
    CDC_TransmitCplt
};
