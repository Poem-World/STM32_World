#include "usbh_serial.h"

static uint32_t Serial_BaudRate = 115200;
static uint8_t  Serial_DataBits = 8;
static uint8_t  Serial_Parity = 0;
static uint8_t  Serial_StopBits = 0;

void USBH_Serial_InitSettings(uint32_t baudrate, uint8_t databits, uint8_t parity, uint8_t stopbits) {
    Serial_BaudRate = baudrate;
    Serial_DataBits = databits;
    Serial_Parity = parity;
    Serial_StopBits = stopbits;
}

static char rx_line_buffer[USBH_SERIAL_RX_LINE_SIZE];
static uint16_t rx_line_idx = 0;

char datafull[USBH_SERIAL_RX_LINE_SIZE] = {0};
volatile uint8_t datafull_ready = 0;

static void ProcessSerialRx(uint8_t *data, uint16_t len) {
  for (uint16_t i = 0; i < len; i++) {
    char c = (char)data[i];
    if (rx_line_idx < USBH_SERIAL_RX_LINE_SIZE - 1) {
      rx_line_buffer[rx_line_idx++] = c;
    }
    if (c == '\n') {
      rx_line_buffer[rx_line_idx] = '\0';
      strncpy(datafull, rx_line_buffer, USBH_SERIAL_RX_LINE_SIZE - 1);
      datafull[USBH_SERIAL_RX_LINE_SIZE - 1] = '\0';
      datafull_ready = 1;
      rx_line_idx = 0;
    }
  }
}

// CDC Reception Support
static uint8_t cdc_rx_buf[64];
static uint8_t cdc_rx_started = 0;

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost) {
  uint16_t len = USBH_CDC_GetLastReceivedDataSize(phost);
  if (len > 0) {
    ProcessSerialRx(cdc_rx_buf, len);
  }
  USBH_CDC_Receive(phost, cdc_rx_buf, sizeof(cdc_rx_buf));
}

// ==========================================
// CP210X DRIVER
// ==========================================
typedef enum { CP210X_INIT_IFC_ENABLE, CP210X_INIT_SET_BAUDDIV, CP210X_INIT_SET_LINE_CTL, CP210X_INIT_DONE } CP210x_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; CP210x_State state; } CP210x_HandleTypeDef;
static CP210x_HandleTypeDef CP210x_Handle;

static USBH_StatusTypeDef CP210x_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x10C4) return USBH_FAIL;
  printf("[CP210x] Vendor ID Matched!\r\n");
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0x00, 0x00);
  if (interface == 0xFF) { printf("[CP210x] Interface failed!\r\n"); return USBH_FAIL; }
  printf("[CP210x] Interface found: %d\r\n", interface);
  USBH_SelectInterface(phost, interface);
  phost->pActiveClass->pData = &CP210x_Handle;
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  for (uint8_t i = 0; i < phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints; i++) {
    if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress & 0x80) {
      CP210x->InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      CP210x->InEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    } else {
      CP210x->OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      CP210x->OutEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    }
  }
  CP210x->OutPipe = USBH_AllocPipe(phost, CP210x->OutEp);
  CP210x->InPipe  = USBH_AllocPipe(phost, CP210x->InEp);
  USBH_OpenPipe(phost, CP210x->OutPipe, CP210x->OutEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, CP210x->OutEpSize);
  USBH_OpenPipe(phost, CP210x->InPipe, CP210x->InEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, CP210x->InEpSize);
  USBH_LL_SetToggle(phost, CP210x->OutPipe, 0); USBH_LL_SetToggle(phost, CP210x->InPipe, 0);
  CP210x->state = CP210X_INIT_IFC_ENABLE; 
  printf("[CP210x] Init OK!\r\n");
  return USBH_OK;
}
static uint8_t cp210x_rx_buf[64];
static uint8_t cp210x_rx_state = 0;

static USBH_StatusTypeDef CP210x_DeInit(USBH_HandleTypeDef *phost) {
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  if (CP210x->OutPipe) { USBH_ClosePipe(phost, CP210x->OutPipe); USBH_FreePipe(phost, CP210x->OutPipe); }
  if (CP210x->InPipe)  { USBH_ClosePipe(phost, CP210x->InPipe);  USBH_FreePipe(phost, CP210x->InPipe);  }
  cp210x_rx_state = 0;
  return USBH_OK;
}
static USBH_StatusTypeDef CP210x_CtlReq(USBH_HandleTypeDef *phost, uint8_t req, uint16_t value) {
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_INTERFACE;
  phost->Control.setup.b.bRequest = req; phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = 0; phost->Control.setup.b.wLength.w = 0;
  return USBH_CtlReq(phost, NULL, 0);
}
static USBH_StatusTypeDef CP210x_Requests(USBH_HandleTypeDef *phost) {
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  USBH_StatusTypeDef status;
  switch (CP210x->state) {
    case CP210X_INIT_IFC_ENABLE: 
      status = CP210x_CtlReq(phost, 0x00, 0x0001);
      if (status == USBH_OK) { printf("[CP210x] IFC_ENABLE OK\r\n"); CP210x->state = CP210X_INIT_SET_BAUDDIV; }
      break;
    case CP210X_INIT_SET_BAUDDIV: {
      uint32_t bauddiv = 3686400 / Serial_BaudRate;
      status = CP210x_CtlReq(phost, 0x01, bauddiv);
      if (status == USBH_OK) { printf("[CP210x] SET_BAUDDIV (%lu) OK\r\n", Serial_BaudRate); CP210x->state = CP210X_INIT_SET_LINE_CTL; }
      break;
    }
    case CP210X_INIT_SET_LINE_CTL: {
      uint16_t line_ctl = (Serial_DataBits << 8);
      if (Serial_Parity == 1) line_ctl |= 0x0010;
      else if (Serial_Parity == 2) line_ctl |= 0x0020;
      else if (Serial_Parity == 3) line_ctl |= 0x0030;
      else if (Serial_Parity == 4) line_ctl |= 0x0040;
      if (Serial_StopBits == 2) line_ctl |= 0x0002;
      
      status = CP210x_CtlReq(phost, 0x03, line_ctl);
      if (status == USBH_OK) { printf("[CP210x] SET_LINE_CTL OK\r\n"); CP210x->state = CP210X_INIT_DONE; }
      break;
    }
    case CP210X_INIT_DONE: return USBH_OK;
  }
  return USBH_BUSY;
}
static USBH_StatusTypeDef CP210x_BgndProcess(USBH_HandleTypeDef *phost) {
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState == HOST_CLASS && !phost->device.is_disconnected) {
    if (cp210x_rx_state == 0) {
      USBH_BulkReceiveData(phost, cp210x_rx_buf, CP210x->InEpSize, CP210x->InPipe);
      cp210x_rx_state = 1;
    } else {
      USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CP210x->InPipe);
      if (urb == USBH_URB_DONE) {
        uint32_t len = USBH_LL_GetLastXferSize(phost, CP210x->InPipe);
        if (len > 0) {
          ProcessSerialRx(cp210x_rx_buf, (uint16_t)len);
        }
        USBH_BulkReceiveData(phost, cp210x_rx_buf, CP210x->InEpSize, CP210x->InPipe);
        cp210x_rx_state = 1;
      } else if (urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
        cp210x_rx_state = 0;
      }
    }
  } else {
    cp210x_rx_state = 0;
  }
  return USBH_OK;
}
static USBH_StatusTypeDef CP210x_SOFProcess(USBH_HandleTypeDef *phost) { return USBH_OK; }
USBH_ClassTypeDef CP210x_Class = { "CP210x_Class", 0xFF, CP210x_Init, CP210x_DeInit, CP210x_Requests, CP210x_BgndProcess, CP210x_SOFProcess, NULL };
USBH_StatusTypeDef USBH_CP210x_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CP210x->OutPipe); if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, CP210x->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// FTDI DRIVER
// ==========================================
typedef enum { FTDI_INIT_RESET, FTDI_INIT_SET_BAUD, FTDI_INIT_SET_DATA, FTDI_INIT_SET_FLOW, FTDI_INIT_SET_MODEM, FTDI_INIT_DONE } FTDI_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; FTDI_State state; } FTDI_HandleTypeDef;
static FTDI_HandleTypeDef FTDI_Handle;
static USBH_StatusTypeDef FTDI_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x0403) return USBH_FAIL;
  printf("[FTDI] Vendor ID Matched!\r\n");
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0xFF, 0xFF);
  if (interface == 0xFF) { printf("[FTDI] FindInterface Failed!\r\n"); return USBH_FAIL; }
  printf("[FTDI] Interface found: %d\r\n", interface);
  USBH_SelectInterface(phost, interface);
  phost->pActiveClass->pData = &FTDI_Handle;
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  for (uint8_t i = 0; i < phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints; i++) {
    if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress & 0x80) {
      FTDI->InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      FTDI->InEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    } else {
      FTDI->OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      FTDI->OutEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    }
  }
  FTDI->OutPipe = USBH_AllocPipe(phost, FTDI->OutEp); FTDI->InPipe  = USBH_AllocPipe(phost, FTDI->InEp);
  USBH_OpenPipe(phost, FTDI->OutPipe, FTDI->OutEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, FTDI->OutEpSize);
  USBH_OpenPipe(phost, FTDI->InPipe, FTDI->InEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, FTDI->InEpSize);
  USBH_LL_SetToggle(phost, FTDI->OutPipe, 0); USBH_LL_SetToggle(phost, FTDI->InPipe, 0);
  FTDI->state = FTDI_INIT_RESET; return USBH_OK;
}
static uint8_t FTDI_RxBuffer[64];
static uint8_t ftdi_rx_state = 0;

static USBH_StatusTypeDef FTDI_DeInit(USBH_HandleTypeDef *phost) {
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  if (FTDI->OutPipe) { USBH_ClosePipe(phost, FTDI->OutPipe); USBH_FreePipe(phost, FTDI->OutPipe); }
  if (FTDI->InPipe)  { USBH_ClosePipe(phost, FTDI->InPipe);  USBH_FreePipe(phost, FTDI->InPipe);  }
  ftdi_rx_state = 0;
  return USBH_OK;
}
static USBH_StatusTypeDef FTDI_CtlReq(USBH_HandleTypeDef *phost, uint8_t req, uint16_t value, uint16_t index) {
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
  phost->Control.setup.b.bRequest = req; phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = index; phost->Control.setup.b.wLength.w = 0;
  return USBH_CtlReq(phost, NULL, 0);
}
static USBH_StatusTypeDef FTDI_Requests(USBH_HandleTypeDef *phost) {
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  USBH_StatusTypeDef status;
  switch (FTDI->state) {
    case FTDI_INIT_RESET:     
        status = FTDI_CtlReq(phost, 0x00, 0x0000, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) { printf("[FTDI] RESET status: %d\r\n", status); FTDI->state = FTDI_INIT_SET_BAUD; }
        break;
    case FTDI_INIT_SET_BAUD: {
        uint32_t divisor = 3000000 / Serial_BaudRate;
        status = FTDI_CtlReq(phost, 0x03, divisor, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) { printf("[FTDI] BAUD (%lu) status: %d\r\n", Serial_BaudRate, status); FTDI->state = FTDI_INIT_SET_DATA; }
        break;
    }
    case FTDI_INIT_SET_DATA: {
        uint16_t data = (Serial_DataBits & 0x0F);
        if (Serial_Parity == 1) data |= (1 << 8);
        else if (Serial_Parity == 2) data |= (2 << 8);
        else if (Serial_Parity == 3) data |= (3 << 8);
        else if (Serial_Parity == 4) data |= (4 << 8);
        
        if (Serial_StopBits == 2) data |= (2 << 11);
        
        status = FTDI_CtlReq(phost, 0x04, data, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) { printf("[FTDI] DATA status: %d\r\n", status); FTDI->state = FTDI_INIT_SET_FLOW; }
        break;
    }
    case FTDI_INIT_SET_FLOW:  
        status = FTDI_CtlReq(phost, 0x02, 0x0000, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) { printf("[FTDI] FLOW status: %d\r\n", status); FTDI->state = FTDI_INIT_SET_MODEM; }
        break;
    case FTDI_INIT_SET_MODEM: 
        status = FTDI_CtlReq(phost, 0x01, 0x0303, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) { printf("[FTDI] MODEM status: %d\r\n", status); FTDI->state = FTDI_INIT_DONE; }
        break;
    case FTDI_INIT_DONE:      return USBH_OK;
  }
  return USBH_BUSY;
}
static USBH_StatusTypeDef FTDI_BgndProcess(USBH_HandleTypeDef *phost) {
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState == HOST_CLASS && !phost->device.is_disconnected) {
      if (ftdi_rx_state == 0) {
          USBH_BulkReceiveData(phost, FTDI_RxBuffer, FTDI->InEpSize, FTDI->InPipe);
          ftdi_rx_state = 1;
      } else {
          USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, FTDI->InPipe);
          if (urb == USBH_URB_DONE) {
              uint32_t len = USBH_LL_GetLastXferSize(phost, FTDI->InPipe);
              if (len > 2) {
                  // FTDI has 2 modem status bytes at [0] and [1]
                  ProcessSerialRx(&FTDI_RxBuffer[2], (uint16_t)(len - 2));
              }
              USBH_BulkReceiveData(phost, FTDI_RxBuffer, FTDI->InEpSize, FTDI->InPipe);
              ftdi_rx_state = 1;
          } else if (urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
              ftdi_rx_state = 0;
          }
      }
  } else {
      ftdi_rx_state = 0;
  }
  return USBH_OK;
}
static USBH_StatusTypeDef FTDI_SOFProcess(USBH_HandleTypeDef *phost) { return USBH_OK; }
USBH_ClassTypeDef FTDI_Class = { "FTDI_Class", 0xFF, FTDI_Init, FTDI_DeInit, FTDI_Requests, FTDI_BgndProcess, FTDI_SOFProcess, NULL };
USBH_StatusTypeDef USBH_FTDI_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, FTDI->OutPipe); if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, FTDI->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// CH34X DRIVER
// ==========================================
typedef enum { CH34X_INIT_SETUP, CH34X_INIT_BAUD, CH34X_INIT_DONE } CH34x_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; CH34x_State state; } CH34x_HandleTypeDef;
static CH34x_HandleTypeDef CH34x_Handle;
static USBH_StatusTypeDef CH34x_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x1A86) return USBH_FAIL;
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0x01, 0x02);
  if (interface == 0xFF) interface = 0;
  USBH_SelectInterface(phost, interface);
  phost->pActiveClass->pData = &CH34x_Handle;
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  for (uint8_t i = 0; i < phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints; i++) {
    if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress & 0x80) {
      CH34x->InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      CH34x->InEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    } else {
      CH34x->OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      CH34x->OutEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    }
  }
  CH34x->OutPipe = USBH_AllocPipe(phost, CH34x->OutEp); CH34x->InPipe  = USBH_AllocPipe(phost, CH34x->InEp);
  USBH_OpenPipe(phost, CH34x->OutPipe, CH34x->OutEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, CH34x->OutEpSize);
  USBH_OpenPipe(phost, CH34x->InPipe, CH34x->InEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, CH34x->InEpSize);
  USBH_LL_SetToggle(phost, CH34x->OutPipe, 0); USBH_LL_SetToggle(phost, CH34x->InPipe, 0);
  CH34x->state = CH34X_INIT_SETUP; return USBH_OK;
}
static uint8_t ch34x_rx_buf[64];
static uint8_t ch34x_rx_state = 0;

static USBH_StatusTypeDef CH34x_DeInit(USBH_HandleTypeDef *phost) {
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  if (CH34x->OutPipe) { USBH_ClosePipe(phost, CH34x->OutPipe); USBH_FreePipe(phost, CH34x->OutPipe); }
  if (CH34x->InPipe)  { USBH_ClosePipe(phost, CH34x->InPipe);  USBH_FreePipe(phost, CH34x->InPipe);  }
  ch34x_rx_state = 0;
  return USBH_OK;
}
static USBH_StatusTypeDef CH34x_CtlReq(USBH_HandleTypeDef *phost, uint8_t req, uint16_t value, uint16_t index) {
  phost->Control.setup.b.bmRequestType = USB_H2D | USB_REQ_TYPE_VENDOR | USB_REQ_RECIPIENT_DEVICE;
  phost->Control.setup.b.bRequest = req; phost->Control.setup.b.wValue.w = value;
  phost->Control.setup.b.wIndex.w = index; phost->Control.setup.b.wLength.w = 0;
  return USBH_CtlReq(phost, NULL, 0);
}
static USBH_StatusTypeDef CH34x_Requests(USBH_HandleTypeDef *phost) {
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  switch (CH34x->state) {
    case CH34X_INIT_SETUP: if (CH34x_CtlReq(phost, 0xA1, 0, 0) == USBH_OK) CH34x->state = CH34X_INIT_BAUD; break;
    case CH34X_INIT_BAUD:  if (CH34x_CtlReq(phost, 0x9A, 0x1312, 0xCC1A) == USBH_OK) CH34x->state = CH34X_INIT_DONE; break;
    case CH34X_INIT_DONE:  return USBH_OK;
  }
  return USBH_BUSY;
}
static USBH_StatusTypeDef CH34x_BgndProcess(USBH_HandleTypeDef *phost) {
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState == HOST_CLASS && !phost->device.is_disconnected) {
    if (ch34x_rx_state == 0) {
      USBH_BulkReceiveData(phost, ch34x_rx_buf, CH34x->InEpSize, CH34x->InPipe);
      ch34x_rx_state = 1;
    } else {
      USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CH34x->InPipe);
      if (urb == USBH_URB_DONE) {
        uint32_t len = USBH_LL_GetLastXferSize(phost, CH34x->InPipe);
        if (len > 0) {
          ProcessSerialRx(ch34x_rx_buf, (uint16_t)len);
        }
        USBH_BulkReceiveData(phost, ch34x_rx_buf, CH34x->InEpSize, CH34x->InPipe);
        ch34x_rx_state = 1;
      } else if (urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
        ch34x_rx_state = 0;
      }
    }
  } else {
    ch34x_rx_state = 0;
  }
  return USBH_OK;
}
static USBH_StatusTypeDef CH34x_SOFProcess(USBH_HandleTypeDef *phost) { return USBH_OK; }
USBH_ClassTypeDef CH34x_Class = { "CH34x_Class", 0xFF, CH34x_Init, CH34x_DeInit, CH34x_Requests, CH34x_BgndProcess, CH34x_SOFProcess, NULL };
USBH_StatusTypeDef USBH_CH34x_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CH34x->OutPipe); if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, CH34x->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// PL2303 DRIVER
// ==========================================
typedef enum { PL2303_INIT_SETUP, PL2303_INIT_DONE } PL2303_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; PL2303_State state; } PL2303_HandleTypeDef;
static PL2303_HandleTypeDef PL2303_Handle;
static USBH_StatusTypeDef PL2303_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x067B) return USBH_FAIL;
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0x00, 0x00);
  if (interface == 0xFF) interface = 0;
  USBH_SelectInterface(phost, interface);
  phost->pActiveClass->pData = &PL2303_Handle;
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  for (uint8_t i = 0; i < phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints; i++) {
    if (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress & 0x80) {
      PL2303->InEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      PL2303->InEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    } else {
      PL2303->OutEp = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].bEndpointAddress;
      PL2303->OutEpSize = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[i].wMaxPacketSize;
    }
  }
  PL2303->OutPipe = USBH_AllocPipe(phost, PL2303->OutEp); PL2303->InPipe  = USBH_AllocPipe(phost, PL2303->InEp);
  USBH_OpenPipe(phost, PL2303->OutPipe, PL2303->OutEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, PL2303->OutEpSize);
  USBH_OpenPipe(phost, PL2303->InPipe, PL2303->InEp, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, PL2303->InEpSize);
  USBH_LL_SetToggle(phost, PL2303->OutPipe, 0); USBH_LL_SetToggle(phost, PL2303->InPipe, 0);
  PL2303->state = PL2303_INIT_SETUP; return USBH_OK;
}
static uint8_t pl2303_rx_buf[64];
static uint8_t pl2303_rx_state = 0;

static USBH_StatusTypeDef PL2303_DeInit(USBH_HandleTypeDef *phost) {
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  if (PL2303->OutPipe) { USBH_ClosePipe(phost, PL2303->OutPipe); USBH_FreePipe(phost, PL2303->OutPipe); }
  if (PL2303->InPipe)  { USBH_ClosePipe(phost, PL2303->InPipe);  USBH_FreePipe(phost, PL2303->InPipe);  }
  pl2303_rx_state = 0;
  return USBH_OK;
}
static USBH_StatusTypeDef PL2303_Requests(USBH_HandleTypeDef *phost) {
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  switch (PL2303->state) {
    case PL2303_INIT_SETUP: {
        static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08}; // 115200 8N1
        phost->Control.setup.b.bmRequestType = 0x21; phost->Control.setup.b.bRequest = 0x20;
        phost->Control.setup.b.wValue.w = 0; phost->Control.setup.b.wIndex.w = 0; phost->Control.setup.b.wLength.w = 7;
        if (USBH_CtlReq(phost, line_coding, 7) == USBH_OK) PL2303->state = PL2303_INIT_DONE;
      } break;
    case PL2303_INIT_DONE: return USBH_OK;
  }
  return USBH_BUSY;
}
static USBH_StatusTypeDef PL2303_BgndProcess(USBH_HandleTypeDef *phost) {
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState == HOST_CLASS && !phost->device.is_disconnected) {
    if (pl2303_rx_state == 0) {
      USBH_BulkReceiveData(phost, pl2303_rx_buf, PL2303->InEpSize, PL2303->InPipe);
      pl2303_rx_state = 1;
    } else {
      USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, PL2303->InPipe);
      if (urb == USBH_URB_DONE) {
        uint32_t len = USBH_LL_GetLastXferSize(phost, PL2303->InPipe);
        if (len > 0) {
          ProcessSerialRx(pl2303_rx_buf, (uint16_t)len);
        }
        USBH_BulkReceiveData(phost, pl2303_rx_buf, PL2303->InEpSize, PL2303->InPipe);
        pl2303_rx_state = 1;
      } else if (urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
        pl2303_rx_state = 0;
      }
    }
  } else {
    pl2303_rx_state = 0;
  }
  return USBH_OK;
}
static USBH_StatusTypeDef PL2303_SOFProcess(USBH_HandleTypeDef *phost) { return USBH_OK; }
USBH_ClassTypeDef PL2303_Class = { "PL2303_Class", 0xFF, PL2303_Init, PL2303_DeInit, PL2303_Requests, PL2303_BgndProcess, PL2303_SOFProcess, NULL };
USBH_StatusTypeDef USBH_PL2303_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, PL2303->OutPipe); if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, PL2303->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// UNIFIED WRAPPER
// ==========================================

USBH_StatusTypeDef USBH_Serial_RegisterClasses(USBH_HandleTypeDef *phost) {
    if (USBH_RegisterClass(phost, USBH_CDC_CLASS) != USBH_OK) return USBH_FAIL;
    if (USBH_RegisterClass(phost, &CP210x_Class) != USBH_OK) return USBH_FAIL;
    if (USBH_RegisterClass(phost, &FTDI_Class) != USBH_OK) return USBH_FAIL;
    if (USBH_RegisterClass(phost, &CH34x_Class) != USBH_OK) return USBH_FAIL;
    if (USBH_RegisterClass(phost, &PL2303_Class) != USBH_OK) return USBH_FAIL;
    return USBH_OK;
}

USBH_StatusTypeDef USBH_Serial_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
    if (phost->gState != HOST_CLASS) return USBH_FAIL;
    if (phost->pActiveClass == USBH_CDC_CLASS) return USBH_CDC_Transmit(phost, pbuff, length);
    else if (phost->pActiveClass == &CP210x_Class) return USBH_CP210x_Transmit(phost, pbuff, length);
    else if (phost->pActiveClass == &FTDI_Class) return USBH_FTDI_Transmit(phost, pbuff, length);
    else if (phost->pActiveClass == &CH34x_Class) return USBH_CH34x_Transmit(phost, pbuff, length);
    else if (phost->pActiveClass == &PL2303_Class) return USBH_PL2303_Transmit(phost, pbuff, length);
    return USBH_FAIL;
}

int USBH_Serial_ReadLine(USBH_HandleTypeDef *phost, char *buf, uint16_t max_len) {
    if (phost != NULL && phost->gState == HOST_CLASS && phost->pActiveClass == USBH_CDC_CLASS) {
        if (!cdc_rx_started) {
            if (USBH_CDC_Receive(phost, cdc_rx_buf, sizeof(cdc_rx_buf)) == USBH_OK) {
                cdc_rx_started = 1;
            }
        }
    } else if (phost != NULL && phost->gState != HOST_CLASS) {
        cdc_rx_started = 0;
    }

    if (datafull_ready) {
        datafull_ready = 0;
        if (buf != NULL && max_len > 0) {
            strncpy(buf, datafull, max_len - 1);
            buf[max_len - 1] = '\0';
            return (int)strlen(buf);
        }
        return 1;
    }
    return 0;
}


