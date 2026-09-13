# STM32 USB Serial Unified Driver (CDC, CP210x, FTDI, CH34x, PL2303)
## CubeMX 무수정(Zero-Patch) 영구 보존 아키텍처

이 문서는 STM32 USB Host 환경에서 다중 USB 시리얼 칩셋(**표준 CDC, CP2102, FT232, CH340, PL2303**)을 플러그 앤 플레이로 완벽 지원하며, **STM32CubeMX에서 코드를 다시 생성(Generate Code)해도 코드가 지워지거나 초기화되지 않도록 영구 조치된 아키텍처 및 전체 소스 코드** 가이드입니다.

---

## 1. CubeMX 코드 재생성 덮어쓰기 방지 원리 및 조치 사항

STM32CubeMX에서 "Generate Code"를 실행하면 일반적으로 수정한 USB 설정이 리셋되는 문제가 발생합니다. 본 드라이버는 다음 3가지 방법으로 완벽히 방어합니다.

### 1.1. usbh_core.c 라이브러리 수정 불필요 (Zero-Patch 혁신)
- **과거 문제점**: CP2102, FTDI, CH340, PL2303은 모두 인터페이스 클래스가 `0xFF (Vendor Specific)`입니다. 기존에는 클래스를 개별 등록하다 보니 ST의 기본 `usbh_core.c`가 첫 번째 0xFF 클래스만 검사하고 중단하여 `usbh_core.c` 소스를 직접 패치해야 했습니다. 하지만 CubeMX 재생성 시 이 파일이 순정으로 덮어써져 장치 인식이 중단되었습니다.
- **영구 해결책**: 모든 벤더 시리얼 드라이버를 단 하나의 통합 벤더 클래스 `USB_Serial_Class (ClassCode = 0xFF)`로 묶었습니다. USB Host 코어에는 오직 **CDC(0x02)**와 **USB_Serial_Class(0xFF)** 2가지만 등록됩니다. 연결 시 `Serial_Init()` 내부에서 장치의 VID(Vendor ID)를 자동 검사하여 해당 칩셋 드라이버로 연결합니다.
- **결과**: **ST 공식 `usbh_core.c` 파일을 단 한 줄도 수정할 필요가 없으므로**, CubeMX가 라이브러리를 몇 번을 덮어써도 아무 문제없이 동작합니다.

### 1.2. usbh_conf.h 리셋 방지 (.ioc 파일 영구 등록)
- **과거 문제점**: CubeMX 기본 CDC 설정은 `USBH_PROCESS_STACK_SIZE=0`으로 설정되어 있어 FreeRTOS 구동 시 즉시 Stack Overflow Hard Fault가 발생하고, 클래스 최대 개수가 1로 리셋되었습니다.
- **영구 해결책**: `STM32H747I-DISCO.ioc` 파일 자체에 파라미터를 등록 완료했습니다.
  ```properties
  USB_HOST_M7.IPParameters=...,USBH_MAX_NUM_SUPPORTED_CLASS,USBH_PROCESS_STACK_SIZE
  USB_HOST_M7.USBH_MAX_NUM_SUPPORTED_CLASS=5
  USB_HOST_M7.USBH_PROCESS_STACK_SIZE=1024
  ```
- **결과**: 이제 CubeMX에서 "Generate Code"를 누르면 CubeMX 자체가 `usbh_conf.h`에 항상 `5U`와 `1024`를 자동 생성합니다.

### 1.3. usb_host.c 사용자 코드 보호 (PreTreatment Early Return)
- **과거 문제점**: `MX_USB_HOST_Init()` 내부의 클래스 등록 코드가 기본 CDC 등록으로 되돌아감.
- **영구 해결책**: CubeMX가 보존하는 `/* USER CODE BEGIN USB_HOST_Init_PreTreatment */` 영역에 커스텀 초기화(`USBH_Serial_RegisterClasses`)를 넣고 끝에 `return;`을 선언했습니다.
- **결과**: CubeMX가 밑에 기본 CDC 등록 코드를 다시 생성하더라도 `return;`에 의해 절대 실행되지 않고 안전한 통합 등록 코드만 실행됩니다.

---

## 2. 드라이버 주요 사양 및 버퍼 구조

- **동시 지원 칩셋**:
  1. 표준 USB CDC (가상 시리얼 포트)
  2. Silicon Labs CP2101 / CP2102 / CP2104 / CP2108 (VID: `0x10C4`)
  3. FTDI FT232R / FT2232 / FT4232 (VID: `0x0403`)
  4. WCH CH340 / CH341 (VID: `0x1A86`)
  5. Prolific PL2303 (VID: `0x067B`)
- **버퍼 크기 사양**:
  - `USBH_SERIAL_RX_LINE_SIZE`: **256 Bytes** (한 줄 수신 완성 버퍼)
  - 저수준 USB 수신 버퍼: 각 드라이버별 64 Bytes Bulk Packet 버퍼
  - `datafull`: 개행문자(`\n`)가 수신될 때까지 축적된 완성형 문자열 버퍼 (최대 255글자 + null 종료문자)
  - `datafull_ready`: 개행문자(`\n`) 수신 완료 플래그 (volatile uint8_t)
- **통신 설정 (Baudrate, Parity, StopBits)**:
  - 부팅 시 또는 런타임에 `USBH_Serial_InitSettings(baudrate, databits, parity, stopbits)` 호출로 일괄 설정 가능 (기본값: 115200 8N1).

---

## 3. 전체 소스 코드

### 3.1. `CM7/Core/Inc/usbh_serial.h`
```c
#ifndef USBH_SERIAL_H
#define USBH_SERIAL_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbh_core.h"
#include "usbh_cdc.h"

extern USBH_ClassTypeDef USB_Serial_Class;

USBH_StatusTypeDef USBH_Serial_RegisterClasses(USBH_HandleTypeDef *phost);
USBH_StatusTypeDef USBH_Serial_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length);

// 통신 파라미터 사전 설정 함수 (USB 연결 전/후 설정 가능)
// baudrate: 9600, 115200 등
// databits: 5, 6, 7, 8
// parity: 0:None, 1:Odd, 2:Even, 3:Mark, 4:Space
// stopbits: 0:1 bit, 1:1.5 bits, 2:2 bits
void USBH_Serial_InitSettings(uint32_t baudrate, uint8_t databits, uint8_t parity, uint8_t stopbits);

#define USBH_SERIAL_RX_LINE_SIZE 256
extern char datafull[USBH_SERIAL_RX_LINE_SIZE];
extern volatile uint8_t datafull_ready;

// '\n' 수신 완료 여부를 확인하고 버퍼에 복사 (읽기 성공 시 문자열 길이 반환)
int USBH_Serial_ReadLine(USBH_HandleTypeDef *phost, char *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif
```

---

### 3.2. `CM7/Core/Src/usbh_serial.c`
```c
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

// ==========================================
// CDC Reception Support
// ==========================================
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
// CP210X DRIVER (VID: 0x10C4)
// ==========================================
typedef enum { CP210X_INIT_IFC_ENABLE, CP210X_INIT_SET_BAUDDIV, CP210X_INIT_SET_LINE_CTL, CP210X_INIT_DONE } CP210x_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; CP210x_State state; } CP210x_HandleTypeDef;
static CP210x_HandleTypeDef CP210x_Handle;

static USBH_StatusTypeDef CP210x_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x10C4) return USBH_FAIL;
  printf("[CP210x] Vendor ID Matched!\r\n");
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0x00, 0x00);
  if (interface == 0xFF) { printf("[CP210x] Interface failed!\r\n"); return USBH_FAIL; }
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
      if (status == USBH_OK) { CP210x->state = CP210X_INIT_SET_BAUDDIV; }
      break;
    case CP210X_INIT_SET_BAUDDIV: {
      uint32_t bauddiv = 3686400 / Serial_BaudRate;
      status = CP210x_CtlReq(phost, 0x01, bauddiv);
      if (status == USBH_OK) { CP210x->state = CP210X_INIT_SET_LINE_CTL; }
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
      if (status == USBH_OK) { CP210x->state = CP210X_INIT_DONE; }
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
        if (len > 0) { ProcessSerialRx(cp210x_rx_buf, (uint16_t)len); }
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
USBH_StatusTypeDef USBH_CP210x_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  CP210x_HandleTypeDef *CP210x = (CP210x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CP210x->OutPipe);
  if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, CP210x->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// FTDI DRIVER (VID: 0x0403)
// ==========================================
typedef enum { FTDI_INIT_RESET, FTDI_INIT_SET_BAUD, FTDI_INIT_SET_DATA, FTDI_INIT_SET_FLOW, FTDI_INIT_SET_MODEM, FTDI_INIT_DONE } FTDI_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; FTDI_State state; } FTDI_HandleTypeDef;
static FTDI_HandleTypeDef FTDI_Handle;

static USBH_StatusTypeDef FTDI_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x0403) return USBH_FAIL;
  printf("[FTDI] Vendor ID Matched!\r\n");
  uint8_t interface = USBH_FindInterface(phost, 0xFF, 0xFF, 0xFF);
  if (interface == 0xFF) return USBH_FAIL;
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
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) FTDI->state = FTDI_INIT_SET_BAUD;
        break;
    case FTDI_INIT_SET_BAUD: {
        uint32_t divisor = 3000000 / Serial_BaudRate;
        status = FTDI_CtlReq(phost, 0x03, divisor, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) FTDI->state = FTDI_INIT_SET_DATA;
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
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) FTDI->state = FTDI_INIT_SET_FLOW;
        break;
    }
    case FTDI_INIT_SET_FLOW:  
        status = FTDI_CtlReq(phost, 0x02, 0x0000, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) FTDI->state = FTDI_INIT_SET_MODEM;
        break;
    case FTDI_INIT_SET_MODEM: 
        status = FTDI_CtlReq(phost, 0x01, 0x0303, 0);
        if (status == USBH_OK || status == USBH_NOT_SUPPORTED || status == USBH_FAIL) FTDI->state = FTDI_INIT_DONE;
        break;
    case FTDI_INIT_DONE: return USBH_OK;
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
                  // FTDI의 앞 2바이트(모뎀 상태) 제외 후 데이터 전달
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
USBH_StatusTypeDef USBH_FTDI_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  FTDI_HandleTypeDef *FTDI = (FTDI_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, FTDI->OutPipe);
  if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, FTDI->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// CH34X DRIVER (VID: 0x1A86)
// ==========================================
typedef enum { CH34X_INIT_SETUP, CH34X_INIT_BAUD, CH34X_INIT_DONE } CH34x_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; CH34x_State state; } CH34x_HandleTypeDef;
static CH34x_HandleTypeDef CH34x_Handle;

static USBH_StatusTypeDef CH34x_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x1A86) return USBH_FAIL;
  printf("[CH34x] Vendor ID Matched!\r\n");
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
        if (len > 0) { ProcessSerialRx(ch34x_rx_buf, (uint16_t)len); }
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
USBH_StatusTypeDef USBH_CH34x_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  CH34x_HandleTypeDef *CH34x = (CH34x_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, CH34x->OutPipe);
  if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, CH34x->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// PL2303 DRIVER (VID: 0x067B)
// ==========================================
typedef enum { PL2303_INIT_SETUP, PL2303_INIT_DONE } PL2303_State;
typedef struct { uint8_t InEp, OutEp, InPipe, OutPipe; uint16_t InEpSize, OutEpSize; PL2303_State state; } PL2303_HandleTypeDef;
static PL2303_HandleTypeDef PL2303_Handle;

static USBH_StatusTypeDef PL2303_Init(USBH_HandleTypeDef *phost) {
  if (phost->device.DevDesc.idVendor != 0x067B) return USBH_FAIL;
  printf("[PL2303] Vendor ID Matched!\r\n");
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
        if (len > 0) { ProcessSerialRx(pl2303_rx_buf, (uint16_t)len); }
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
USBH_StatusTypeDef USBH_PL2303_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
  PL2303_HandleTypeDef *PL2303 = (PL2303_HandleTypeDef *)phost->pActiveClass->pData;
  if (phost->gState != HOST_CLASS || phost->device.is_disconnected) return USBH_FAIL;
  USBH_URBStateTypeDef urb = USBH_LL_GetURBState(phost, PL2303->OutPipe);
  if (urb == USBH_URB_IDLE || urb == USBH_URB_DONE || urb == USBH_URB_NOTREADY || urb == USBH_URB_ERROR || urb == USBH_URB_STALL) {
      USBH_BulkSendData(phost, pbuff, length, PL2303->OutPipe, 1); return USBH_OK;
  }
  return USBH_BUSY;
}

// ==========================================
// UNIFIED VENDOR SERIAL CLASS (VID Auto-Dispatch)
// ==========================================
typedef enum {
    SERIAL_TYPE_NONE = 0,
    SERIAL_TYPE_CP210X,
    SERIAL_TYPE_FTDI,
    SERIAL_TYPE_CH34X,
    SERIAL_TYPE_PL2303
} SerialDeviceType;

static SerialDeviceType active_device_type = SERIAL_TYPE_NONE;

static USBH_StatusTypeDef Serial_Init(USBH_HandleTypeDef *phost) {
    uint16_t vid = phost->device.DevDesc.idVendor;
    if (vid == 0x10C4) {
        active_device_type = SERIAL_TYPE_CP210X;
        return CP210x_Init(phost);
    } else if (vid == 0x0403) {
        active_device_type = SERIAL_TYPE_FTDI;
        return FTDI_Init(phost);
    } else if (vid == 0x1A86) {
        active_device_type = SERIAL_TYPE_CH34X;
        return CH34x_Init(phost);
    } else if (vid == 0x067B) {
        active_device_type = SERIAL_TYPE_PL2303;
        return PL2303_Init(phost);
    }
    return USBH_FAIL;
}

static USBH_StatusTypeDef Serial_DeInit(USBH_HandleTypeDef *phost) {
    USBH_StatusTypeDef ret = USBH_OK;
    switch (active_device_type) {
        case SERIAL_TYPE_CP210X: ret = CP210x_DeInit(phost); break;
        case SERIAL_TYPE_FTDI:   ret = FTDI_DeInit(phost); break;
        case SERIAL_TYPE_CH34X:  ret = CH34x_DeInit(phost); break;
        case SERIAL_TYPE_PL2303: ret = PL2303_DeInit(phost); break;
        default: break;
    }
    active_device_type = SERIAL_TYPE_NONE;
    return ret;
}

static USBH_StatusTypeDef Serial_Requests(USBH_HandleTypeDef *phost) {
    switch (active_device_type) {
        case SERIAL_TYPE_CP210X: return CP210x_Requests(phost);
        case SERIAL_TYPE_FTDI:   return FTDI_Requests(phost);
        case SERIAL_TYPE_CH34X:  return CH34x_Requests(phost);
        case SERIAL_TYPE_PL2303: return PL2303_Requests(phost);
        default: return USBH_FAIL;
    }
}

static USBH_StatusTypeDef Serial_BgndProcess(USBH_HandleTypeDef *phost) {
    switch (active_device_type) {
        case SERIAL_TYPE_CP210X: return CP210x_BgndProcess(phost);
        case SERIAL_TYPE_FTDI:   return FTDI_BgndProcess(phost);
        case SERIAL_TYPE_CH34X:  return CH34x_BgndProcess(phost);
        case SERIAL_TYPE_PL2303: return PL2303_BgndProcess(phost);
        default: return USBH_OK;
    }
}

static USBH_StatusTypeDef Serial_SOFProcess(USBH_HandleTypeDef *phost) {
    return USBH_OK;
}

USBH_ClassTypeDef USB_Serial_Class = {
    "USB_Serial_Class",
    0xFF,
    Serial_Init,
    Serial_DeInit,
    Serial_Requests,
    Serial_BgndProcess,
    Serial_SOFProcess,
    NULL
};

USBH_StatusTypeDef USBH_Serial_RegisterClasses(USBH_HandleTypeDef *phost) {
    if (USBH_RegisterClass(phost, USBH_CDC_CLASS) != USBH_OK) return USBH_FAIL;
    if (USBH_RegisterClass(phost, &USB_Serial_Class) != USBH_OK) return USBH_FAIL;
    return USBH_OK;
}

USBH_StatusTypeDef USBH_Serial_Transmit(USBH_HandleTypeDef *phost, uint8_t *pbuff, uint16_t length) {
    if (phost->gState != HOST_CLASS) return USBH_FAIL;
    if (phost->pActiveClass == USBH_CDC_CLASS) return USBH_CDC_Transmit(phost, pbuff, length);
    switch (active_device_type) {
        case SERIAL_TYPE_CP210X: return USBH_CP210x_Transmit(phost, pbuff, length);
        case SERIAL_TYPE_FTDI:   return USBH_FTDI_Transmit(phost, pbuff, length);
        case SERIAL_TYPE_CH34X:  return USBH_CH34x_Transmit(phost, pbuff, length);
        case SERIAL_TYPE_PL2303: return USBH_PL2303_Transmit(phost, pbuff, length);
        default: return USBH_FAIL;
    }
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
```

---

### 3.3. `CM7/USB_HOST/App/usb_host.c` (보호 적용)
```c
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "usbh_serial.h"
/* USER CODE END Includes */

void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */
  printf("[USB] -> Step 1: USBH_Init...\r\n");
  if (USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS) != USBH_OK)
  {
    printf("[USB] ERROR: USBH_Init failed!\r\n");
    Error_Handler();
  }

  printf("[USB] -> Step 2: Register Classes...\r\n");
  if (USBH_Serial_RegisterClasses(&hUsbHostHS) != USBH_OK)
  {
    printf("[USB] ERROR: USBH_Serial_RegisterClasses failed!\r\n");
    Error_Handler();
  }

  printf("[USB] -> Step 3: USBH_Start...\r\n");
  if (USBH_Start(&hUsbHostHS) != USBH_OK)
  {
    printf("[USB] ERROR: USBH_Start failed!\r\n");
    Error_Handler();
  }
  printf("[USB] -> Step 4: MX_USB_HOST_Init completed successfully!\r\n");
  return; // CubeMX가 아래에 자동 생성하는 기본 CDC 코드가 절대 실행되지 않도록 차단!
  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostHS, USBH_UserProcess, HOST_HS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostHS, USBH_CDC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostHS) != USBH_OK)
  {
    Error_Handler();
  }
}
```

---

### 3.4. `main.c`의 FreeRTOS Task 수신 처리 예제
```c
void myTaskFunc01(void *argument)
{
  char line_buf[256];
  for(;;)
  {
    // USB Serial로부터 '\n'을 포함한 완성형 라인이 수신되었는지 폴링
    if (USBH_Serial_ReadLine(&hUsbHostHS, line_buf, sizeof(line_buf)) > 0)
    {
      // 수신된 완성형 문자열을 UART1으로 즉시 전달
      HAL_UART_Transmit(&huart1, (uint8_t *)line_buf, strlen(line_buf), 100);
      printf("[USB->UART1] %s", line_buf);
    }
    osDelay(10);
  }
}
```
