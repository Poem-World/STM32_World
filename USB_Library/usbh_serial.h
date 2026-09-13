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

// Call this before USB device connection (e.g., at boot after reading EEPROM)
// baudrate: e.g. 115200, 9600
// databits: 5, 6, 7, 8
// parity: 0:None, 1:Odd, 2:Even, 3:Mark, 4:Space
// stopbits: 0:1 bit, 1:1.5 bits, 2:2 bits
void USBH_Serial_InitSettings(uint32_t baudrate, uint8_t databits, uint8_t parity, uint8_t stopbits);

#define USBH_SERIAL_RX_LINE_SIZE 256
extern char datafull[USBH_SERIAL_RX_LINE_SIZE];
extern volatile uint8_t datafull_ready;

// Checks and reads a complete line ending with '\n' received from USB Serial (CDC/CP210x/FTDI/etc.)
// Returns the string length if a line is ready, or 0 if not ready.
int USBH_Serial_ReadLine(USBH_HandleTypeDef *phost, char *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif
