#ifndef __BSP_SPI_H
#define __BSP_SPI_H

#include <stdint.h>
#include "at32f435_437.h"

#define SPI3_CS_PORT    GPIOA
#define SPI3_CS_PIN     GPIO_PINS_15

static inline void BSP_SPI3_CS_Low(void)  { SPI3_CS_PORT->clr = SPI3_CS_PIN; }
static inline void BSP_SPI3_CS_High(void) { SPI3_CS_PORT->scr = SPI3_CS_PIN; }

void BSP_SPI_Init(void);
uint8_t BSP_SPI3_ReadWrite_Byte(uint8_t tx_data);
uint16_t BSP_MT6816_Read_Raw16(void);

#endif
