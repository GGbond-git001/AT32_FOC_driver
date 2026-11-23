#include "bsp_spi.h"

void BSP_SPI_Init(void)
{
    BSP_SPI3_CS_High();
    spi_enable(SPI3, TRUE);
}

/**
 * @brief  SPI3 8-bit 寄存器级收发
 */
uint8_t BSP_SPI3_ReadWrite_Byte(uint8_t tx_data)
{
    uint32_t timeout = 500U;
    while ((SPI3->sts_bit.tdbe == RESET) && (--timeout));
    SPI3->dt = tx_data;

    timeout = 500U;
    while ((SPI3->sts_bit.rdbf == RESET) && (--timeout));
    return (uint8_t)(SPI3->dt);
}

/**
 * @brief  采用 MT6816 连续突发读模式(Burst Read)读取 0x03 与 0x04 寄存器
 * @return 拼接后的 16 位原始帧: [15:2]=14位角度, [1]=弱磁警告, [0]=偶校验位
 */
uint16_t BSP_MT6816_Read_Raw16(void)
{
    BSP_SPI3_CS_Low();

    /* 第1字节: 发送读 0x03 寄存器命令 (0x80 | 0x03 = 0x83) */
    (void)BSP_SPI3_ReadWrite_Byte(0x83U);

    /* 第2、3字节: 连续读取 Reg 0x03 (高8位) 和 Reg 0x04 (低8位) */
    uint8_t reg_03 = BSP_SPI3_ReadWrite_Byte(0x00U);
    uint8_t reg_04 = BSP_SPI3_ReadWrite_Byte(0x00U);

    /* 等待总线完全空闲后再拉高 CSN */
    uint32_t timeout = 500U;
    while ((SPI3->sts_bit.bf != RESET) && (--timeout));
    BSP_SPI3_CS_High();

    return ((uint16_t)reg_03 << 8) | (uint16_t)reg_04;
}
