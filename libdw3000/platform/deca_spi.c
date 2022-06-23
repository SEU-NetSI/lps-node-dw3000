/*! ----------------------------------------------------------------------------
 * @file    deca_spi.c
 * @brief   SPI access functions
 *
 * @attention
 *
 * Copyright 2015-2020 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */

#include <deca_spi.h>
#include <deca_device_api.h>
#include <port_dw3000.h>
#include <stm32f0xx_hal.h>
// #include <stm32f4xx_hal_def.h>

extern  SPI_HandleTypeDef hspi1;    /*clocked from 72MHz*/
#define DW_NSS_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_4

/****************************************************************************//**
 *
 *                              DW1000 SPI section
 *
 *******************************************************************************/
/*! ------------------------------------------------------------------------------------------------------------------
 * Function: openspi()
 *
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
} // end openspi()

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: closespi()
 *
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
} // end closespi()

// Aligned buffer of 128bytes
// This is used as a "scratch" buffer to the SPI transfers
// The problem is that the Cortex-m0 only supports 2Bytes-aligned memory access
uint16_t alignedBuffer[64];

static void spiWrite(const void *header, size_t headerLength,
                                      const void* data, size_t dataLength)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  memcpy(alignedBuffer, header, headerLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
  memcpy(alignedBuffer, data, dataLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

static void spiRead(const void *header, size_t headerLength,
                                     void* data, size_t dataLength)
{
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  memcpy(alignedBuffer, header, headerLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);
  memcpy(data, alignedBuffer, dataLength);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospiwithcrc()
 *
 * Low level abstract function to write to the SPI when SPI CRC mode is used
 * Takes two separate byte buffers for write header and write data, and a CRC8 byte which is written last
 * returns 0 for success, or -1 for error
 */
int writetospiwithcrc(
                uint16_t      headerLength,
                const uint8_t *headerBuffer,
                uint16_t      bodyLength,
                const uint8_t *bodyBuffer,
                uint8_t       crc8)
{
    while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */

    HAL_SPI_Transmit(&hspi1, (uint8_t *)headerBuffer, headerLength, 10);    /* Send header in polling mode */
    HAL_SPI_Transmit(&hspi1, (uint8_t *)bodyBuffer, bodyLength, 10);        /* Send data in polling mode */
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&crc8, 1, 10);      /* Send data in polling mode */

    HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */
    return 0;
} // end writetospiwithcrc()


/*! ------------------------------------------------------------------------------------------------------------------
 * Function: writetospi()
 *
 * Low level abstract function to write to the SPI
 * Takes two separate byte buffers for write header and write data
 * returns 0 for success, or -1 for error
 */
int writetospi(uint16_t       headerLength,
               const uint8_t  *headerBuffer,
               uint16_t       bodyLength,
               const uint8_t  *bodyBuffer)
{
    spiWrite(headerBuffer, headerLength, bodyBuffer, bodyLength);
    return 0;
} // end writetospi()



/*! ------------------------------------------------------------------------------------------------------------------
* @fn spi_cs_low_delay()
*
* @brief This function sets the CS to '0' for ms delay and than raises it up
*
* input parameters:
* @param ms_delay - The delay for CS to be in '0' state
*
* no return value
*/
uint16_t spi_cs_low_delay(uint16_t delay_ms)
{
	/* Blocking: Check whether previous transfer has been finished */
	while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	/* Process Locked */
	__HAL_LOCK(&hspi1);
	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET); /**< Put chip select line low */
	Sleep(delay_ms);
	HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET); /**< Put chip select line high */
	/* Process Unlocked */
	__HAL_UNLOCK(&hspi1);

	return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * Function: readfromspi()
 *
 * Low level abstract function to read from the SPI
 * Takes two separate byte buffers for write header and read data
 * returns the offset into read buffer where first byte of read data may be found,
 * or returns -1 if there was an error
 */
//#pragma GCC optimize ("O3")
int readfromspi(uint16_t  headerLength,
                uint8_t   *headerBuffer,
                uint16_t  readlength,
                uint8_t   *readBuffer)
{   
    spiRead(headerBuffer, headerLength, readBuffer, readlength);
    return 0;
} // end readfromspi()

/****************************************************************************//**
 *
 *                              END OF DW1000 SPI section
 *
 *******************************************************************************/


