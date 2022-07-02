#include "uwb.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stm32f0xx_hal.h>
#include <stdbool.h>
#include <stdio.h>
#include "port_dw3000.h"
#include <string.h>
#include <stdlib.h>
#include "ranging.h"

static bool isInit = false;
static SemaphoreHandle_t irqSemaphore;

static QueueHandle_t txQueue;
static StaticQueue_t txQueueBuffer;
static uint8_t txQueueStorage[TX_QUEUE_SIZE * TX_ITEM_SIZE];

static QueueHandle_t rxQueue;
static StaticQueue_t rxQueueBuffer;
static uint8_t rxQueueStorage[RX_QUEUE_SIZE * RX_ITEM_SIZE];

/* rx buffer used in rx_callback */
static uint8_t rxBuffer[FRAME_LEN_MAX];

void queueInit()
{
    txQueue = xQueueCreateStatic(TX_QUEUE_SIZE, TX_ITEM_SIZE, txQueueStorage, &txQueueBuffer);
    rxQueue = xQueueCreateStatic(RX_QUEUE_SIZE, RX_ITEM_SIZE, rxQueueStorage, &rxQueueBuffer);
}

void uwbInit()
{
    printf("uwbInit");
    queueInit();
    static StaticSemaphore_t irqSemaphoreBuffer;
    irqSemaphore = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

    isInit = true;

    port_set_dw_ic_spi_fastrate();
    reset_DWIC();
    Sleep(2);
    while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before proceeding */
    {
        printf("error\r\n");
    };
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR)
    {
        isInit = false;
        return;
    }
    if (dwt_configure(&config) == DWT_ERROR)
    {
        isInit = false;
        return;
    }
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);

    /* Configure the TX spectrum parameters (power, PG delay and PG count) */
    dwt_configuretxrf(&txconfig_options);
    /* Auto re-enable receiver after a frame reception failure (except a frame wait timeout), the receiver will re-enable to re-attempt reception.*/
    dwt_or32bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXAUTR_BIT_MASK);
    dwt_setrxtimeout(UWB_RX_TIMEOUT);

    dwt_setcallbacks(&tx_cb, &rx_cb, &rx_to_cb, &rx_err_cb, NULL, NULL);
    /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
    dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                         SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                         SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK | SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                     0, DWT_ENABLE_INT);

    /* Clearing the SPI ready interrupt */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    isInit = true;
}

static int checkIrq()
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
}

static void uwbTask(void *parameters)
{
    while (!isInit)
    {
        printf("false");
        vTaskDelay(1000);
    }
    while (true)
    {
        if (xSemaphoreTake(irqSemaphore, portMAX_DELAY))
        {
            do
            {
                dwt_isr();
            } while (checkIrq() != 0);
        }
    }
}

static void uwbTxTask(void *parameters)
{
    while (!isInit)
    {
        printf("false");
        vTaskDelay(1000);
    }

    Mock_Packet packetCache;

    while (true)
    {
        if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY))
        {
            dwt_forcetrxoff();
            dwt_writetxdata(sizeof(packetCache) - FCS_LEN, &packetCache, 0);
            dwt_writetxfctrl(sizeof(packetCache), 0, 0);
            /* Start transmission. */
            if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) == DWT_ERROR)
            {
                printf("uwbTxTask:  TX ERROR\r\n");
            }
            printf("uwbTxTask: TX seq: %d \r\n", packetCache.header.seqNumber);
        }
    }
}

static void uwbRxTask(void *parameters)
{
    while (!isInit)
    {
        printf("false");
        vTaskDelay(1000);
    }

    Mock_Packet packetCache;

    while (true)
    {
        if (xQueueReceive(rxQueue, &packetCache, portMAX_DELAY))
        {
            printf("uwbRxTask: RX seq: %d \r\n", packetCache.header.seqNumber);
        }
    }
}

static void uwbPeriodSendTask(void *parameters)
{
    while (!isInit)
    {
        printf("false");
        vTaskDelay(1000);
    }
    int seqNumber = 0;
    Mock_Packet packetCache;

    while (true)
    {
        Mock_Packet packetCache;
        packetCache.header.seqNumber = seqNumber++;
        xQueueSend(txQueue, &packetCache, portMAX_DELAY);
        vTaskDelay(TX_PERIOD_IN_MS);
    }
}

void uwbStart()
{
    printf("uwbStart");
    static StaticTask_t uwbStaticTask;
    static StackType_t uwbStaticStack[configMINIMAL_STACK_SIZE];

    xTaskCreateStatic(uwbTask, "uwbTask", configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, uwbStaticStack, &uwbStaticTask);

    static StaticTask_t uwbTxStaticTask;
    static StackType_t uwbTxStaticStack[2 * configMINIMAL_STACK_SIZE];

    xTaskCreateStatic(uwbTxTask, "uwbTxTask", 2 * configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, uwbTxStaticStack, &uwbTxStaticTask);

    static StaticTask_t uwbRxStaticTask;
    static StackType_t uwbRxStaticStack[2 * configMINIMAL_STACK_SIZE];

    xTaskCreateStatic(uwbRxTask, "uwbRxTask", 2 * configMINIMAL_STACK_SIZE, NULL,
                      configMAX_PRIORITIES - 1, uwbRxStaticStack, &uwbRxStaticTask);

    rangingInit();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // printf("HAL_GPIO_EXTI_Callback\n");
    BaseType_t higherPriorityTaskWoken = pdFALSE;

    switch (GPIO_Pin)
    {
    case GPIO_PIN_0:
        xSemaphoreGiveFromISR(irqSemaphore, &higherPriorityTaskWoken);
        HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
        break;
    default:
        break;
    }
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void rx_cb()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
    if (dataLength != 0 && dataLength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */
    }
    xQueueSendFromISR(rxQueue, &rxBuffer, &xHigherPriorityTaskWoken);
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // printf("rx_cb\r\n");
}

void tx_cb()
{
    // printf("tx_cb\r\n");
}

void rx_to_cb()
{
    dwt_forcetrxoff();
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    // printf("rx_to_cb\r\n");
}

void rx_err_cb()
{
    printf("rx_err_cb\r\n");
}