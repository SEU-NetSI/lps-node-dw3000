#include <stm32f0xx_hal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "uwb.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "dwTypes.h"
#include "swarm_ranging.h"
#include "libdw3000.h"

extern dwOps_t dwt_ops;
static bool isInit = false;
static SemaphoreHandle_t irq_semphr;
static SemaphoreHandle_t ranging_set_lock;
static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_buffer;
static uint8_t tx_queue_storage[TX_QUEUE_SIZE * TX_QUEUE_ITEM_SIZE];

static QueueHandle_t txQueue;
static xQueueHandle queues[MESSAGE_TYPE_COUNT];
static UWB_Message_Listener_t listeners[MESSAGE_TYPE_COUNT];
static MESSAGE_TYPE TX_MESSAGE_TYPE;

static int packetSeqNumber;

/* rx buffer used in rx_callback */
static uint8_t rxBuffer[FRAME_LEN_MAX];

static void txCallback() {
  packetSeqNumber++;
  if (TX_MESSAGE_TYPE < MESSAGE_TYPE_COUNT && listeners[TX_MESSAGE_TYPE].txCb) {
    listeners[TX_MESSAGE_TYPE].txCb(NULL); // TODO no parameter passed into txCb now
  }
}

static void rxCallback() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  uint32_t dataLength = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (dataLength != 0 && dataLength <= FRAME_LEN_MAX) {
    dwt_readrxdata(rxBuffer, dataLength - FCS_LEN, 0); /* No need to read the FCS/CRC. */
  }
//  DEBUG_PRINT("rxCallback: data length = %lu \n", dataLength);

  UWB_Packet_t *packet = (UWB_Packet_t *) &rxBuffer;
  MESSAGE_TYPE msgType = packet->header.type;

  if (listeners[msgType].rxCb) {
    listeners[msgType].rxCb(packet);
  }

  if (listeners[msgType].rxQueue) {
    xQueueSendFromISR(listeners[msgType].rxQueue, packet, &xHigherPriorityTaskWoken);
  }

  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxTimeoutCallback() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

static void rxErrorCallback() {
  DEBUG_PRINT("rxErrorCallback: some error occurs when rx\n");
}

uint16_t getUWBAddress() {
  return MY_UWB_ADDRESS;
}

int uwbSendPacket(UWB_Packet_t *packet) {
  xQueueSend(txQueue, packet, 0);
}

int uwbSendPacketBlock(UWB_Packet_t *packet) {
  xQueueSend(txQueue, packet, portMAX_DELAY);
}

int uwbReceivePacket(MESSAGE_TYPE type, UWB_Packet_t *packet) {
  return xQueueReceive(queues[type], packet, 0);
}

int uwbReceivePacketBlock(MESSAGE_TYPE type, UWB_Packet_t *packet) {
  return xQueueReceive(queues[type], packet, portMAX_DELAY);
}

int uwbReceivePacketWait(MESSAGE_TYPE type, UWB_Packet_t *packet, int wait) {
  return xQueueReceive(queues[type], packet, M2T(wait));
}

void uwbRegisterListener(UWB_Message_Listener_t *listener) {
  queues[listener->type] = listener->rxQueue;
  listeners[listener->type] = *listener;
}

void queueInit() {
  txQueue = xQueueCreateStatic(TX_QUEUE_SIZE, TX_QUEUE_ITEM_SIZE, tx_queue_storage,
                                &tx_queue_buffer);
}
SPI_HandleTypeDef hspi1;
uint16_t alignedBuffer[FRAME_LEN_MAX + 1];

void spiRead(const void *header, size_t headerLength, void* data, size_t dataLength) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  memcpy(alignedBuffer, header, headerLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);
  memcpy(data, alignedBuffer, dataLength);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

void spiWrite(const void *header, size_t headerLength, const void* data, size_t dataLength) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 0);

  memcpy(alignedBuffer, header, headerLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, headerLength, HAL_MAX_DELAY);
  memcpy(alignedBuffer, data, dataLength);
  HAL_SPI_Transmit(&hspi1, (uint8_t *)alignedBuffer, dataLength, HAL_MAX_DELAY);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, 1);
}

void delayms(unsigned int delay) {
  vTaskDelay(delay);
}

void spiSetSpeed(dwSpiSpeed_t speed) {
  if (speed == dwSpiSpeedLow) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 10;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    HAL_SPI_Init(&hspi1);
  } else if (speed == dwSpiSpeedHigh) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    hspi1.Init.CRCPolynomial = 10;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
    HAL_SPI_Init(&hspi1);
  }
}

#define DW_RESET_Pin GPIO_PIN_12
#define DW_RESET_GPIO_Port GPIOB

void reset(void) {
    HAL_GPIO_WritePin(GPIOB, DW_RESET_Pin, 0);
    HAL_Delay(2);
    HAL_GPIO_WritePin(GPIOB, DW_RESET_Pin, 1);
    NVIC_EnableIRQ(EXTI0_1_IRQn);
}

void dwOpsInit() {
  dwt_ops.spiRead = spiRead;
  dwt_ops.spiWrite = spiWrite;
  dwt_ops.delayms = delayms;
  dwt_ops.spiSetSpeed = spiSetSpeed;
  dwt_ops.reset = reset;
}

void uwbInit() {
  printf("uwbInit");
  queueInit();
  rangingInit();
  static StaticSemaphore_t irq_semphr_buffer;
  irq_semphr = xSemaphoreCreateBinaryStatic(&irq_semphr_buffer);
  static StaticSemaphore_t ranging_set_lock_buffer;
  ranging_set_lock = xSemaphoreCreateBinaryStatic(&ranging_set_lock_buffer);
  isInit = true;
  dwOpsInit();
  // port_set_dw_ic_spi_fastrate();
  dwt_ops.spiSetSpeed(dwSpiSpeedHigh);
  dwt_ops.reset();
  dwt_ops.delayms(2);
  while (!dwt_checkidlerc()) /* Need to make sure DW IC is in IDLE_RC before
                                proceeding */
  {
    printf("error\r\n");
  };
  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    isInit = false;
    return;
  }
  if (dwt_configure(&config) == DWT_ERROR) {
    isInit = false;
    return;
  }
  dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);
  
  /* Configure the TX spectrum parameters (power, PG delay and PG count) */
  dwt_configuretxrf(&txconfig_options);

  /* Configure Antenna Delay */
  dwt_setrxantennadelay(RX_ANT_DLY);
  dwt_settxantennadelay(TX_ANT_DLY);

  /* Auto re-enable receiver after a frame reception failure (except a frame
   * wait timeout), the receiver will re-enable to re-attempt reception.*/
  dwt_or32bitoffsetreg(SYS_CFG_ID, 0, SYS_CFG_RXAUTR_BIT_MASK);
  dwt_setrxtimeout(DEFAULT_RX_TIMEOUT);

  dwt_setcallbacks(&tx_cb, &rx_cb, &rx_to_cb, &rx_err_cb, NULL, NULL);
  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and
   * RX errors). */
  dwt_setinterrupt(SYS_ENABLE_LO_TXFRS_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPTO_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXPHE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFCE_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXFSL_ENABLE_BIT_MASK |
                       SYS_ENABLE_LO_RXSTO_ENABLE_BIT_MASK,
                   0, DWT_ENABLE_INT);

  /* Clearing the SPI ready interrupt */
  dwt_write32bitreg(SYS_STATUS_ID,
                    SYS_STATUS_RCINIT_BIT_MASK | SYS_STATUS_SPIRDY_BIT_MASK);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  isInit = true;
}

static int checkIrq() { return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0); }

static void uwbTask(void* parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }
  while (true) {
    if (xSemaphoreTake(irq_semphr, portMAX_DELAY)) {
      do {
        dwt_isr();
      } while (checkIrq() != 0);
    }
  }
}

static void uwbTxTask(void* parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  UWB_Packet_t packetCache;
  while (true) {
    if (xQueueReceive(txQueue, &packetCache, portMAX_DELAY)) {
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.length, (uint8_t *) &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.length + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        DEBUG_PRINT("uwbTxTask:  TX ERROR\n");
      }
    }
  }
}

void uwbStart() {
  printf("uwbStart");
  static StaticTask_t uwbStaticTask;
  static StackType_t uwbStaticStack[configMINIMAL_STACK_SIZE];

  xTaskCreateStatic(uwbTask, "uwbTask", configMINIMAL_STACK_SIZE, NULL,
                    configMAX_PRIORITIES - 1, uwbStaticStack, &uwbStaticTask);

  static StaticTask_t uwbTxStaticTask;
  static StackType_t uwbTxStaticStack[2 * configMINIMAL_STACK_SIZE];

  xTaskCreateStatic(uwbTxTask, "uwbTxTask", 2 * configMINIMAL_STACK_SIZE, NULL,
                    configMAX_PRIORITIES - 1, uwbTxStaticStack,
                    &uwbTxStaticTask);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  switch (GPIO_Pin) {
    case GPIO_PIN_0:
      xSemaphoreGiveFromISR(irq_semphr, &higherPriorityTaskWoken);
      HAL_NVIC_ClearPendingIRQ(EXTI0_1_IRQn);
      break;
    default:
      break;
  }
  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}
