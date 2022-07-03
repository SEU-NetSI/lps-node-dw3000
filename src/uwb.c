#include "uwb.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f0xx_hal.h>
#include <string.h>

#include "FreeRTOS.h"
#include "port_dw3000.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

static bool isInit = false;
static SemaphoreHandle_t irq_semphr;

static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_buffer;
static uint8_t tx_queue_storage[TX_QUEUE_SIZE * TX_ITEM_SIZE];

static QueueHandle_t rx_queue;
static StaticQueue_t rx_queue_buffer;
static uint8_t rx_queue_storage[RX_QUEUE_SIZE * RX_ITEM_SIZE];

/* rx buffer used in rx_callback */
static uint8_t rx_buffer[RX_BUFFER_SIZE];

void queueInit() {
  tx_queue = xQueueCreateStatic(TX_QUEUE_SIZE, TX_ITEM_SIZE, tx_queue_storage,
                               &tx_queue_buffer);
  rx_queue = xQueueCreateStatic(RX_QUEUE_SIZE, RX_ITEM_SIZE, rx_queue_storage,
                               &rx_queue_buffer);
}

void uwbInit() {
  printf("uwbInit");
  queueInit();
  ranging_table_set_init(&ranging_table_set);
  static StaticSemaphore_t irqSemaphoreBuffer;
  irq_semphr = xSemaphoreCreateBinaryStatic(&irqSemaphoreBuffer);

  isInit = true;

  port_set_dw_ic_spi_fastrate();
  reset_DWIC();
  Sleep(2);
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
  dwt_setrxtimeout(UWB_RX_TIMEOUT);

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

static void uwbTask(void *parameters) {
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

static void uwbTxTask(void *parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  Ranging_Message_t packetCache;

  while (true) {
    if (xQueueReceive(tx_queue, &packetCache, portMAX_DELAY)) {
      dwt_forcetrxoff();
      dwt_writetxdata(sizeof(packetCache), &packetCache, 0);
      dwt_writetxfctrl(sizeof(packetCache) + FCS_LEN, 0, 1);
      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        printf("uwbTxTask:  TX ERROR\r\n");
      }
    }
  }
}

int16_t compute_distance(Ranging_Table_t* table) {
    int64_t round1, reply1, round2, reply2, tof_dtu;
    round1 = table->Rr.timestamp.full - table->Tp.timestamp.full;
    reply1 = table->Tr.timestamp.full - table->Rp.timestamp.full;
    round2 = table->Rf.timestamp.full - table->Tr.timestamp.full;
    reply2 = table->Tf.timestamp.full - table->Rr.timestamp.full;
    tof_dtu = (round1 * round2 - reply2 * reply1) / (round1 + round2 + reply1 + reply2);
    double tof = tof_dtu * DWT_TIME_UNITS;
    double distance = tof * SPEED_OF_LIGHT;
    printf("distance=%f\r\n", distance);

    /* update ranging table */
    table->Rp = table->Rf;
    table->Tp = table->Tf;
    table->Rr = table->Re;
    table->Rf.timestamp.full = 0;
    table->Tf.timestamp.full = 0;
    table->Re.timestamp.full = 0;

    return (int16_t) distance;
}


void process_ranging_message(Ranging_Message_With_Timestamp_t * ranging_message_with_timestamp) {
    Ranging_Message_t* ranging_message = &ranging_message_with_timestamp->ranging_message;
    address_t neighbor_addr = ranging_message->header.source_address;

    set_index_t neighbor_index = find_in_ranging_table_set(&ranging_table_set, neighbor_addr);
    /* handle new neighbor */
    if (neighbor_index == -1) {
        if (ranging_table_set.free_queue_entry == -1) {
            /* ranging table set is full */
            return;
        }
        Ranging_Table_t table;
        memset(&table, 0, sizeof(Ranging_Table_t)); // TODO check if necessary
        table.neighbor_address = neighbor_addr;
        table.period = TX_PERIOD_IN_MS;
        table.next_delivery_time = xTaskGetTickCount() + table.period;
        table.expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
        neighbor_index = ranging_table_set_insert(&ranging_table_set, &table);
    }

    Timestamp_Tuple_t neighbor_Tr = ranging_message->header.last_tx_timestamp; 
    Timestamp_Tuple_t neighbor_Rf = {.timestamp.full = 0};
    uint8_t body_unit_count = (ranging_message->header.message_length - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
    for (int i = 0; i < body_unit_count; i++) {
        if (ranging_message->body_units[i].address == MY_UWB_ADDRESS) {
            neighbor_Rf = ranging_message->body_units[i].timestamp;
            break;
        }
    }

    Ranging_Table_t* neighbor_ranging_table = &ranging_table_set.set_data[neighbor_index].data;
    /* update Re & Tr & Rf */
    neighbor_ranging_table->Re.timestamp = ranging_message_with_timestamp->rx_time;
    neighbor_ranging_table->Re.sequence_number = ranging_message->header.message_sequence;
    neighbor_ranging_table->Tr = neighbor_Tr;
    neighbor_ranging_table->Rf = neighbor_Rf;

    /* handle message loss and mismatch */
    if (neighbor_ranging_table->Rf.timestamp.full == 0) {
        neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
        // TODO check if neighbor_ranging_table->Tr = 0 works
        neighbor_ranging_table->Tr.timestamp.full == 0;
        neighbor_ranging_table->Tf.timestamp.full == 0;
        neighbor_ranging_table->Re.timestamp.full == 0;
    } else if (neighbor_ranging_table->Tr.sequence_number != neighbor_ranging_table->Rr.sequence_number) {
        neighbor_ranging_table->Rp = neighbor_ranging_table->Rf;
        neighbor_ranging_table->Tp = neighbor_ranging_table->Tf;
        neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
        neighbor_ranging_table->Tr.timestamp.full = 0;
        neighbor_ranging_table->Rf.timestamp.full = 0;
        neighbor_ranging_table->Tf.timestamp.full = 0;
        neighbor_ranging_table->Re.timestamp.full = 0;
    }

    /* try to compute distance */
    if (neighbor_ranging_table->Tp.timestamp.full && 
    neighbor_ranging_table->Rp.timestamp.full && 
    neighbor_ranging_table->Tr.timestamp.full && 
    neighbor_ranging_table->Rr.timestamp.full && 
    neighbor_ranging_table->Tf.timestamp.full && 
    neighbor_ranging_table->Rf.timestamp.full) {
        compute_distance(neighbor_ranging_table);
    }

    /* update experiation time */
    neighbor_ranging_table->expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
}

static void uwbRxTask(void *parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  Ranging_Message_With_Timestamp_t rx_packet_cache;

  while (true) {
    if (xQueueReceive(rx_queue, &rx_packet_cache, portMAX_DELAY)) {
        process_ranging_message(&rx_packet_cache);
    }
  }
}

static void uwbRangingTask(void *parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }
  int seq_number = 0;

  while (true) {
    Ranging_Message_t tx_packet_cache;
    seq_number++;
    
    xQueueSend(tx_queue, &tx_packet_cache, portMAX_DELAY);
    vTaskDelay(TX_PERIOD_IN_MS);
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

  static StaticTask_t uwbRxStaticTask;
  static StackType_t uwbRxStaticStack[2 * configMINIMAL_STACK_SIZE];

  xTaskCreateStatic(uwbRxTask, "uwbRxTask", 2 * configMINIMAL_STACK_SIZE, NULL,
                    configMAX_PRIORITIES - 1, uwbRxStaticStack,
                    &uwbRxStaticTask);

  static StaticTask_t uwbRangingStaticTask;
  static StackType_t uwbRangingStaticStack[2 * configMINIMAL_STACK_SIZE];

  xTaskCreateStatic(
      uwbRangingTask, "uwbRangingTask", 2 * configMINIMAL_STACK_SIZE, NULL,
      configMAX_PRIORITIES - 1, uwbRangingStaticStack, &uwbRangingStaticTask);
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

void rx_cb() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  uint32_t data_length =
      dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_BIT_MASK;
  if (data_length != 0 && data_length <= FRAME_LEN_MAX) {
    dwt_readrxdata(rx_buffer, data_length - FCS_LEN,
                   0); /* No need to read the FCS/CRC. */
  }

  dw_time_t rx_time;
  dwt_readrxtimestamp(&rx_time);
  Ranging_Message_With_Timestamp_t rx_message_with_timestamp = {.rx_time=rx_time, .ranging_message=rx_buffer};
  xQueueSendFromISR(rx_queue, &rx_message_with_timestamp, &xHigherPriorityTaskWoken);

  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void tx_cb() {

}

void rx_to_cb() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_err_cb() { printf("rx_err_cb\r\n"); }
