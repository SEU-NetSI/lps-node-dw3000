#include "uwb.h"

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <stm32f0xx_hal.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "port_dw3000.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "usbcomm.h"

static bool isInit = false;
static SemaphoreHandle_t irq_semphr;
static SemaphoreHandle_t ranging_set_lock;
static QueueHandle_t tx_queue;
static StaticQueue_t tx_queue_buffer;
static uint8_t tx_queue_storage[TX_QUEUE_SIZE * TX_ITEM_SIZE];

static QueueHandle_t rx_queue;
static StaticQueue_t rx_queue_buffer;
static uint8_t rx_queue_storage[RX_QUEUE_SIZE * RX_ITEM_SIZE];

/* rx buffer used in rx_callback */
static uint8_t rx_buffer[RX_BUFFER_SIZE];
Timestamp_Tuple_t Tf_buffer[Tf_BUFFER_POLL_SIZE] = {0};
static int Tf_buffer_index = 0;
static int seq_number = 0;

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
  static StaticSemaphore_t irq_semphr_buffer;
  irq_semphr = xSemaphoreCreateBinaryStatic(&irq_semphr_buffer);
  static StaticSemaphore_t ranging_set_lock_buffer;
  ranging_set_lock = xSemaphoreCreateBinaryStatic(&ranging_set_lock_buffer);
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

  Ranging_Message_t packetCache;

  while (true) {
    if (xQueueReceive(tx_queue, &packetCache, portMAX_DELAY)) {
      // printf("===uwbTxTask===\r\n");
      // print_ranging_message(&packetCache);
      dwt_forcetrxoff();
      dwt_writetxdata(packetCache.header.message_length, &packetCache, 0);
      dwt_writetxfctrl(packetCache.header.message_length + FCS_LEN, 0, 1);
      
      write(STDOUT_FILENO, "\xbc", 1);
      write(STDOUT_FILENO, &packetCache.header.source_address, 2);
      write(STDOUT_FILENO, &packetCache.header.message_sequence, 2);

      /* Start transmission. */
      if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) ==
          DWT_ERROR) {
        printf("uwbTxTask:  TX ERROR\r\n");
      }
    }
  }
}

int16_t compute_distance(Ranging_Table_t* table) {

  int64_t tRound1, tReply1, tRound2, tReply2, diff1, diff2, tprop_ctn;
  tRound1 = (table->Rr.timestamp.full - table->Tp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply1 = (table->Tr.timestamp.full - table->Rp.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tRound2 = (table->Rf.timestamp.full - table->Tr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  tReply2 = (table->Tf.timestamp.full - table->Rr.timestamp.full + MAX_TIMESTAMP) % MAX_TIMESTAMP;
  diff1 = tRound1 - tReply1;
  diff2 = tRound2 - tReply2;
  tprop_ctn = (diff1 * tReply2 + diff2 * tReply1 + diff2 * diff1) / (tRound1 + tRound2 + tReply1 + tReply2);

  int16_t distance = (int16_t) tprop_ctn * 0.4691763978616;
  // printf("distance=%d cm\r\n", distance);
  /* update ranging table */
  table->Rp = table->Rf;
  table->Tp = table->Tf;
  table->Rr = table->Re;

  table->Rf.timestamp.full = 0;
  table->Rf.sequence_number = 0;
  
  table->Tf.timestamp.full = 0;
  table->Tf.sequence_number = 0;

  table->Tr.timestamp.full = 0;
  table->Tr.sequence_number = 0;

  return (int16_t)distance;
}

void process_ranging_message(
  Ranging_Message_With_Timestamp_t* ranging_message_with_timestamp) {
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
    memset(&table, 0, sizeof(Ranging_Table_t));  // TODO check if necessary
    table.neighbor_address = neighbor_addr;
    table.period = TX_PERIOD_IN_MS;
    table.next_delivery_time = xTaskGetTickCount() + table.period;
    table.expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
    neighbor_index = ranging_table_set_insert(&ranging_table_set, &table);
  }
  Ranging_Table_t* neighbor_ranging_table = &ranging_table_set.set_data[neighbor_index].data;
  /* update Re */
  neighbor_ranging_table->Re.timestamp = ranging_message_with_timestamp->rx_time;
  neighbor_ranging_table->Re.sequence_number = ranging_message->header.message_sequence;

  Timestamp_Tuple_t neighbor_Tr = ranging_message->header.last_tx_timestamp;
  // printf("##########neighbor Tr=%d#########\r\n", neighbor_Tr.sequence_number);
  /* update Tr or Rr*/
  if (neighbor_ranging_table->Tr.timestamp.full == 0) {
    if (neighbor_ranging_table->Rr.sequence_number == neighbor_Tr.sequence_number) {
      neighbor_ranging_table->Tr = neighbor_Tr;
    } else {
      neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
    }
  }
  Timestamp_Tuple_t neighbor_Rf = {.timestamp.full = 0};
  uint8_t body_unit_count = (ranging_message->header.message_length - sizeof(Ranging_Message_Header_t)) / sizeof(Body_Unit_t);
  for (int i = 0; i < body_unit_count; i++) {
    if (ranging_message->body_units[i].address == MY_UWB_ADDRESS) {
      neighbor_Rf = ranging_message->body_units[i].timestamp;
      break;
    }
  }
  /* update Rf and Tf*/
  if (neighbor_Rf.timestamp.full) {
    neighbor_ranging_table->Rf = neighbor_Rf;
    /* find corresponding Tf in Tf_buffer */
    for (int i = 0; i < Tf_BUFFER_POLL_SIZE; i++) {
      if (Tf_buffer[i].sequence_number == neighbor_Rf.sequence_number) {
        neighbor_ranging_table->Tf = Tf_buffer[i];
      }
    }
  }

  if (neighbor_ranging_table->Tr.timestamp.full && neighbor_ranging_table->Rf.timestamp.full && neighbor_ranging_table->Tf.timestamp.full) {
      // printf("===before compute distance===\r\n");
      // print_ranging_table(&ranging_table_set);
      int16_t distance = compute_distance(neighbor_ranging_table);
      // printf("distance to neighbor %d = %d cm\r\n", ranging_message->header.source_address, distance);
      // printf("===after compute distance===\r\n");
      // print_ranging_table(&ranging_table_set);
  } else if (neighbor_ranging_table->Rf.timestamp.full && neighbor_ranging_table->Tf.timestamp.full) {
      neighbor_ranging_table->Rp = neighbor_ranging_table->Rf;
      neighbor_ranging_table->Tp = neighbor_ranging_table->Tf;
      neighbor_ranging_table->Rr = neighbor_ranging_table->Re;
      neighbor_ranging_table->Rf.timestamp.full = 0;
      neighbor_ranging_table->Tf.timestamp.full = 0;
      neighbor_ranging_table->Tr.timestamp.full = 0;
  }
  /* update expiration time */
  neighbor_ranging_table->expiration_time = xTaskGetTickCount() + M2T(RANGING_TABLE_HOLD_TIME);
}

static void uwbRxTask(void* parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  Ranging_Message_With_Timestamp_t rx_packet_cache;

  while (true) {
    if (xQueueReceive(rx_queue, &rx_packet_cache, portMAX_DELAY)) {
      // xSemaphoreTake(ranging_set_lock, portMAX_DELAY);
      // printf("before process ranging table\r\n");
      // print_ranging_table(&ranging_table_set);
      // printf("after process ranging table\r\n");
      // printf("===uwbRxTask===\r\n");
      // print_ranging_message(&rx_packet_cache.ranging_message);
      process_ranging_message(&rx_packet_cache);
      // print_ranging_table(&ranging_table_set);
      // xSemaphoreGive(ranging_set_lock);
    }
  }
}

static int get_sequence_number() {
  seq_number++;
  return seq_number;
}

static void generate_ranging_message(Ranging_Message_t* ranging_message) {
  /* generate body unit */
  int8_t body_unit_number = 0;
  int cur_seq_number = get_sequence_number();

  for (set_index_t index = ranging_table_set.full_queue_entry; index != -1;
       index = ranging_table_set.set_data[index].next) {
    Ranging_Table_t* table = &ranging_table_set.set_data[index].data;
    if (body_unit_number >= MAX_BODY_UNIT_NUMBER) {
      break;
    }
    if (table->Re.timestamp.full) {
      ranging_message->body_units[body_unit_number].address =
          table->neighbor_address;
      ranging_message->body_units[body_unit_number].timestamp = table->Re;
      body_unit_number++;
      table->Re.sequence_number = 0;
      table->Re.timestamp.full = 0;
    }
  }
  /* generate message header */
  ranging_message->header.source_address = MY_UWB_ADDRESS;
  ranging_message->header.message_length = sizeof(Ranging_Message_Header_t) + sizeof(Body_Unit_t) * body_unit_number;
  // printf("Tx Task Generate: size of ranging_message=%d\r\n", ranging_message->header.message_length);
  // printf("Tx Task Generate: number of body_unit=%d\r\n", body_unit_number);
//   printf("size of message_header=%d, size of body_unit=%d \r\n", sizeof(Ranging_Message_Header_t), sizeof(Body_Unit_t));
  ranging_message->header.message_sequence = cur_seq_number;
  ranging_message->header.last_tx_timestamp = Tf_buffer[Tf_buffer_index];
  ranging_message->header.velocity = 0;
}

static void uwbRangingTask(void* parameters) {
  while (!isInit) {
    printf("false");
    vTaskDelay(1000);
  }

  while (true) {
    // xSemaphoreTake(ranging_set_lock, portMAX_DELAY);

    Ranging_Message_t tx_packet_cache;
    generate_ranging_message(&tx_packet_cache);
    // printf("===Tx Task===\r\n");
    // print_ranging_message(&tx_packet_cache);
    xQueueSend(tx_queue, &tx_packet_cache, portMAX_DELAY);
    // xSemaphoreGive(ranging_set_lock);

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
  // printf("===Rx callback===\r\n");
  // printf("rx_data length = %u \r\n", data_length - FCS_LEN);
  // print_ranging_message(&rx_buffer);
  dw_time_t rx_time;
  dwt_readrxtimestamp(&rx_time.raw);
  Ranging_Message_With_Timestamp_t rx_message_with_timestamp;
  rx_message_with_timestamp.rx_time = rx_time;
  Ranging_Message_t* ranging_message = &rx_buffer;
  rx_message_with_timestamp.ranging_message = *ranging_message;
  xQueueSendFromISR(rx_queue, &rx_message_with_timestamp,
                    &xHigherPriorityTaskWoken);
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void tx_cb() {
  dw_time_t tx_time;
  dwt_readtxtimestamp(&tx_time.raw);
  // printf("===Tx callback===\r\n");
  // printf("Current Tx Timestasmp: %2x%8lx \r\n",tx_time.high8,tx_time.low32);
  Tf_buffer_index++;
  Tf_buffer_index %= Tf_BUFFER_POLL_SIZE;
  Tf_buffer[Tf_buffer_index].sequence_number = seq_number;
  Tf_buffer[Tf_buffer_index].timestamp = tx_time;
}

void rx_to_cb() {
  dwt_forcetrxoff();
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_err_cb() { printf("rx_err_cb\r\n"); }
