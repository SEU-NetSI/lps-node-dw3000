#ifndef __UWB_H__
#define __UWB_H__

#include "deca_device_api.h"
#include "deca_regs.h"
#include "FreeRTOS.h"
#include <stdbool.h>

#define FRAME_LEN_MAX 127
#define UWB_RX_TIMEOUT 0xFFFFF // RX_TIMEOUT in us
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

extern dwt_txconfig_t txconfig_options;
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,               /* Channel number. */
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64,  /* STS length, see allowed values in Enum dwt_sts_lengths_e */
    DWT_PDOA_M0      /* PDOA mode off */
};

#define TX_QUEUE_SIZE 5
#define RX_QUEUE_SIZE 5
#define TX_ITEM_SIZE sizeof(Ranging_Message_t)
#define RX_ITEM_SIZE sizeof(Ranging_Message_t)

#define TX_PERIOD_IN_MS 200

#define RANGING_TABLE_SIZE 10

typedef uint16_t address_t;
typedef portTickType Time_t;
typedef short set_index_t;

/* timestamp */
typedef union dwTime_u {
  uint8_t raw[5];
  uint64_t full;
  struct {
    uint32_t low32;
    uint8_t high8;
  } __attribute__((packed));
  struct {
    uint8_t low8;
    uint32_t high32;
  } __attribute__((packed));
} dw_time_t;

/* Timestamp Tuple */
typedef struct {
  uint16_t sequence_number;
  dw_time_t timestamp;
} __attribute__((packed)) Timestamp_Tuple_t;

/* Body Unit */
typedef struct {
  address_t address;
  Timestamp_Tuple_t timestamp;
} __attribute__((packed)) Body_Unit_t;

/* Ranging Message Header*/
typedef struct {
  address_t source_address;
  uint16_t message_sequence;
  Timestamp_Tuple_t last_tx_timestamp;
  short velocity;
} __attribute__((packed)) Ranging_Message_Header_t;

/* Ranging Message */
typedef struct {
  Ranging_Message_Header_t header;
  Body_Unit_t body_units[RANGING_TABLE_SIZE];
} __attribute__((packed)) Ranging_Message_t;

/* Ranging Table
  +------+------+------+------+------+
  |  Rp  |  Tr  |  Rf  |  P   |  tn  |
  +------+------+------+------+------+
  |  Tp  |  Rr  |  Tf  |  Re  |  ts  |
  +------+------+------+------+------+
*/
typedef struct {
  address_t neighbor_address;

  Timestamp_Tuple_t Rp;
  Timestamp_Tuple_t Tp;
  Timestamp_Tuple_t Rr;
  Timestamp_Tuple_t Tr;
  Timestamp_Tuple_t Rf;
  Timestamp_Tuple_t Tf;
  Timestamp_Tuple_t Re;
  
  Time_t period;
  Time_t next_delivery_time;
  Time_t expiration_time;
  int16_t distance;
} __attribute__((packed)) Ranging_Table_t;

/* Ranging Table Set*/
typedef struct {
  set_index_t index;
  Ranging_Table_t tables[RANGING_TABLE_SIZE];
} __attribute__((packed)) Ranging_Table_Set_Item_t;

typedef struct
{
  Ranging_Table_Set_Item_t set_data[RANGING_TABLE_SIZE];
  set_index_t free;
  set_index_t full;
  int size;
} Ranging_Table_Set_t;

Ranging_Table_Set_t ranging_table_set;

/*Ranging Table Set Operations*/`
void ranging_table_init(Ranging_Table_Set_t *rangingTable);

set_index_t ranging_table_insert(Ranging_Table_Set_t *ranging_table_set, Ranging_Table_t *table);

set_index_t find_in_ranging_table(Ranging_Table_Set_t *ranging_table_set, address_t addr);

bool delete_ranging_tuple_by_index(Ranging_Table_Set_t *ranging_table_set, set_index_t index);

void print_ranging_table_tuple(Ranging_Table_Set_t *tuple);

void print_ranging_table(Ranging_Table_Set_t *ranging_table_set);

bool ranging_table_clear_expire(Ranging_Table_Set_t *ranging_table_set);

void sort_ranging_table(Ranging_Table_Set_t *ranging_table_set);

void uwbInit();
void uwbStart();

void rx_cb();
void tx_cb();
void rx_to_cb();
void rx_err_cb();

#endif