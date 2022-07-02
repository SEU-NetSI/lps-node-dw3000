#include <stdbool.h>

#include "FreeRTOS.h"

#define MAX_NEIGHBOR_SIZE 5
#define RANGING_TABLE_SIZE MAX_NEIGHBOR_SIZE

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
  Body_Unit_t body_units[MAX_NEIGHBOR_SIZE];
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

typedef struct {
  set_index_t next;
  Ranging_Table_t data;
} __attribute__((packed)) Ranging_Table_Set_Item_t;

/* Ranging Table Set*/
typedef struct {
  Ranging_Table_Set_Item_t set_data[RANGING_TABLE_SIZE];
  set_index_t free_queue_entry;
  set_index_t full_queue_entry;
  int size;
} Ranging_Table_Set_t;

Ranging_Table_Set_t ranging_table_set;

/*Ranging Table Set Operations*/
void ranging_table_init(Ranging_Table_Set_t *ranging_table_set);

set_index_t ranging_table_insert(Ranging_Table_Set_t *ranging_table_set,
                                 Ranging_Table_t *table);

set_index_t find_in_ranging_table(Ranging_Table_Set_t *ranging_table_set,
                                  address_t addr);

bool delete_ranging_tuple_by_index(Ranging_Table_Set_t *ranging_table_set,
                                   set_index_t index);

void print_ranging_table_tuple(Ranging_Table_Set_t *tuple);

void print_ranging_table(Ranging_Table_Set_t *ranging_table_set);

bool ranging_table_clear_expire(Ranging_Table_Set_t *ranging_table_set);

void sort_ranging_table(Ranging_Table_Set_t *ranging_table_set);