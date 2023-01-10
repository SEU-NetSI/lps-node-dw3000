#ifndef __UWB_H__
#define __UWB_H__

#include "libdw3000.h"
#include "dwTypes.h"
#include "mac_802_15_4.h"

/* Function Switch */
#define ENABLE_PHR_EXT_MODE

#define SPEED_OF_LIGHT 299702547
#define MAX_TIMESTAMP 1099511627776  // 2**40
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define FRAME_LEN_STD 127
#define FRAME_LEN_EXT 1023
#ifdef ENABLE_PHR_EXT_MODE
#define FRAME_LEN_MAX FRAME_LEN_EXT
#else
#define FRAME_LEN_MAX FRAME_LEN_STD
#endif

#define DEFAULT_RX_TIMEOUT 0xFFFFF

/* Queue Constants */
#define RX_QUEUE_SIZE 2
#define RX_QUEUE_ITEM_SIZE sizeof(UWB_Packet_t)

typedef uint16_t address_t;

/* Packet */
#define PACKET_SIZE FRAME_LEN_MAX
#define PAYLOAD_SIZE (PACKET_SIZE - sizeof(Packet_Header_t))

/* TX options */
static dwt_txconfig_t txconfig_options = {
    .PGcount = 0x0,
    .PGdly = 0x34,
    .power = 0xfdfdfdfd
};

/* PHR configuration */
static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
          non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
#ifdef ENABLE_PHR_EXT_MODE
    DWT_PHRMODE_EXT, /* Extended PHY header mode. */
#else
    DWT_PHRMODE_STD, /* Standard PHY header mode. */
#endif
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e
                     */
    DWT_PDOA_M0     /* PDOA mode off */
};

/* UWB packet definition */
typedef enum {
  RANGING = 0,
  FLOODING = 1,
  DATA = 2,
  MESSAGE_TYPE_COUNT, /* only used for counting message types. */
} MESSAGE_TYPE;

typedef struct {
  mhr_802_15_4_t mac;    // mac header
  struct {
    MESSAGE_TYPE type: 6;
    uint16_t length: 10;
  };
} __attribute__((packed)) Packet_Header_t;

typedef struct {
  Packet_Header_t header; // Packet header
  uint8_t payload[PAYLOAD_SIZE]
} __attribute__((packed)) UWB_Packet_t;

typedef void (*UWBCallback)(void *);

void uwbInit();
void uwbStart();

void rx_cb();
void tx_cb();
void rx_to_cb();
void rx_err_cb();

#endif