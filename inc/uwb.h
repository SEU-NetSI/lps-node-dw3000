#ifndef __UWB_H__
#define __UWB_H__

#include "deca_device_api.h"
#include "deca_regs.h"
#include "ranging_struct.h"

#define UWB_RX_TIMEOUT 0xFFFFF       // RX_TIMEOUT in us
#define MAX_TIMESTAMP 1099511627776  // 2**40
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385

#define SPEED_OF_LIGHT 299702547
#define FRAME_LEN_MAX 127
#define FRAME_LEN_MAX_EX 1023

extern dwt_txconfig_t txconfig_options;
/* Default communication configuration. We use default non-STS DW mode. */
static dwt_config_t config = {
    5,            /* Channel number. */
    DWT_PLEN_128, /* Preamble length. Used in TX only. */
    DWT_PAC8,     /* Preamble acquisition chunk size. Used in RX only. */
    9,            /* TX preamble code. Used in TX only. */
    9,            /* RX preamble code. Used in RX only. */
    1, /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for
          non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */
    DWT_BR_6M8,      /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    DWT_PHRRATE_STD, /* PHY header rate. */
    (129 + 8 - 8), /* SFD timeout (preamble length + 1 + SFD length - PAC size).
                      Used in RX only. */
    DWT_STS_MODE_OFF,
    DWT_STS_LEN_64, /* STS length, see allowed values in Enum dwt_sts_lengths_e
                     */
    DWT_PDOA_M0     /* PDOA mode off */
};

#define MY_UWB_ADDRESS 1
#define TX_QUEUE_SIZE 5
#define RX_QUEUE_SIZE 5
#define TX_ITEM_SIZE sizeof(Ranging_Message_t)
#define RX_ITEM_SIZE sizeof(Ranging_Message_With_Timestamp_t)
#define RX_BUFFER_SIZE RX_ITEM_SIZE  // RX_BUFFER_SIZE ≤ FRAME_LEN_MAX
#define Tf_BUFFER_POLL_SIZE 5
#define TX_PERIOD_IN_MS 50

void uwbInit();
void uwbStart();

void rx_cb();
void tx_cb();
void rx_to_cb();
void rx_err_cb();

#endif