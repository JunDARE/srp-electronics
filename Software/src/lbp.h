#ifndef _LBP_H_
#define _LBP_H_

#include "config.h"

/**
 * This file contains a launch box protocol implementation interface.
 * To use this file in a project, implement the callback functions
 */

 /**
  * Defines relating to LBP
  */

// size of the rx and tx buffer. Current code only accounts for one packet / buffer
#define LBP_BUFFER_SIZE 32

// masks
#define LBP_TYPE_MASK       0xC0
#define LBP_SEQNUM_MASK     0xC0
#define LBP_ADDRESS_MASK    0x3F

// packet types
#define LBP_SYNC       0x00
#define LBP_REPLY      0x40
#define LBP_ASYNC      0x80
#define LBP_BROADCAST  0xC0

// Default source address
#define LBP_SOURCE_ADDRESS 0x3F

// message id's
#define LBP_NACK                            0x01
#define LBP_IDENTIFY                        0x02
#define LBP_IDENTIFY_ASYNC_REPLY            0x03
#define LBP_EXTENDED_IDENTIFY               0x03
#define LBP_NETWORK_DISCOVERY               0x04
#define LBP_NETWORK_DISCOVERY_ASYNC_REPLY   0x05
#define LBP_STATUS_REQUEST                  0x06
#define LBP_STATUS_REQUEST_ASYNC_REPLY      0x07

// Default replies
#define LBP_IDENTIFY_CONTENT_0              0xB0 // identification code 0x000B, major version 0
#define LBP_IDENTIFY_CONTENT_1              0x01 // minor version 0, but stable

#define LBP_EXTENDED_IDENTIFY_CONTENT_0     0x0B
#define LBP_EXTENDED_IDENTIFY_CONTENT_1     0x00
#define LBP_EXTENDED_IDENTIFY_NAME          "SRP V0.0 "

// Macros to extract information from packets
#define LBP_TYPE(packet) ((packet)->srcinfo & LBP_TYPE_MASK)
#define LBP_SRC_ADDR(packet) ((packet)->srcinfo & LBP_ADDRESS_MASK)
#define LBP_SEQNUM(packet) ((packet)->destinfo & LBP_SEQNUM_MASK)

/**
 * packet type
 */
typedef struct __attribute__((packed)) {
    uint8_t srcinfo;
    uint8_t destinfo;
    uint8_t id;
    uint8_t data[LBP_BUFFER_SIZE - 3];
} lbp_packet;

/**
 * The following functions must be implemented in order to handle incoming packets
 */

/**
 * Handle arbitrary synchronous packets. The function MUST call lbp_send_message() or lbp_discard_message().
 * Note that the reply address and packet type are already set. The only thing the application has to set is
 * the id and the data.
 */
void lbp_handler(lbp_packet *packet, uint8_t data_length, lbp_packet *reply);

/**
 * Should return nonzero when the rocket has encountered an error
 */
uint8_t lbp_state_error();

/**
 * Should return nonzero when the rocket is armed
 */
uint8_t lbp_state_armed();

/**
 * The following functions are the interface to the driver
 */

/**
 * Initialize the peripheral and internal state
 */
void init_lbp();

/**
 * Acquire access to the tx buffer. This will return NULL when the reply buffer is in use.
 */
lbp_packet *lbp_get_tx_buffer();

/**
 * Send the current message in the tx buffer and revokes access to it.
 */
void lbp_send_message(uint8_t data_length);

/**
 * Discard the current tx buffer and revokes access to it.
 */
void lbp_discard_message();



#endif
