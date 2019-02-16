#include "lbp.h"

/**
 * Internal data structures
 */

// link layer state
#define CHAR_ESCAPE     0x50
#define CHAR_START      0x55
#define CHAR_STOP       0x5A

#define STATE_IDLE      0
#define STATE_FRAME     1
#define STATE_ESCAPING  2
#define STATE_ENDING    3
#define STATE_FILLING   4

// rx state
static uint8_t rx_link_state = STATE_IDLE;
static uint8_t rx_crc;

// tx state
volatile static uint8_t tx_link_state = STATE_IDLE;
static uint8_t tx_crc;

// receive buffer
static uint8_t lbp_rx_buffer[LBP_BUFFER_SIZE];
static uint8_t lbp_rx_buffer_length;

// transmit buffer
static uint8_t lbp_tx_buffer[LBP_BUFFER_SIZE];
static uint8_t lbp_tx_buffer_length;
static uint8_t lbp_tx_buffer_index;

/**
 * The crc function used by the launchbox protocol
 */
static uint8_t crc8(uint8_t data, uint8_t crc) {
    crc = crc ^ data;
    for (uint8_t i = 0; i < 8; i++) {
        if (crc & 1) {
            crc = (crc >> 1) ^ 0x8C;

        } else {
            crc = (crc >> 1);

        }
    }
    return crc;
}

/**
 * This function parses a packet in the receive buffer and
 * handles any reserved messages. If the message is meant for user code, it
 * calls lbp_handler.
 */
static void parse_packet(lbp_packet *packet, uint8_t data_length) {
    // acquire a reply buffer
    lbp_packet *reply = lbp_get_tx_buffer();
    if (!reply) {
        return;
    }

    reply->destinfo = LBP_SRC_ADDR(packet) | LBP_SEQNUM(packet);

    // temp var used for the reserved commands
    uint8_t temp;

    // This device doesn't care about getting a reply
    if (LBP_TYPE(packet) == LBP_REPLY) {
        lbp_discard_message();
        return;
    }

    // handle supported reserved commands
    if (packet->id < 0x10) {
        switch (packet->id) {
            case LBP_NACK:
                lbp_discard_message();
                break;

            case LBP_IDENTIFY:
                packet->srcinfo |= (LBP_TYPE(packet) == LBP_SYNC) ? LBP_REPLY : LBP_ASYNC;
                packet->id = (LBP_TYPE(packet) == LBP_ASYNC) ? LBP_IDENTIFY_ASYNC_REPLY : LBP_IDENTIFY;
                reply->data[0] = LBP_IDENTIFY_CONTENT_0;
                reply->data[1] = LBP_IDENTIFY_CONTENT_1;
                lbp_send_message(2);
                break;

            case LBP_EXTENDED_IDENTIFY:
                // syncronous only
                if (LBP_TYPE(packet) != LBP_SYNC) {
                    lbp_discard_message();
                    break;
                }

                packet->srcinfo |= LBP_REPLY;
                // calculate the page number and nack if it's reserved
                temp = (data_length < 1) ? 0 : packet->data[0];
                if (temp >= 0x10) {
                    packet->id = LBP_NACK;
                    lbp_send_message(0);
                    break;
                }

                packet->id = LBP_EXTENDED_IDENTIFY;

                if (temp == 0) {
                    reply->data[0] = LBP_EXTENDED_IDENTIFY_CONTENT_0;
                    reply->data[1] = LBP_EXTENDED_IDENTIFY_CONTENT_1;
                    lbp_send_message(2);

                } else if (temp == 1) {
                    for (temp = 0; LBP_EXTENDED_IDENTIFY_NAME[temp]; temp++) {
                        reply->data[temp] = LBP_EXTENDED_IDENTIFY_NAME[temp];
                    }
                    lbp_send_message(temp - 1);

                } else {
                    lbp_send_message(0);

                }
                break;

            case LBP_NETWORK_DISCOVERY:
                if (LBP_TYPE(packet) == LBP_SYNC) {
                    packet->srcinfo |= LBP_REPLY;
                    packet->id = LBP_NACK;
                    lbp_send_message(0);
                } else {
                    lbp_discard_message();

                }
                break;

            case LBP_STATUS_REQUEST:
                packet->srcinfo |= (LBP_TYPE(packet) == LBP_SYNC) ? LBP_REPLY : LBP_ASYNC;
                packet->id = (LBP_TYPE(packet) == LBP_ASYNC) ? LBP_STATUS_REQUEST_ASYNC_REPLY : LBP_STATUS_REQUEST;

                reply->data[0] = (1 << 4) | (lbp_state_error() ? 2 << 1 : 0) | (lbp_state_armed() ? 1 : 0);
                lbp_send_message(1);
                break;

            default:
                if (LBP_TYPE(packet) == LBP_SYNC) {
                    packet->srcinfo |= LBP_REPLY;
                    packet->id = LBP_NACK;
                    lbp_send_message(0);

                } else {
                    lbp_discard_message();

                }
        }
    } else {
        if (LBP_TYPE(packet) == LBP_SYNC) {
            reply->srcinfo |= LBP_REPLY;
            lbp_handler(packet, data_length, reply);

        } else {
            lbp_discard_message();

        }
    }
}

/**
 * Interrupt handlers
 */
#include "actuators.h"
/**
 * This interrupt fires once a complete byte has been received. It will parse the 
 * link layer state and when the frame is complete it will call parse_packet on the packet
 */
ISR(USART0_RX_vect) {
    // read the byte from the shift reg
    uint8_t byte = UDR0;

    // are we escaping
    if (rx_link_state == STATE_ESCAPING) {
        byte = ~byte;
        rx_link_state = STATE_FRAME;

    // or are we in a frame
    } else if (rx_link_state == STATE_FRAME) {
        switch (byte) {
            case CHAR_ESCAPE:
                // escape char
                rx_link_state = STATE_ESCAPING;
                return;

            case CHAR_START:
                // can't start a frame inside a frame
                rx_link_state = STATE_IDLE;
                return;

            case CHAR_STOP:
                // end of frame
                rx_link_state = STATE_IDLE;

                // check the crc 
                if (!rx_crc && lbp_rx_buffer_length >= 4) {
                    // Do something with the packet

                    parse_packet((lbp_packet *)lbp_rx_buffer, lbp_rx_buffer_length - 4);
                }
                return;
        }

    // or are we starting a frame
    } else if (byte == CHAR_START) {
        lbp_rx_buffer_length = 0;
        rx_link_state = STATE_FRAME;
        rx_crc = 0;
        return;
    }

    // this is a valid data bit, add it
    if (lbp_rx_buffer_length == LBP_BUFFER_SIZE) {
        // we're full, ignore this packet
        rx_link_state = STATE_IDLE;
        return;
    }

    lbp_rx_buffer[lbp_rx_buffer_length++] = byte;
    rx_crc = crc8(byte, rx_crc);
}

/**
 * This interrupt fires when a byte has been transmitted successfully. 
 * It handles the link layer, escaping bytes in the tx buffer where
 * necessary and computing the crc
 */
ISR(USART0_TX_vect) {
    // if we're idling, do nothing
    if (tx_link_state == STATE_IDLE || tx_link_state == STATE_FILLING) {
        return;
    }

    // if we've ended the packet
    if (tx_link_state == STATE_ENDING) {
        UDR0 = CHAR_STOP;
        tx_link_state = STATE_IDLE;

    // if we hit the end of the data we need to write a (possibly escaped) crc
    } else if (lbp_tx_buffer_index == lbp_tx_buffer_length) {
        // we've printed the escape character for the crc
        if (tx_link_state == STATE_ESCAPING) {
            UDR0 = ~tx_crc;
            tx_link_state = STATE_ENDING;

        // the crc needs to be escaped
        } else if (tx_crc == CHAR_STOP || tx_crc == CHAR_START || tx_crc == CHAR_ESCAPE) {
            UDR0 = CHAR_ESCAPE;
            tx_link_state = STATE_ESCAPING;

        // The crc can be printed without escaping
        } else {
            UDR0 = tx_crc;
            tx_link_state = STATE_ENDING;

        }

    // if this char is following an escape char
    } else if (tx_link_state == STATE_ESCAPING) {
        UDR0 = ~lbp_tx_buffer[lbp_tx_buffer_index - 1];
        tx_link_state = STATE_FRAME;


    // if we're sending data
    } else {
        uint8_t byte = lbp_tx_buffer[lbp_tx_buffer_index++];

        // update the crc
        tx_crc = crc8(byte, tx_crc);

        // check if we should escape
        if (byte == CHAR_STOP || byte == CHAR_START || byte == CHAR_ESCAPE) {
            UDR0 = CHAR_ESCAPE;
            tx_link_state = STATE_ESCAPING;

        } else {
            UDR0 = byte;

        }
    }
}

/**
 * Public interface
 */

/**
 * Initialize the peripheral and internal state
 */
void init_lbp() {
    // USART0 is used for UART communication
    
    // no special modes
    UCSR0A = 0;
    // Enable the pins and enable the interrupt on receiving data
    UCSR0B = (1 << RXCIE0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << TXEN0);
    // Set the frame format (8bits, no parity, 1 stop bit)
    UCSR0C = (3 << UCSZ00);
    // Baud rate registers
    UBRR0H = ((CPU_FREQ / UART_BAUD / 16) - 1) >> 8;
    UBRR0L = ((CPU_FREQ / UART_BAUD / 16) - 1) & 0xFF;

}

/**
 * Acquire access to the tx buffer. This will return NULL when the reply buffer is in use.
 */
lbp_packet *lbp_get_tx_buffer() {
    lbp_packet *buffer = NULL;
    // make sure we don't get interrupted when claiming the buffer
    ATOMIC(
        // can we claim the buffer?
        if (tx_link_state == STATE_IDLE) {
            tx_link_state = STATE_FILLING;
            buffer = (lbp_packet *)lbp_tx_buffer;
        }
    );
    if (buffer) {
        buffer->srcinfo = LBP_SOURCE_ADDRESS;
    }
    return buffer;
}

/**
 * Send the current message in the tx buffer and revokes access to it.
 */
void lbp_send_message(uint8_t data_length) {
    lbp_tx_buffer_length = data_length + 3;
    lbp_tx_buffer_index = 0;
    tx_crc = 0;

    // this function can only be called if the transmit buffer was empty so we can just directly write to output reg
    // start the frame
    UDR0 = CHAR_START;
    tx_link_state = STATE_FRAME;

}

/**
 * Discard the current tx buffer and revokes access to it.
 */
void lbp_discard_message() {
    tx_link_state = STATE_IDLE;
}