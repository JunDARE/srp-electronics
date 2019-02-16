#include "eeprom.h"
#include "inputs.h"
#include "actuators.h"
#include "state_machine.h"
#include "lbp.h"

/**
 * Fuse config
 */
FUSES = {
  
    (FUSE_SUT_CKSEL1), // .low - crystal oscillator 3..8MHz
    (FUSE_EESAVE & FUSE_SPIEN & FUSE_BODLEVEL1 & FUSE_BODLEVEL0), // .high - enable preserving eeprom and enable programming through SPI, set maximum levels for BOD
    (FUSE_BODACT0 & FUSE_BODPD0), // .extended - BOD enable in Active, Idle and Sleep Modes
        
};

/**
 * Initialization routine. Called directly after boot with interrupts disabled
 */
void init() {
    // set the main clock divider to 1
    CLKPR = 0; // Prescaler set to 1 -> CPU running at 7.3728 MHz

    // run all module initializers
    init_actuators();
    init_eeprom();
    init_inputs();
    init_state_machine();
    init_lbp();
}

/**
 * Update routine. Called in a loop after the initialization routine has completed
 */
void update() {
    update_state_machine();
    _delay_ms(10); // wait 10ms between successive state transitions
}

/**
 * Program entry point
 */
int main() {
    // make sure all interrupts are disabled
    ATOMIC(
        init();
    );
    while (1) {
        update();
    }
    return 0;
}

/**
 * Below is the implementation of the LBP message handler
 */

/*
 * Definition of the message id's
 */
#define LBP_GET_MIN_DEPLOY_TIME             0x10
#define LBP_SET_MIN_DEPLOY_TIME             0x20

#define LBP_GET_MAX_DEPLOY_TIME             0x11
#define LBP_SET_MAX_DEPLOY_TIME             0x21

#define LBP_GET_MEASURED_DEPLOY_TIME        0x12

#define LBP_GET_BATTERY_VOLTAGE             0x13

#define LBP_GET_BATTERY_EMPTY_LIMIT         0x14
#define LBP_SET_BATTERY_EMPTY_LIMIT         0x24

#define LBP_GET_DEPLOY_MODE                 0x15
#define LBP_SET_DEPLOY_MODE                 0x25

#define LBP_GET_SERVO_CLOSED_POSITION       0x16
#define LBP_SET_SERVO_CLOSED_POSITION       0x26

#define LBP_GET_SERVO_OPEN_POSITION         0x17
#define LBP_SET_SERVO_OPEN_POSITION         0x27

#define LBP_SET_SERVO_POSITION              0x28

#define LBP_GET_ADDRESS                     0x19
#define LBP_SET_ADDRESS                     0x29

/**
 * LBP message handler
 */
void lbp_handler(lbp_packet *packet, uint8_t data_length, lbp_packet *reply) {
    reply->id = packet->id;

    uint16_t temp;

    if (packet->id >= 0x20) {
        // all setters
        switch (packet->id) {
            case LBP_SET_MIN_DEPLOY_TIME:
                if (data_length != 2) {
                    break;
                }
                temp = ((uint16_t)packet->data[1]) << 8 | packet->data[0];
                eeprom_write(&min_deploy_time, temp);
                reply->data[0] = packet->data[0];
                reply->data[1] = packet->data[1];
                lbp_send_message(2);
                return;

            case LBP_SET_MAX_DEPLOY_TIME:
                if (data_length != 2) {
                    break;
                }
                temp = ((uint16_t)packet->data[1]) << 8 | packet->data[0];
                eeprom_write(&max_deploy_time, temp);
                reply->data[0] = packet->data[0];
                reply->data[1] = packet->data[1];
                lbp_send_message(2);
                return;

            case LBP_SET_BATTERY_EMPTY_LIMIT:
                if (data_length != 1) {
                    break;
                }
                eeprom_write(&battery_empty_limit, packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;

            case LBP_SET_DEPLOY_MODE:
                if (data_length != 1) {
                    break;
                }
                eeprom_write(&use_servo, packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;

            case LBP_SET_SERVO_CLOSED_POSITION:
                if (data_length != 1) {
                    break;
                }
                eeprom_write(&servo_closed_position, packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;

            case LBP_SET_SERVO_OPEN_POSITION:
                if (data_length != 1) {
                    break;
                }
                eeprom_write(&servo_open_position, packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;

            case LBP_SET_SERVO_POSITION:
                if (data_length != 1) {
                    break;
                }
                set_servo_position(packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;

            case LBP_SET_ADDRESS:
                if (data_length != 1) {
                    break;
                }
                eeprom_write(&lbp_address, packet->data[0]);
                reply->data[0] = packet->data[0];
                lbp_send_message(1);
                return;
        }
    } else if (!data_length) {
        // getters
        switch (packet->id) {
            case LBP_GET_MIN_DEPLOY_TIME:
                temp = eeprom_read(&min_deploy_time);
                reply->data[1] = temp >> 8;
                reply->data[0] = temp & 0xFF;
                lbp_send_message(2);
                return;

            case LBP_GET_MAX_DEPLOY_TIME:
                temp = eeprom_read(&max_deploy_time);
                reply->data[1] = temp >> 8;
                reply->data[0] = temp & 0xFF;
                lbp_send_message(2);
                return;

            case LBP_GET_MEASURED_DEPLOY_TIME:
                temp = eeprom_read(&last_logged_deploy_time);
                reply->data[1] = temp >> 8;
                reply->data[0] = temp & 0xFF;
                lbp_send_message(2);
                return;

            case LBP_GET_BATTERY_VOLTAGE:
                reply->data[0] = get_battery_value();
                lbp_send_message(1);
                return;

            case LBP_GET_BATTERY_EMPTY_LIMIT:
                reply->data[0] = eeprom_read(&battery_empty_limit);
                lbp_send_message(1);
                return;

            case LBP_GET_DEPLOY_MODE:
                reply->data[0] = eeprom_read(&use_servo);
                lbp_send_message(1);
                return;

            case LBP_GET_SERVO_CLOSED_POSITION:
                reply->data[0] = eeprom_read(&servo_closed_position);
                lbp_send_message(1);
                return;

            case LBP_GET_SERVO_OPEN_POSITION:
                reply->data[0] = eeprom_read(&servo_open_position);
                lbp_send_message(1);
                return;

            case LBP_GET_ADDRESS:
                reply->data[0] = eeprom_read(&lbp_address);
                lbp_send_message(1);
                return;
        }
    }
    reply->id = LBP_NACK;
    lbp_send_message(0);
}
