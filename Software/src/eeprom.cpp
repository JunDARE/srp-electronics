#include "eeprom.h"

// deployment times
uint16_t EEMEM min_deploy_time = 500; // 20ms increments: 10 sec
uint16_t EEMEM max_deploy_time = 700; // 20ms increments: 14 sec

// Logging of actual deployment time
uint16_t EEMEM last_logged_deploy_time = 0; // 20ms increments, written to by the microcontroller

// status checking
uint8_t EEMEM battery_empty_limit = 166; // This correspond to 6.5V that goes through a voltage divider (/2) and is read in a 8-bit ADC (19.53 mV/bit)


// deployment settings
uint8_t  EEMEM use_servo = 1; // by default servo is used, not pyro
uint8_t  EEMEM servo_closed_position = 0; // servorange/256 increments
uint8_t  EEMEM servo_open_position = 255; // servorange/256 increments

// communication settings
uint8_t  EEMEM lbp_address = 0;

/**
 * Initialize the EEPROM state
 */
void init_eeprom() {

}
