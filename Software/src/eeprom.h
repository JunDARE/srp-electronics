#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <avr/eeprom.h>
#include "config.h"

/**
 * This file contains interfaces for manipulating the EEPROM and declarations
 * of the variables stored in EEPROM.
 */

 /**
  * Variable declarations
  */

// deployment times
extern uint16_t EEMEM min_deploy_time; // 20ms increments
extern uint16_t EEMEM max_deploy_time; // 20ms increments

// Logging of actual deployment time
extern uint16_t EEMEM last_logged_deploy_time; // 20ms increments

// status checking
extern uint8_t EEMEM battery_empty_limit; // 6.5V read in the 8-bit ADC

// deployment settings
extern uint8_t  EEMEM use_servo; // nonzero: use the servo, zero: use the pyro
extern uint8_t  EEMEM servo_closed_position; // servorange/256 increments
extern uint8_t  EEMEM servo_open_position; // servorange/256 increments

// communication settings
extern uint8_t  EEMEM lbp_address; // Contains an id for the rocket


/**
 * These overloaded functions do the right thing based on their input data types
 * With this API, to read any of the above variables, just call eeprom_read(&var_name);
 *
 * Note: the since the default eeprom handling functions aren't interrupt safe, 
 * the safe functions disable interrupts during the read/write, they should be used
 * During a non-interrupt context if any eeprom functions are called in interrupts.
 *
 * Note2: These functions busy wait until the eeprom is available.
 */
inline uint8_t  eeprom_read(uint8_t *address)  {
    return eeprom_read_byte(address);
}
inline uint16_t eeprom_read(uint16_t *address) {
    return eeprom_read_word(address);
}
inline uint16_t eeprom_read(uint32_t *address) {
    return eeprom_read_dword(address);
}

inline uint8_t eeprom_read_safe(uint8_t *address)  {
    uint8_t value;
    ATOMIC(
        value = eeprom_read_byte(address);
    );
    return value;
}
inline uint16_t eeprom_read_safe(uint16_t *address) {
    uint16_t value;
    ATOMIC(
        value = eeprom_read_word(address);
    );
    return value;
}
inline uint16_t eeprom_read_safe(uint32_t *address) {
    uint32_t value;
    ATOMIC(
        value = eeprom_read_dword(address);
    );
    return value;
}

inline void eeprom_write(uint8_t *address,  uint8_t value)  {
    eeprom_update_byte(address,  value);
}
inline void eeprom_write(uint16_t *address, uint16_t value) {
    eeprom_update_word(address,  value);
}
inline void eeprom_write(uint32_t *address, uint32_t value) {
    eeprom_update_dword(address, value);
}


inline void eeprom_write_safe(uint8_t *address,  uint8_t value)  {
    ATOMIC(
        eeprom_update_byte(address,  value);
    );
}
inline void eeprom_write_safe(uint16_t *address, uint16_t value) {
    ATOMIC(
        eeprom_update_word(address,  value);
    );
}
inline void eeprom_write_safe(uint32_t *address, uint32_t value) {
    ATOMIC(
        eeprom_update_dword(address, value);
    );
}

/**
 * Initialize the EEPROM state
 */
void init_eeprom();

#endif
