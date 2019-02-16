#ifndef _INPUTS_H_
#define _INPUTS_H_

#include "config.h"

/**
 * This file contains interfaces that measure data from the ouside world
 */

/**
 * Initialize the input peripherals
 */
void init_inputs();

/**
 * Returns nonzero when the vote in pin is pulled high
 */
uint8_t is_vote_asserted();

/**
 * Returns nonzero when a valid pyro is connected to the pyro output
 */
uint8_t is_squib_connected();

/**
 * Returns nonzero when the armed switch is in armed mode
 */
uint8_t is_armed();

/**
 * Returns nonzero when a breakwire is connected
 */
uint8_t is_breakwire_connected();

/**
 * Returns the most recent ADC measurement of the battery sensor.
 */
uint8_t get_battery_value() ;

#endif
