#ifndef _STATE_MACHINE_H_
#define _STATE_MACHINE_H_

#include "eeprom.h"
#include "inputs.h"
#include "actuators.h"
#include "lbp.h"
#include "test.h"

/**
 * This file contains the interface to the SRP state machine
 */

/**
 * Initialize the state machine
 */
void init_state_machine();

/**
 * Update the state machine.
 */
void update_state_machine();


#endif
