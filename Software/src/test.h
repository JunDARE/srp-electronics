#ifndef _TEST_H_
#define _TEST_H_

#include "config.h"
#include "actuators.h"
#include "inputs.h"
#include "lbp.h"

/**
 * This file contains the interface to test code that is activated when the programming jumper is shorted
 */
void test();


/**
 * beeps a byte encoded on the buzzer. long beep = 1, short beep = 0. It's terminated by a normal beep.
 * This exists for debugging reasons
 */
void beep_byte(uint8_t b);

#endif
