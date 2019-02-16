#ifndef _ACTUATORS_H_
#define _ACTUATORS_H_

#include "config.h"

/**
 * This file contains functions that manipulate peripherals to interface with
 * the world.
 */

// actuator state
#define ON  1
#define OFF 0

// buzzer beeps
#define BEEP_SHORT 12
#define BEEP_NORMAL 25
#define BEEP_LONG 50
#define BEEP_FOREVER 0xFF

// amount of entries in buzzer queue
#define BUZZER_QUEUE_SIZE 8

/**
 * Initialize the actuator peripherals
 */
void init_actuators();

/**
 * Buzzer control interface. This function will enable the buzzer for length*20ms and then be
 * silent for length*20ms. There is a BUFFER_QUEUE_SIZE length queue for the buzzer, any items
 * that do not fit in the queue will be discarded silently
 */
void buzzer_beep(uint8_t length);

/**
 * Returns the amount of items in the buzzer queue.
 */
uint8_t buzzer_queue_length();

/**
 * Set the state of a led connected to the breakout connector.
 * enabled should be set to ON or OFF.
 */
void set_status_led(uint8_t enabled);

/**
 * Sets the state of the pyro output.
 * enabled should be set to ON or OFF.
 */
void set_pyro_state(uint8_t enabled);

/**
 * Sets the position of the servo.
 * position can be anywhere from 0 to 255 (which maps to the entire servo range).
 */
void set_servo_position(uint8_t position);

/**
 * Sets the status of the GPIO pin in the breakout connector.
 * This is used to signal to an expansion board that the rocket has been launched.
 * enabled should be set to ON or OFF.
 */
void set_launch_asserted(uint8_t enabled);

/**
 * Sets the value of the timer to 0. The timer is a 16-bit unsigned int that counts
 * every 20 ms. This means it wraps around after slightly more than 1300 sec.
 */
void reset_timer();

/**
 * Gets the value of the timer that can be reset by reset_timer.
 */
uint16_t get_timer();

#endif
