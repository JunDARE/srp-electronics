#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stddef.h>

/**
 * This file contains the configuration for the SRP Electronics board and
 * Some utility methods for the files.
 */

// circuit configuration

// Crystal frequency
#define CPU_FREQ            7372800

// Necessary for delay.h
#define F_CPU               CPU_FREQ
#include <util/delay.h>

// UART baud rate
#define UART_BAUD           38400

// input pins (for Attiny-1634)
#define VOTE_IN_PIN                 C, 1
#define ARMED_SWITCH_PIN            A, 4
#define BREAKWIRE_PIN               B, 1
#define CONTINUITY_DETECTION_PIN    A, 5
#define EXTRA_GPIO1                 A, 6
#define EXTRA_GPIO2                 A, 1
#define EXTRA_GPIO3                 A, 0

// actuator pins (for Attiny-1634)
#define BUZZER_PIN                  B, 3
#define PYRO_PIN                    A, 2
#define SERVO_PIN                   C, 0
#define LED_PIN                     B, 2
#define LAUNCH_ASSERTED_PIN         C, 2

/**
 * Peripheral configuration is generally hardcoded in the modules.
 * actuators.cpp uses Timer 1 and the ADC0
 * eeprom.cpp uses the EEPROM
 * lbp.cpp uses the USART
 */

/**
 * Utility defines
 */

// preprocessor messing around
#define CONCAT_(A, B) A ## B
#define CONCAT(A, B) CONCAT_(A, B)

// masks a bit with a certain value
#define MASK(name, shift, value) ((name) = ((name) & ~(1 << (shift))) | ((value) << (shift)))

// intermediaties
#define BANK_NAME(name, number) name
#define BANK_NUMBER(name, number) number

// initialization
#define SET_PIN_AS_OUTPUT(pin) MASK(CONCAT(DDR, BANK_NAME(pin)), BANK_NUMBER(pin), 1)
#define SET_PIN_AS_INPUT(pin) MASK(CONCAT(DDR, BANK_NAME(pin)), BANK_NUMBER(pin), 0)
#define SET_PULLUP(pin, val) MASK(CONCAT(PUE, BANK_NAME(pin)), BANK_NUMBER(pin), val)

// reading / writing
#define READ_PIN(pin) (CONCAT(PIN, BANK_NAME(pin)) & (1 << BANK_NUMBER(pin)))
#define WRITE_PIN(pin, val) MASK(CONCAT(PORT, BANK_NAME(pin)), BANK_NUMBER(pin), (val))

// atomic section
// a simple atomic section. Use this only in the main loop while interrupts are enabled
#define ATOMIC(block) do {cli(); {block} sei();} while (0)
// reentrant atomic section. use this if a function can be called from both contexts where
// interrupts are enabled and where interrupts are disabled.
#define NESTED_ATOMIC(block) do {uint8_t oldSREG = SREG; cli(); {block} SREG = oldSREG} while (0)

#endif
