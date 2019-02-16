#include "inputs.h"

/**
 * Initialize the input peripherals
 */
void init_inputs() {
    // inputs
    SET_PIN_AS_INPUT(VOTE_IN_PIN);
    SET_PIN_AS_INPUT(ARMED_SWITCH_PIN);
    SET_PIN_AS_INPUT(BREAKWIRE_PIN);
    SET_PIN_AS_INPUT(CONTINUITY_DETECTION_PIN);

    // pullups
    SET_PULLUP(VOTE_IN_PIN, 1);
    SET_PULLUP(ARMED_SWITCH_PIN, 1);
    SET_PULLUP(CONTINUITY_DETECTION_PIN, 1);
    
    // ADC for battery measurement (ADC0 - PA3)
    DIDR0 = 
            (1 << ADC0D);      // Disable digital input register for the ADC0
            
    ADMUX =
            (0 << REFS1) |     // Sets ref. voltage to VCC, bit 1
            (0 << REFS0) |     // Sets ref. voltage to VCC, bit 0
            (0 << MUX3)  |     // use ADC0 for input (PA3), MUX bit 3
            (0 << MUX2)  |     // use ADC0 for input (PA3), MUX bit 2
            (0 << MUX1)  |     // use ADC0 for input (PA3), MUX bit 1
            (0 << MUX0);       // use ADC0 for input (PA3), MUX bit 0

    ADCSRB =
            (1 << ADLAR) |     // Left adjusted result. Only necessary to read ADCH for 8-bit precission.
            (0 << ADTS2) |     // Free Running mode, ADTS bit 2
            (0 << ADTS1) |     // Free Running mode, ADTS bit 1
            (0 << ADTS0);      // Free Running mode, ADTS bit 0
            
    ADCSRA = 
            (1 << ADEN)  |     // Enable ADC 
            (1 << ADSC)  |     // Start the conversion manually or start Free Running mode
            (1 << ADATE) |     // Enable auto triggering for free running mode.
            (0 << ADIE)  |     // Disable Interruptions when conversion is complete
            (1 << ADPS2) |     // set prescaler to 64, bit 2 // For 8Mhz CLK it is 125kHz ADC_CLK
            (1 << ADPS1) |     // set prescaler to 64, bit 1 // Usually 13 ADC_CLK cycles are needed per conversion
            (0 << ADPS0);      // set prescaler to 64, bit 0 // So, after 100uS a measurement is available after activation
            
}

/**
 * Returns nonzero when the vote in pin is pulled high
 */
uint8_t is_vote_asserted() {
    return !READ_PIN(VOTE_IN_PIN);
}

/**
 * Returns nonzero when a valid pyro is connected to the pyro output
 */
uint8_t is_squib_connected() {
    return READ_PIN(CONTINUITY_DETECTION_PIN);
}

/**
 * Returns nonzero when the armed switch is in armed mode
 */
uint8_t is_armed() {
    return !READ_PIN(ARMED_SWITCH_PIN);
}

/**
 * Returns nonzero when a breakwire is connected
 */
uint8_t is_breakwire_connected() {
    return READ_PIN(BREAKWIRE_PIN);
}

/**
 * Returns the most recent ADC measurement of the battery sensor.
 * 
 */
uint8_t get_battery_value() {
    // The ADC is in Free Running mode, then we need to wait for ADIF == 1 in order to read a new value
    while (! (ADCSRA & (1 << ADIF) ) );
    return ADCH; // Store the battery voltage     return battery_voltage;  // Return the battery voltage
}
