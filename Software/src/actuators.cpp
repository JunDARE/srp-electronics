#include "actuators.h"

/**
 * Internal data
 */

// buzzer queue data structures
typedef struct {
    uint8_t duration;
} buzzer_queue_entry;

typedef struct {
    volatile uint8_t index;
    volatile uint8_t length;
    buzzer_queue_entry queue[BUZZER_QUEUE_SIZE];
} buzzer_queue_type;

static buzzer_queue_type buzzer_queue;

// buzzer current beep data
#define BUZZER_OFF      0
#define BUZZER_ON       1
#define BUZZER_COOLDOWN 2
static uint8_t buzzer_state = BUZZER_OFF;
static uint8_t current_beep_duration;

// timer data structure
static volatile uint16_t timer_20ms;

/**
 * Internal routines.
 * the tick routines are called every 20 ms in a timer1 interrupt
 */

static void servo_tick() {
    // pull the servo pwm pin high
    WRITE_PIN(SERVO_PIN, 1);
}

static void timer_tick() {
    timer_20ms++;
}

static void buzzer_tick() {
    // should we turn the buzzer on?
    if (buzzer_state == BUZZER_OFF) {
        if (buzzer_queue.length) {
            buzzer_state = BUZZER_ON;
            current_beep_duration = 0;
            WRITE_PIN(BUZZER_PIN, 1);

        }
        return;

    } 

    // check if we should state switch
    current_beep_duration++;
    if (current_beep_duration > buzzer_queue.queue[buzzer_queue.index].duration) {
        if (buzzer_state == BUZZER_ON) {
            // we're done beeping and go into a cooldown for the same duration
            buzzer_state = BUZZER_COOLDOWN;
            current_beep_duration = 0;
            WRITE_PIN(BUZZER_PIN, 0);

        } else {
            buzzer_queue.index = (buzzer_queue.index + 1) % BUZZER_QUEUE_SIZE;
            buzzer_queue.length--;
            // There's something in the queue
            if (buzzer_queue.length) {
                buzzer_state = BUZZER_ON;
                current_beep_duration = 0;
                WRITE_PIN(BUZZER_PIN, 1);

            // or not
            } else {
                buzzer_state = BUZZER_OFF;

            }
        }
    }
}

/* Timer interrupts. They are used as follows
 * Timer 0: manages the pwm signal generation and keeps a counter that
 * counts in 20ms increments.
 * The first match (COMPB) will pull the PWM pin
 * low when the servo pwm width is reached.
 * The second match (COMPA) will pull the PWM pin high when 20ms is reached
 */

ISR(TIMER1_COMPB_vect) {
    // pull the servo pwm pin low
    WRITE_PIN(SERVO_PIN, 0);
}

ISR(TIMER1_COMPA_vect) {
    servo_tick();
    timer_tick();
    buzzer_tick();
}

/**
 * Public routines
 */


/**
 * Initialize the actuator peripherals
 */
void init_actuators() {
    // pins
    SET_PIN_AS_OUTPUT(BUZZER_PIN);
    SET_PIN_AS_OUTPUT(PYRO_PIN);
    SET_PIN_AS_OUTPUT(SERVO_PIN);
    SET_PIN_AS_OUTPUT(LED_PIN);
    SET_PIN_AS_OUTPUT(LAUNCH_ASSERTED_PIN);

    WRITE_PIN(BUZZER_PIN, 0);
    WRITE_PIN(PYRO_PIN, 0);
    WRITE_PIN(SERVO_PIN, 0);
    WRITE_PIN(LED_PIN, 0);
    WRITE_PIN(LAUNCH_ASSERTED_PIN, 0);

    // timer
    // enable CTC (reset on) OCR1A
    TCCR1B = 1 << WGM12;
    // Enable both the COMPA and COMB interrupts
    TIMSK = (1 << OCIE1A) | (1 << OCIE1B);
    // Use a clock divider of x8
    TCCR1B |= 1 << CS11;
    // Configure the COMPA interrupt to happen after 20ms
    OCR1A = 18432;
    // And initialize the COMBP interrupt to happen at 0ms for now
    // This is later modified to determine the servo position
    OCR1B = 0;
}

/**
 * Buzzer control interface. This function will enable the buzzer for length*20ms and then be
 * silent for length*20ms. There is a BUFFER_QUEUE_SIZE length queue for the buzzer, any items
 * that do not fit in the queue will be discarded silently
 */
void buzzer_beep(uint8_t length) {
    ATOMIC(
        // guard against full queue
        if (buzzer_queue.length != BUZZER_QUEUE_SIZE) {
            // index is the current entry being buzzed, (index + length) % size is therefore where a new entry is added
            buzzer_queue_entry *entry = buzzer_queue.queue + ((buzzer_queue.index + buzzer_queue.length) % BUZZER_QUEUE_SIZE);
            entry->duration = length;
            buzzer_queue.length++;
        }
    );
}

/**
 * Returns the amount of items in the buzzer queue.
 */
uint8_t buzzer_queue_length() {
    return buzzer_queue.length;
}

/**
 * Set the state of a led connected to the breakout connector.
 * enabled should be set to ON or OFF.
 */
void set_status_led(uint8_t enabled) {
    WRITE_PIN(LED_PIN, enabled);
}

/**
 * Sets the state of the pyro output.
 * enabled should be set to ON or OFF.
 */
void set_pyro_state(uint8_t enabled) {
    WRITE_PIN(PYRO_PIN, enabled);
}

/**
 * Sets the position of the servo.
 * position can be anywhere from 0 to 255 (which maps to the entire servo range).
 */
void set_servo_position(uint8_t position) {
    // position is 0-256 mapped to 1-2 ms aka 1843.2 - 3686.4 counts
    OCR1B = 921 + ((uint16_t)position) * 231 / 64;
}

/**
 * Sets the status of the GPIO pin in the breakout connector.
 * This is used to signal to an expansion board that the rocket has been launched.
 * enabled should be set to ON or OFF.
 */
void set_launch_asserted(uint8_t enabled) {
    WRITE_PIN(LAUNCH_ASSERTED_PIN, enabled);
}

/**
 * Sets the value of the timer to 0. The timer is a 16-bit unsigned int that counts
 * every 20 ms. This means it wraps around after slightly more than 1300 sec.
 */
void reset_timer() {
    ATOMIC(timer_20ms = 0;);
}

/**
 * Gets the value of the timer that can be reset by reset_timer.
 */
uint16_t get_timer() {
    uint16_t temp;
    ATOMIC(temp = timer_20ms;);
    return temp;
}


