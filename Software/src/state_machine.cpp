#include "state_machine.h"

/**
 * State machine data
 */

typedef enum {
    ERROR           = 0,
    SYSTEMS_CHECK   = 1,
    IDLE            = 2,
    PREPARATION     = 3,
    ARMED           = 4,
    LAUNCHED        = 5,
    DEPLOYED        = 6
} state_type;

static state_type flight_state = SYSTEMS_CHECK;

/**
 * Implementation of the lbp state callbacks
 */

uint8_t lbp_state_error() {
    return flight_state == ERROR;
}

uint8_t lbp_state_armed() {
    return flight_state >= ARMED;
}

/**
 * Initialize the state machine
 */
void init_state_machine() {

}

/**
 * Update the state machine.
 */
void update_state_machine() {
    switch (flight_state) {
        case ERROR:
            // be annoying
            if (!buzzer_queue_length()) {
                buzzer_beep(BEEP_LONG);
            }

            // exit the state once we're no longer armed,
            // if battery voltage is in good state
            // and if there's a squib connected if one is necessary
            if (!is_armed() &&
                get_battery_value() > eeprom_read_safe(&battery_empty_limit) &&
                (eeprom_read_safe(&use_servo) || is_squib_connected())) {

                buzzer_beep(BEEP_SHORT);
                buzzer_beep(BEEP_SHORT);
                set_status_led(ON);
                flight_state = IDLE;
            }
            break;

        case SYSTEMS_CHECK:
            // this state is the entry state, it performs startup checking of some peripherals

            // close the servo if necessary
            if (eeprom_read_safe(&use_servo)) {
                set_servo_position(eeprom_read_safe(&servo_closed_position));
            }

            // check if the battery is empty
            // also, check if there's a squib connected if we're configured for one.
            if ((get_battery_value() <= eeprom_read_safe(&battery_empty_limit)) ||
			((!eeprom_read_safe(&use_servo) && !is_squib_connected()))) {

                flight_state = ERROR;
                break;
            }

            // if everything's okay, go into idle
            buzzer_beep(BEEP_SHORT);
            buzzer_beep(BEEP_SHORT);
            set_status_led(ON);
            flight_state = IDLE;
            break;

        case IDLE:
            if (is_armed()) {
                flight_state = ERROR;
                break;
            }

            if (is_breakwire_connected()) {
                buzzer_beep(BEEP_SHORT);
                buzzer_beep(BEEP_SHORT);
                set_status_led(OFF);
                flight_state = PREPARATION;
                break;
            }
            break;

        case PREPARATION:
            if (!is_breakwire_connected()) {
                buzzer_beep(BEEP_LONG);
                set_status_led(ON);
                flight_state = IDLE;
                break;
            }

            if (is_armed()) {
                if (!eeprom_read_safe(&use_servo) && !is_squib_connected()) {
                    flight_state = ERROR;

                } else {
                    buzzer_beep(BEEP_SHORT);
                    buzzer_beep(BEEP_SHORT);
                    set_status_led(ON);
                    flight_state = ARMED;
                }
            }
            break;

        case ARMED:
            if (!is_armed()) {
                buzzer_beep(BEEP_LONG);
                set_status_led(OFF);
                flight_state = PREPARATION;
                break;
            }

            if (!is_breakwire_connected()) {
                reset_timer();
                set_launch_asserted(ON);
                flight_state = LAUNCHED;
                break;
            }
            break;

        case LAUNCHED:
            if (!buzzer_queue_length()) {
                buzzer_beep(BEEP_SHORT);
            }

            if (get_timer() >= eeprom_read_safe(&max_deploy_time) || 
                (get_timer() >= eeprom_read_safe(&min_deploy_time) && is_vote_asserted())) {
                if (eeprom_read_safe(&use_servo)) {
                    set_servo_position(eeprom_read_safe(&servo_open_position));

                } else {
                    set_pyro_state(ON);

                }
                eeprom_write_safe(&last_logged_deploy_time, get_timer());
                flight_state = DEPLOYED;
                break;
            }

            break;

        case DEPLOYED:
            if (!buzzer_queue_length()) {
                buzzer_beep(BEEP_LONG);
            }

            break;
    }

}
