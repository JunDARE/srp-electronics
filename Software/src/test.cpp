#define A 6 // 0.25 sec tone
#define K 12 // 0.5 sec tone
#define H 24 // 1 sec tone
#define F 48 // 2 sec tone
#define S -12 // 0.5 sec silence

#include "test.h"

/**
 * Bytecode for the song on single pitch
 */
#define SONG_LEN 18
const int8_t song[SONG_LEN] = {
    S, S, A, A, A, A,
    K, A, A, K, A, A,
    H, A, A, A, A,
    F,
};

/**
 * Default test impl
 */
static void beep_song() {
    reset_timer();
    for (uint8_t i = 0; i < SONG_LEN; i++) {
        int8_t dur = song[i];
        if (dur >= 0) {
            buzzer_beep(dur);
            while (buzzer_queue_length()) {
                _delay_us(1000);
            }
        } else {
            uint16_t start = get_timer();
            while (get_timer() < (start - dur - dur)) {
                _delay_us(1000);
            }
        }
    }
}

/**
 * This file contains the interface to test code that is activated when the programming jumper is shorted
 */
void test() {
    while (1) {
        beep_song();
    }
}

/**
 * beeps a byte encoded on the buzzer. long beep = 1, short beep = 0. It's terminated by a normal beep.
 */
void beep_byte(uint8_t b) {
    while (b) {
        if (b & 1) {
            buzzer_beep(BEEP_LONG);
        } else {
            buzzer_beep(BEEP_SHORT);
        }
        b >>= 1;
    }
    while (buzzer_queue_length());
    buzzer_beep(BEEP_NORMAL);
    _delay_ms(2000);
}
