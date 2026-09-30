/*
 * sw.h - board slide switches S1..S4 (PB12..PB15)
 *
 * A switch reads 0 when it is on ("pressed"), like the original
 * sw_data_set(): bit0 = PB12 (S1) ... bit3 = PB15 (S4).
 */
#ifndef SW_H
#define SW_H

#include <stdint.h>

#define SW1 0x01
#define SW2 0x02
#define SW3 0x04
#define SW4 0x08

void sw_init(void);
/* Raw pin levels, bit0..3 = PB12..PB15 (1 = high = off) */
uint8_t sw_read(void);
/* Bits of the switches that are on (pin low) */
uint8_t sw_on(void);

#endif /* SW_H */
