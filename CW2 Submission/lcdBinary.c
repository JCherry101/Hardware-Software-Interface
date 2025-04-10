/* ***************************************************************************** */
/* You can use this file to define the low-level hardware control fcts for       */
/* LED, button and LCD devices.                                                  */ 
/* Note that these need to be implemented in Assembler.                          */
/* You can use inline Assembler code, or use a stand-alone Assembler file.       */
/* Alternatively, you can implement all fcts directly in master-mind.c,          */  
/* using inline Assembler code there.                                            */
/* The Makefile assumes you define the functions here.                           */
/* ***************************************************************************** */


#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT		  1

#define	LOW			 0
#define	HIGH		 1


// APP constants   ---------------------------------

// Wiring (see call to lcdInit in main, using BCM numbering)
// NB: this needs to match the wiring as defined in master-mind.c

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// -----------------------------------------------------------------------------
// includes 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

// -----------------------------------------------------------------------------
// prototypes

int failure (int fatal, const char *message, ...);

// -----------------------------------------------------------------------------
// Functions to implement here (or directly in master-mind.c)

/* this version needs gpio as argument, because it is in a separate file */
void digitalWrite(uint32_t *gpio, int pin, int value) {
    if (value == LOW) {
        asm volatile (
            "mov r2, #1\n\t"
            "lsl r2, %[pin]\n\t"
            "str r2, [%[gpio], #40]" 
            :
            : [gpio] "r" (gpio), [pin] "r" (pin)
            : "r2", "memory"
        );
    } else {
        asm volatile (
            "mov r2, #1\n\t"
            "lsl r2, %[pin]\n\t"
            "str r2, [%[gpio], #28]" 
            :
            : [gpio] "r" (gpio), [pin] "r" (pin)
            : "r2", "memory"
        );
    }
}



/* set the @mode@ of a GPIO @pin@ to INPUT or OUTPUT; @gpio@ is the mmaped GPIO base address */
void pinMode(uint32_t *gpio, int pin, int mode) {
    int register_offset = pin / 10;
    int bit_offset = (pin % 10) * 3;
    
    if (mode == OUTPUT) {
        asm volatile (
            "ldr r3, [%[gpio], %[offset]]\n\t"
            "mov r2, #1\n\t"
            "lsl r2, %[bit_offset]\n\t"
            "orr r3, r3, r2\n\t" 
            "str r3, [%[gpio], %[offset]]\n\t"
            :
            : [gpio] "r" (gpio), [offset] "r" (register_offset * 4), [bit_offset] "r" (bit_offset)
            : "r2", "r3", "memory"
        );
    } else {
        asm volatile (
            "ldr r3, [%[gpio], %[offset]]\n\t"
            "mov r2, #7\n\t"
            "lsl r2, %[bit_offset]\n\t"
            "mvn r2, r2\n\t" 
            "and r3, r3, r2\n\t" 
            "str r3, [%[gpio], %[offset]]\n\t"
            :
            : [gpio] "r" (gpio), [offset] "r" (register_offset * 4), [bit_offset] "r" (bit_offset)
            : "r2", "r3", "memory"
        );
    }
}


/* send a @value@ (LOW or HIGH) on pin number @pin@; @gpio@ is the mmaped GPIO base address */
/* can use digitalWrite(), depending on your implementation */
void writeLED(uint32_t *gpio, int led, int value) {
    if (value == HIGH) {
        asm volatile (
            "mov r2, #1\n\t"
            "lsl r2, %[led]\n\t"
            "str r2, [%[gpio], #28]" 
            :
            : [gpio] "r" (gpio), [led] "r" (led)
            : "r2", "memory"
        );
    } else {
        asm volatile (
            "mov r2, #1\n\t"
            "lsl r2, %[led]\n\t"
            "str r2, [%[gpio], #40]" 
            :
            : [gpio] "r" (gpio), [led] "r" (led)
            : "r2", "memory"
        );
    }
}


/* read a @value@ (OFF or ON) from pin number @pin@ (a button device); @gpio@ is the mmaped GPIO base address */
int readButton(uint32_t *gpio, int pin) {
    int value;
    asm volatile (
        "ldr r2, [%[gpio], #52]\n\t" 
        "mov r3, #1\n\t"
        "lsl r3, %[pin]\n\t" 
        "and %[value], r2, r3\n\t" 
        "cmp %[value], #0\n\t"
        "moveq %[value], #0\n\t" 
        "movne %[value], #1\n\t" 
        : [value] "=r" (value)  
        : [gpio] "r" (gpio), [pin] "r" (pin) 
        : "r2", "r3", "cc" 
    );
    return value;
}

/* wait for a button input on pin number @button@; @gpio@ is the mmaped GPIO base address */
/* can use readButton(), depending on your implementation */
int waitForButton(uint32_t *gpio, int button)
{
   while (1) {
        int state = readButton(gpio, button);
        if (state == HIGH) {
            return 1;
            } 
            else {
            struct timespec sleeper, dummy;
            sleeper.tv_sec = 0;
            sleeper.tv_nsec = 100000000;
            nanosleep(&sleeper, &dummy);
        }
    }
}