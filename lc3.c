/**
* Little computer 3
* word size = 16 bits
*/

#include <inttypes.h>
#include <stdio.h>

/** 
* Mem addr range
* 0000..FFFF
*/

#define MAX_CAPACITY (1 << 16) /* 2**16 -> 65536 */
uint16_t mem[MAX_CAPACITY];

/**
* Registers
* General purpose R0..R7 (16 bit width)
*/
enum
{
    R0 = 0,
    R1,
    R2,
    R3,
    R4,
    R5,
    R6,
    R7,
    PC,    /* progam counter */
    FLAGS, /* processor flags (CONDITIONS?) */
    RCOUNT,
};

uint16_t reg[RCOUNT];

enum 
{
    FPOS = 1 << 0,  /* P flag */
    FZRO = 1 << 1,  /* Z flag */
    FNEG = 1 << 2   /* N flag */
};


enum
{
    BR = 0, /* branch */
    ADD,    /* add */
    LD,     /* load */
    ST,     /* store */
    JSR,    /* jump subroutine */
    AND,    /* bitwise and */
    LDR,    /* load register */
    STR,    /* store register */
    RTI,    /* unused (return from interrupt?) */
    NOT,    /* bitwise not */
    LDI,    /* load indirect */
    STI,    /* store indirect */
    JMP,    /* jump */
    RES,    /* unused. reserved */
    LEA,    /* load effective addr */
    TRAP    
};

int
main(void)
{
    return 0;
}
