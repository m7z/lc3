/**
* Little computer 3
* word size = 16 bits
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

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

uint16_t MemRead(uint16_t r);
uint16_t ReadImage(const char *f);

int
main(int argc, const char **argv)
{
    int j, alive; 

    if (argc < 2)
    {
        printf("lc3 [file-image] ...\n");
        exit(2); /* invalid args */
    }

    for (j = 1; j < argc; j++)
    {
        if (!ReadImage(argv[j]))
        {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    /* set zero flag. one flag must always be set */
    reg[FLAGS] = FZRO;
    
    /* set PC */
    enum { PCSTART = 0x3000 };
    reg[PC] = PCSTART;

    alive = 1;
    while (alive)
    {
        /* FETCH */
        uint16_t instr = MemRead(reg[PC]++); /* 16 bit instruction */
        uint16_t op    = instr >> 12; /* top 4 bits set opcode */
        switch (op)
        {
        case BR:
        {
            break;
        }
        case ADD:
        {
            break;
        }
        case LD:
        {
            break;
        }
        case ST:
        {
            break;
        }
        case JSR:
        {
            break;
        }
        case AND:
        {
            break;
        }
        case LDR:
        {
            break;
        }
        case STR:
        {
            break;
        }
        case NOT:
        {
            break;
        }
        case LDI:
        {
            break;
        }
        case STI:
        {
            break;
        }
        case JMP:
        {
            break;
        }
        case LEA:
        {
            break;
        }
        case TRAP:
        {
            break;
        }
        case RES:
        case RTI:
        default:
        {
            /* ERROR */
            break;
        }

        }
    }

    return 0;
}
