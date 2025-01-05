/**
* LC-3
* WORD SIZE = 16 bits
*/

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

#if defined(__GNUC__) || defined(__clang__)
#define Trap() __buildtin_trap()
#else
#error Unknown trap intrinsic for this compiler.
#endif

#define AssertAlways(x) \
    do { \
        if(!(x)) { \
        printf("\n%s(%d): assertion failed: %s\n", __FILE__, __LINE__, #x); \
        Trap(); \
        } \
    } while(0)

#define Assert(x)           AssertAlways(x)
#define NotImplemented      Assert(!"Not Implemented!")
#define ArrayCount(Array)   (sizeof(Array) / sizeof((Array)[0]))

/** 
* MEM ADDR RANGE
* 0000..FFFF
*
* -- MEMORY MAP --
* 0000 
*  ..   Trap Vector Table
* 00FF 
* 0100  
*  ..   Interrupt Vector Table
* 01FF
* 0200
*  ..   Operating Systems and Supervisor Stack Pointer
* 2FFF
* 3000
*  ..   Userspace
* FDFF
* FE00
*  ..   Device register addr (memory mapped I/O)
* FFFF
* 
*/

enum { MAX_MEM = (1 << 16) }; /* 2**16 -> 65536 */
uint16_t mem[MAX_MEM]; 

/**
* REGISTERS (16-bit width)
* 8 General Purpose,   000..111
* 1 Program counter == Instruction Pointer
* 1 Condition codes == Processor Flags
* ..
*/

enum {
    R0 = 0, /* general purpose */
    R1,     /* general purpose */
    R2,     /* general purpose */
    R3,     /* general purpose */
    R4,     /* general purpose */
    R5,     /* general purpose */
    R6,     /* general purpose */
    R7,     /* general purpose */
    IP,     /* instruction pointer */ /* program counter */
    FLAGS,  /* processor flags */
    RCOUNT,
}; uint16_t reg[RCOUNT];

/**
* PROCESSOR FLAGS 
* LD, LDI, LDR, LEA and ADD, AND and NOT
* load a result into one of the 8 general
* purpose registers. Flags are set based
* on whether the result, a 16-bit two's
* complement number is NEGATIVE, POSITIVE
* or ZERO.    
*/
enum {
    FPOS = 1 << 0,  /* 0001 (1), positive */
    FZRO = 1 << 1,  /* 0010 (2), zero     */
    FNEG = 1 << 2   /* 0100 (4), negative */
};

/**
* INSTRUCTIONS (16 bits)
* Bits 15:12 -> opcode
* Bits 11:0  -> extra info
*/
enum {
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

static uint16_t
memread(uint16_t r)
{
}

static uint16_t
readimage(const char *filepath)
{
}

/**
* -- NOTE(M): Two's complement and the sign extension technique
*
*             Two's complement is a method of signed integer representation,
*             with a few very desirable properties, e.g., a single form for
*             zero and arithmetic consistency between signed and unsigned.
*                         
*             Definition: The Two's complement of an N-bit number is the
*             complement of that number with respect to 2^N
*
*             Given identical bit width, given any binary number and it's
*             two's complement representation, the sum of this two equals 2^N
*             e.g., 2-bit, N=2
*                   2^N = 2^2 = 4 = 100
*                   10 -> Binary representation of '2'
*                   11 -> Two's complement of '10'
*                   10 + 11 = 100 = 2^N
*                   
*             Two's complement algorithm:
*                   Given absolute binary representation of some number x,
*                   |x| (base-2)
*                   1. Invert all bits (one's complement)
*                   2. Add 1 to obtained number on step 1. (ignore any
*                   overflow)
*                   3. Leading bit is the sign bit (0 = Positive, 1 = Negative)
*                   Verify:
*                   Add place values together and subtract leading place (sign)
*                   i.e., 101 = -(1*2^2)+(0*2^1)+(1*2^0) = -4+0+1 = -3
*
*             Sign extension technique:
*             or numerical value preservation given greater (new) width
*             check the code in signextend() first (and the comments)
*
*             Consider this:
*             1100 (base-2) = -4 decimal (two's complement)
*             1100 (4-bit) -> 1111 1111 1111 1100 (16-bit)
*             Two's complement of 1111 1111 1111 1100
*             is:
*                   1. Invert = 0000 0000 0000 0011
*                   2. Add 1  = 0000 0000 0000 0100 = 4 in decimal
*             Therefore,
*             1100 and 1111 1111 1111 1100 are -4 decimal
*/

static inline uint16_t
signextend(uint16_t n, int width)
{
    /**
    * In order to extend a two's complement number
    * we must preserve the sign by repeating the
    * sign bit (MSB) in all the (new) extra bits,
    * e.g., 0x000C(=1100) -> 0xFFFC(=1111 1111 1111 1100)
    */
    if ((n >> (width - 1)) & 1) { /* check sign bit (MSB) of n */
        /* n is negative */

        /**
        * Extend n by setting to 1 all extra bits
        * e.g.,
        * If n=-4=0x000C, width=4=0x0004, 0xFFFF=1111 1111 1111 1111
        * then,
        * 0xFFFF << 0x0004 = 0xFFF0
        * n = 0x000C OR 0xFFF0; n = 0xFFFC = 1111 1111 1111 1100 = -4
        */
        n |= (0xFFFF << width);
    }

    /**
    * Else n is positive
    *
    * In theory we set to 0 all extra bits
    * but **we are not** actually going from
    * a 4-bit number to a 16-bit number, so
    * we don't do anything
    */
    return n;
}


static inline uint16_t
swap16(uint16_t x)
{
    return (x << 8) | (x >> 8);
    
}

static void 
updateflags(const uint16_t r)
{
    if      (reg[r] == 0)   reg[FLAGS] = FZRO;
    else if (reg[r] >> 15)  reg[FLAGS] = FNEG;
    else                    reg[r] = FPOS;
}

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
        if (!readimage(argv[j]))
        {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    /* set zero flag. one flag must always be set */
    reg[FLAGS] = FZRO;
    
    /* set IP */
    enum { IPSTART = 0x3000 }; /* code origin */
    reg[IP] = IPSTART;

    alive = 1; /* set to 0 by HALT (Trap) */
    while (alive)
    {
        /* FETCH */
        uint16_t instr = memread(reg[IP]++); /* read 16-bit instr */
        uint16_t op    = instr >> 12;        /* bits 15:12 (4) set opcode */
        switch (op)
        {
        case BR:    /* branch */
        {
            break;
        }
        case ADD:   /* add */
        {

            break;
        }
        case LD:    /* load */
        {
            break;
        }
        case ST:    /* store */
        {
            break;
        }
        case JSR:   /* jump subroutine */
        {
            break;
        }
        case AND:   /* bitwise and */
        {
            break;
        }
        case LDR:   /* load register */
        {
            break;
        }
        case STR:   /* store register */
        {
            break;
        }
        case NOT:   /* bitwsie not */
        {
            break;
        }
        case LDI:   /* load indirect */
        {
            break;
        }
        case STI:   /* store indirect */
        {
            break;
        }
        case JMP:   /* jump */
        {
            break;
        }
        case LEA:   /* load effective addr */
        {
            break;
        }
        case TRAP: 
        {
            break;
        }
        case RES: /* unsupported */
        case RTI: /* unsupported */
        default:
        {
            /* ERROR */
            abort();
            break;
        }

        }
    }

    return 0;
}
