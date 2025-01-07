/**
* Little Ccomputer 3
* see README  for original resources
* see INTRO   for an introductory note
* see LICENSE for 0BSD
*/

/* Headers */
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>

/* Macros */
#if defined(__GNUC__) || defined(__clang__)
#define Trap() __builtin_trap()
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
* INTRO
*
* Heavy use of comments, attempt at full clarification of what's going on.
* Long comments are not necessarily linked to immediate code before or after
* them, nevertheless it reads fine.
*
* The code layout is not very efficient but it exposes the underlying ideas
* very clearly, since the design is fully flat. You can read the code (not
* the headers) from top to bottom and understand everything.
*
* Fidelity with original resources is not guaranteed.
*
*/


/** 
* Mem addr range
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

enum { MAX_MEM = (1 << 16) }; /* 2**16 -> 65536 mem locations */
uint16_t mem[MAX_MEM]; 

/**
* Registers (16-bit width)
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
    FLAGS,  /* processor flags */     /* condition codes */
    RCOUNT,
}; uint16_t reg[RCOUNT];

/**
* Processor flags (Condition codes)
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
* Instructions (16 bits)
* Bits 15:12 -> opcode
* Bits 11:0  -> extra info
*
* NOTE(M): Extremely important to keep this order! As you can see from the
* comments, each instruction matches it's enum position, counting from 0..15
*
* If you swap any one of them, you break the DECODE step. 
* see switch(op) in main()
*/
enum {      /* Instruction name                         Opcode              */
    BR = 0, /* Conditional Branch                       0000                */
    ADD,    /* Addition                                 0001                */
    LD,     /* Load                                     0010                */
    ST,     /* Store                                    0011                */
    JSR,    /* Jump to Subroutine                       0100                */
    AND,    /* Bit-wise Logical AND                     0101                */
    LDR,    /* Load Register (Base+Offset)              0110                */
    STR,    /* Store Register (Base+Offset)             0111                */
    RTI,    /* Return from Interrupt                    1000                */
    NOT,    /* Bit-Wise Logical NOT                     1001                */
    LDI,    /* Load Indirect                            1010                */
    STI,    /* Store Indirect                           1011                */
    JMP,    /* Jump                                     1100                */
    RES,    /* Reserved                                 1101                */
    LEA,    /* Load Effective Addr                      1110                */
    TRAP    /* System Call                              1111                */
};

static uint16_t
readimage(const char *filepath)
{
    NotImplemented;
    return 0;
}

static uint16_t
memread(uint16_t addr)
{
    NotImplemented;
    return 0;
}

static inline void
memwrite(uint16_t addr, uint16_t value)
{
    mem[addr] = value;
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
*
*             Recommended: https://www.youtube.com/watch?v=4qH4unVtJkE
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
    else                    reg[FLAGS] = FPOS;
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

    /* Initial state: ZERO flag */
    reg[FLAGS] = FZRO;
    
    /* Set Instruction Pointer (PC) */
    enum { IPSTART = 0x3000 }; /* code origin */
    reg[IP] = IPSTART;

    alive = 1; /* set to 0 by HALT (Trap) */
    while (alive)
    {
        /* FETCH */
        uint16_t instr = memread(reg[IP]++); /* read 16-bit instr */
        uint16_t op    = instr >> 12;        /* bits[15:12] set 4-bit opcode */
        switch (op) /* DECODE */
        {
        case ADD:   /* add */
        {
            uint16_t DR, SR1, SR2, imm5, immediate;

            /* Destination Register, bits[11:9] */
            DR  = (instr >> 9) & 0x7; /* 0x7=0111;  compare bottom 3 bits */

            /* Source Register 1, bits[8:6] */
            SR1 = (instr >> 6) & 0x7; /* 0x7=0111; compare bottom 3 bits */

            /* bits[5] ? immediate mode : nonimmediate */
            immediate = (instr >> 5) & 0x1; /* 5th bit set? */
            if (immediate)
            {
                /* imm5 can only store unsigned values <= 2^5=32 */
                imm5 = instr & 0x1F; /* 0x1F=00011111; compare bottom 5 bits */
                signextend(imm5, 5); /* 5-bit -> 16-bit value */
                reg[DR] = reg[SR1] + imm5;
            }
            else
            {
                SR2 = instr & 0x7; /* 0x7=0111; compare bottom 3 bits */
                reg[DR] = reg[SR1] + reg[SR2];
            }

            updateflags(DR); /* ADD instr sets Processor FLAGS */
            break;
        }
        case AND:   /* bitwise and */
        {
            uint16_t DR, SR1, SR2, imm5, immediate;

            /* Destination Register, bits[11:9] */
            DR  = (instr >> 9) & 0x7; /* 0x7=0111;  compare bottom 3 bits */

            /* Source Register 1, bits[8:6] */
            SR1 = (instr >> 6) & 0x7; /* 0x7=0111; compare bottom 3 bits */

            /* bits[5] ? immediate mode : nonimmediate */
            immediate = (instr >> 5) & 0x1; /* 5th bit set? */
            if (immediate)
            {
                /* imm5 can only store unsigned values <= 2^5=32 */
                imm5 = instr & 0x1F; /* 0x1F=00011111; compare bottom 5 bits */
                signextend(imm5, 5); /* 5-bit -> 16-bit value */
                reg[DR] = reg[SR1] & imm5;
            }
            else /* nonimmeditate */
            {
                SR2 = instr & 0x7; /* 0x7=0111; compare bottom 3 bits */
                reg[DR] = reg[SR1] & reg[SR2];
            }

            updateflags(DR); /* AND instr sets Processor FLAGS */
            break;
        }
        case NOT:   /* bitwise not */
        {
            uint16_t DR, SR;

            /* Destinarion Register, bits[11:9] */
            DR = (instr >> 9) & 0x7;
            /* Source Register, bits[8:6] */
            SR = (instr >> 6) & 0x7;

            /* Complement, invert all bits */
            reg[DR] = ~reg[SR];
            updateflags(reg[DR]); /* NOT instr sets Processor FLAGS */
            break;
        }
        case BR:    /* branch */
        {
            uint16_t flagstate, N, Z, P, PCoffset9;

            /* Offset, bits[8:0]*/
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Procesor Flags, bits[11:9] */
            flagstate = (instr >> 9) & 0x7; 
            if (flagstate & reg[FLAGS])
            {

                /**
                * Before the following assignment, the Instruction Pointer's
                * current value is the BR instruction we are decoding.
                *
                * The next instruction to be decoded (in the next fetch cycle)
                * would be the one we are storing in this assignment.
                * 
                */
                reg[IP] += PCoffset9;
            }
            break;
        }
        case LD:
        {
            uint16_t DR, PCoffset9;
            
            /* Destination Register, bits[11:9] */
            DR = (instr >> 9) & 0x7;
            /* Offset from PC, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Load value read into register */
            reg[DR] = memread(reg[IP] + PCoffset9);
            updateflags(reg[DR]); /* set Processor FLAGS */
            break;
        }
        case ST:
        {
            uint16_t SR, PCoffset9;
            
            /* Source Register, bits[11:9] */
            SR = (instr >> 9) & 0x7;
            /* Offset from PC, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Write SR content into PC+Offset addr */
            memwrite(reg[IP] + PCoffset9, reg[SR]);
            break;
        }
        case JSR:
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


