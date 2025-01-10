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
* Registers 
*
* NOTE(M): General Purpose Register selection in instruction implementation 
* All registers in LC-3 (DR, SRx, BaseR...) are 3-bit wide (2^3=0..7)
* During decoding, we read the 3-bit value which selects the corresponding
* register. 
*
* e.g., DR = (instr >> 9) & 0x7, reads bits[11:9] from a 16-bit instruction.
* Later on in the computation we select the correct register, reg[DR], which
* could be 1 of 8, R0..R7 
* 
* see any case in switch(op) inside main()
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
    PC,     /* program counter */
    FLAGS,  /* processor flags */
    RCOUNT,
}; uint16_t reg[RCOUNT];

/**
* Condition codes
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

/* Trap Vector Table (see MEMORY MAP) */
enum
{
    /* Read char from keyboard */
    __GETC  = 0x20,
    /* Write char to the console display */
    __OUT   = 0x21,
    /* Write string, one char per word */
    __PUTS  = 0x22,
    /* Read char from keyboard and echo onto terminal */
    __IN    = 0x23,
    /* Write string, one char per byte, two bytes per word */
    __PUTSP = 0x24,
    /* Halt execution and print msg on console */
    __HALT  = 0x25
};

/* Device/Memory Mapped Registers (see MEMORY MAP) */
enum
{
    __KBSR = 0xFE00, /* Keyboard Status, has any key been pressed?   */
    __KBDR = 0xFE02, /* Keyboard Data,   what key has been pressed?  */
    __DSR  = 0xFE04, /* Display Status,  display ready to print?     */
    __DDR  = 0xFE06, /* Display Data,    display written char        */
    __MCR  = 0xFEEE, /* Machine Control, clock enable/disable        */
};


static uint16_t
memread(uint16_t addr)
{
    if (addr == __KBSR)
    {
        if (checkkey())
        {
        }
    }
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

static int
readimage(const char *filepath)
{
    FILE *file;
    uint16_t *p; /* Pointer to the memory location for writing file data. */
    uint16_t addr_origin; /* start address from the file */
    uint16_t maxread; /* maxread ensures bounds */
    size_t read; /* Number of 16-bit words read from the file */

    file = fopen(filepath, "rb");
    if (!file) return -1; 

    /* Read the origin address (2 bytes) from the file. */
    fread(&addr_origin, sizeof(addr_origin), 1, file);
    /* LC-3 is Big Endian, host is not */
    addr_origin = swap16(addr_origin); 

    /* Max number of words that can be read without overflowing memory */
    maxread = MAX_MEM - addr_origin;

    /* Set the pointer to the start of memory region for the image file */
    p = mem + addr_origin;

    /* Read up to maxread 16-bit words from the file into memory */
    read = fread(p, sizeof(uint16_t), maxread, file);

    /* Convert each word read to the correct byte order */
    while (read-- > 0)
    {
        *p = swap16(*p); 
        ++p; 
    }

    fclose(file); 
    return 0; 
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
        if (readimage(argv[j]) < 0)
        {
            printf("failed to load image: %s\n", argv[j]);
            exit(1);
        }
    }

    /* Initial state: ZERO flag */
    reg[FLAGS] = FZRO;
    
    /* Set inital Program Counter */
    enum { PCSTART = 0x3000 }; /* Userspace code origin (see MEMORY MAP) */
    reg[PC] = PCSTART;

    alive = 1; /* set to 0 by HALT (Trap) */
    while (alive)
    {
        /* FETCH */
        uint16_t instr = memread(reg[PC]++); /* read 16-bit instr */
        uint16_t op    = instr >> 12;        /* bits[15:12] set 4-bit opcode */
        switch (op) /* DECODE */
        {
        case ADD:
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

            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case AND:
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

            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case NOT:
        {
            uint16_t DR, SR;

            /* Destinarion Register, bits[11:9] */
            DR = (instr >> 9) & 0x7;
            /* Source Register, bits[8:6] */
            SR = (instr >> 6) & 0x7;
            /* Complement, invert all bits */
            reg[DR] = ~reg[SR];
            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case LD:
        {
            /**
             * NOTE(M): Indirect addressing vs. PC-relative addressing
             * LD is limited to a 9-bit offset relative to the Program Counter,
             * with an addressable range of [PC-256, PC+255]. Thus, LD cannot
             * directly load "far away" values in memory.
             * LDI, however, dereferences an intermediate memory address within
             * this range, which can store a 16-bit pointer to any location in 
             * memory. By dereferencing twice, LDI can access any value stored 
             * anywhere.
             * 
             * Caveat: The address of the "far away" value must be stored in 
             * memory within the PC-relative range.
             */

            uint16_t DR, PCoffset9;
            /* Destination Register, bits[11:9] */
            DR = (instr >> 9) & 0x7;
            /* Offset, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Load value from PC+offset addr */
            reg[DR] = memread(reg[PC] + PCoffset9);
            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case LDI:
        {
            /* see NOTE(M): Indirect addressing vs. PC-relative addressing */
            uint16_t DR, PCoffset9;
            /* Destination Register, bits [11:9] */
            DR = (instr >> 9) & 0x7;
            /* Offset, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Load addr (instead of value like in LD) of PC+offset */
            reg[DR] = memread(memread(reg[PC] + PCoffset9));
            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case LDR:
        {
            /**
             * NOTE(M): Register-relative addresing (see LDI for related idea) 
             * Abstracts PC-relative addressing; load any General
             * Purpose register with an address and use that as a base for the
             * offset.
             * range -> [BaseR-32, BaseR+31]
             * 
             * Useful to traverse arrays or any data structure (stack, etc.)
             * where placing elements away from a base is common.
             */
            uint16_t DR, BaseR, offset6;
            /* Destination Register, bits[11:9] */
            DR = (instr >> 9) & 0x7;
            /* Base Register, bits[8:6] */
            BaseR = (instr >> 6) & 0x7;
            /* Offset from BaseR, bits[5:0] */
            offset6 = signextend(instr & 0x3F, 6);
            /* Load contents of Base Register+Offset */
            reg[DR] = memread(reg[BaseR] + offset6);
            updateflags(DR); /* set Processsor FLAGS */
            break;
        }
        case LEA:
        {
            /* Load address of a LABEL */
            uint16_t DR, PCoffset9;

            /* Destination Register, bits[11:9] */
            DR = (instr >> 9) & 0x7; 
            /* Offset, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Load address instead of content (no memread) */
            reg[DR] = reg[PC] + PCoffset9;
            updateflags(DR); /* set Processor FLAGS */
            break;
        }
        case ST:
        {
            uint16_t SR, PCoffset9;
            
            /* Source Register, bits[11:9] */
            SR = (instr >> 9) & 0x7;
            /* Offset, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Write SR content into Offset addr */
            memwrite(reg[PC] + PCoffset9, reg[SR]);
            break;
        }
        case STI:
        {
            /* see NOTE(M): Indirect addressing vs. PC-relative addressing */
            uint16_t SR, PCoffset9;
            /* Source Register, bits[11:9] */
            SR = (instr >> 9) & 0x7;
            /* Offset, bits[8:0] */
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Store address of SR's contents in PC+Offset */
            memwrite(memread(reg[PC] + PCoffset9), reg[SR]);
            break;
        }
        case STR:
        {
            /* see NOTE(M): Register-relative addresing */
            uint16_t SR, BaseR, offset6;
            /* Source Register, bits[11:9] */
            SR = (instr >> 9) & 0x7;
            /* Base Register, bits[8:6] */
            BaseR = (instr >> 6) & 0x7;
            /* Offset, bits[5:0] */
            offset6 = signextend(instr & 0x3F, 6);
            /* Store SR's contents in BaseR+offset */
            memwrite(reg[BaseR] + offset6, reg[SR]);
            break;
        }
        case BR:
        {
            uint16_t flagstate, PCoffset9;
            /* Offset, bits[8:0]*/
            PCoffset9 = signextend(instr & 0x1FF, 9);
            /* Procesor Flags, bits[11:9] */
            flagstate = (instr >> 9) & 0x7; 
            if (flagstate & reg[FLAGS])
            {
                /**
                 * Before the following assignment, the Program Counter's 
                 * current value is the BR instruction we are decoding.
                 *
                 * The next instruction to be decoded (in the next fetch cycle)
                 * would be the one we are storing in this assignment.
                 * 
                 */
                reg[PC] += PCoffset9;
            }
            break;
        }
        case JMP: /* case RET: */
        {
            /**
             * Unconditional jump to the address specified by BaseR.
             *
             * The JMP instruction simply loads the Program Counter (PC)
             * with the value in the register BaseR, allowing an unconditional
             * branch to any memory location.
             *
             * The RET instruction is a specific form of JMP where BaseR is R7.
             * In subroutine calls, the current PC is stored in R7 
             * (linkage register) by JSR or JSRR. When a subroutine completes,
             * RET is used to restore the PC from R7, effectively returning to
             * the instruction immediately following the subroutine call.
             *
             * This makes RET an implied return mechanism for subroutine calls.
             */
            uint16_t BaseR;
            /* Base Register, bits[8:6] */
            BaseR = (instr >> 6) & 0x7;
            reg[PC] = reg[BaseR];
            break;
        }
        case JSR:
        {
            /**
             * Jump to Subroutine (JSR) instruction:
             * Saves the current PC into R7 (linkage register) to enable return
             * to the calling location.
             * Determines the target address of the subroutine based on the
             * addressing mode:
             * - If bit[11] is set, the address is computed using PC + offset11
             * - Otherwise, the address is taken from the specified BaseR
             */
            uint16_t BaseR, PCoffset11;
            reg[R7] = reg[PC]; /* Save current PC into R7 for linkage */

            if ((instr >> 11) & 0x1) /* Addressing mode: PC-relative */ {
                /* Offset, bits[10:0] */
                PCoffset11 = signextend(instr & 0x7FF, 11);
                reg[PC] += PCoffset11;
            }
            else
            {
                /* Base Register, bits[8:6] */
                BaseR = (instr >> 6) & 0x7;
                reg[PC] = reg[BaseR]; /* Address from BaseR */
            }
            break;
        }
        case TRAP: 
        {
            uint16_t trapvect8;

            /**
             * Save current PC into R7, after Trap Routine is handled code
             * jumps back to the **next** instruction after the Trap, unless
             * the Trap overwrites this default behaviour.
             */
            reg[R7] = reg[PC];

            /**
             * Trap Vector, bits[7:0] 
             * LC-3 ISA requires trapvect8 to be zero extended to 16 bits. 
             * This is automatically achieved here by defining `trapvect8` as 
             * an unsigned 16-bit integer.
             */
            trapvect8 = instr & 0xFF; 

            switch (trapvect8)
            {
                case __GETC:
                {
                    /* ASCII code copied into R0 */
                    reg[R0] = (uint16_t)getchar();
                    updateflags(R0);
                    break;
                }
                case __PUTS:
                {
                    
                    break;
                }
                case __PUTSP:
                {
                    break;
                }
                case __IN:
                {
                    break;
                }
                case __OUT:
                {
                    break;
                }
                case __HALT:
                {
                    break;
                }
            }
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


