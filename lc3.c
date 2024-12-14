/**
* LC-3
* word size = 16 bits
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
* Mem addr range
* 0000..FFFF
*/

enum { MAX_MEM = (1 << 16) }; /* 2**16 -> 65536 */

/**
* Registers
* 16 bit width
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
    PC,     /* progam counter  */
    FLAGS,  /* processor flags */
    RCOUNT,
};

enum {
    FPOS = 1 << 0,  /* 1 (0001), positive */
    FZRO = 1 << 1,  /* 2 (0010), zero     */
    FNEG = 1 << 2   /* 4 (0100), negative */
};

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

uint16_t mem[MAX_MEM];
uint16_t reg[RCOUNT];

/*
*  struct lc3 {
*          struct memory       mem;
*          struct registers    reg;    
*          struct flags        fl;
*          struct opcodes      op;
*  };
*/

static uint16_t
mem_read(uint16_t r);

static uint16_t
read_image(const char *filepath);


static inline uint16_t
sign_extend(uint16_t x, int bit_count)
{
    /**
    * in order to extend a two's complement number
    * we must preserve the sign by repeating the MSB
    * in all the (new) extra bits, i.e., 1100 -> 1111 1100 
    *
    * we only extend the negative numbers (MSB = 1)
    * positive number extension is given
    */
    if ((x >> (bit_count - 1)) & 1) { /* check MSB == 1 */

        x |= (0xFFFF << bit_count); /* set to 1 all extra bits */
    }

    return x;
}


static inline uint16_t
swap16(uint16_t x)
{
    return (x << 8) | (x >> 8);
    
}

static void 
update_flags(const uint16_t r)
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
        if (!read_image(argv[j]))
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
        uint16_t instr = mem_read(reg[PC]++); /* 16 bit instruction */
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
            abort();
            break;
        }

        }
    }

    return 0;
}
