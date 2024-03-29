#include <stdio.h>

#include "nand.h"
#include "at91.h"

typedef struct NandState {
    NANDFlashState *nand_state;
} NandState;

//#define AT91_NAND_DEBUG
#ifdef AT91_NAND_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91NAND: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif

extern CPUState *g_env;

static uint64_t at91_nand_mem_read(void *opaque, target_phys_addr_t offset,
    unsigned size)
{
    NandState *s = opaque;
    DeviceState *d = &s->nand_state->busdev.qdev;

    uint8_t res = nand_getio(d);
    DPRINTF("(IP %X) read from %X (res %X)\n", g_env->regs[15], offset, res);
    return res;
}

static void at91_nand_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    NandState *s = opaque;        
    DeviceState *d = &s->nand_state->busdev.qdev;

    nand_setpins(d, offset & (1 << 22), offset & (1 << 21), 0, 1, 0);
    DPRINTF("(IP %X) write to %X %X\n", g_env->regs[15], offset, value);
    nand_setio(d, value & 0xFF);
}

static const MemoryRegionOps at91_nand_mmio_ops = {
    .read = at91_nand_mem_read,
    .write = at91_nand_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

void at91_nand_register(NANDFlashState *st)
{
    NandState *s;
    //int iomemtype;

    s = g_malloc(sizeof(*s));
    s->nand_state = st;
    //memory_region_init_io(&s->nand_regs_region, &at91_nand_mmio_ops, s, 
    //        "at91,nand", NAND_SIZE);

    // TODO: don't know what the equivalent registration command is here?
    //cpu_register_physical_memory(0x40000000, 0x10000000, iomemtype);
}


