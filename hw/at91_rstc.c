/*
 * AT91 Reset Controller
 *
 * Copyright (c) 2009 Filip Navara
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "sysbus.h"

/* #define DEBUG_RSTC */

#define RSTC_SIZE        0x10

#define RSTC_CR          0x00 /* Control Register */
#define RSTC_SR          0x04 /* Status Register */
#define RSTC_MR          0x08 /* Mode Register */

#define WRITE_KEY        0xa5

#define CR_PROCRST       0x01 /* Processor Reset */
#define CR_PERRST        0x04 /* Peripheral Reset */

#define SR_URSTS         0x01 /* User Reset Status */
#define SR_BODSTS        0x02 /* Brownout Detection Status */
#define SR_NRSTL         0x10000 /* NRST Level */

typedef struct RSTCState {
    SysBusDevice busdev;
    MemoryRegion rstc_regs_region;
    uint32_t sr;
    uint32_t mr;
} RSTCState;

static uint64_t at91_rstc_mem_read(void *opaque, target_phys_addr_t offset, 
        unsigned size)
{
    RSTCState *s = opaque;

    offset &= RSTC_SIZE - 1;
    switch (offset) {
    case RSTC_SR:
        return s->sr;
    case RSTC_MR:
        return s->mr;
    default:
        return 0;
    }
}

static void at91_rstc_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    RSTCState *s = opaque;

    if ((value >> 24) != WRITE_KEY)
        return;

    offset &= RSTC_SIZE - 1;
    switch (offset) {
    case RSTC_CR:
        /* TODO */
        break;
    case RSTC_MR:
        s->mr = value;
        break;
    }
}

#ifdef DEBUG_RSTC
static uint32_t at91_rstc_mem_read_dbg(void *opaque, target_phys_addr_t offset,
        unsigned size)
{
    uint32_t value = at91_rstc_mem_read(opaque, offset);
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    return value;
}

static void at91_rstc_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint32_t value, unsigned size)
{
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    at91_rstc_mem_write(opaque, offset, value);
}

#define at91_rstc_mem_read at91_rstc_mem_read_dbg
#define at91_rstc_mem_write at91_rstc_mem_write_dbg
#endif

static const MemoryRegionOps at91_rstc_mmio_ops = {
    .read = at91_rstc_mem_read,
    .write = at91_rstc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void at91_rstc_reset(void *opaque)
{
    RSTCState *s = opaque;

    s->sr = SR_NRSTL;
    s->mr = 0;
}

static const VMStateDescription vmstate_at91_rstc = {
    .name = "at91,rstc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(sr, RSTCState),
        VMSTATE_UINT32(mr, RSTCState),
        VMSTATE_END_OF_LIST()
    }
};

static int at91_rstc_init(SysBusDevice *dev)
{
    RSTCState *s = FROM_SYSBUS(typeof (*s), dev);

    memory_region_init_io(&s->rstc_regs_region, &at91_rstc_mmio_ops, s, 
            "at91,rstc", RSTC_SIZE);

    sysbus_init_mmio(dev, &s->rstc_regs_region);

    return 0;
}


static void at91_rstc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_rstc_init;
    dc->reset = at91_rstc_reset;
    dc->vmsd = &vmstate_at91_rstc;
}

static TypeInfo at91_rstc_info = {
    .name  = "at91,rstc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(RSTCState),
    .class_init    = at91_rstc_class_init,
};

static void at91_rstc_register_types(void)
{
    type_register_static(&at91_rstc_info);
}

type_init(at91_rstc_register_types)
