/*
 * AT91 Power Management Controller
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
#include "qemu-timer.h"
#include "at91.h"

//#define AT91_PMC_DEBUG
#ifdef AT91_PMC_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91PMC: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif


#define PMC_SIZE        0x70

#define PMC_SCER        0x00 /* System Clock Enable Register */
#define PMC_SCDR        0x04 /* System Clock Disable Register */
#define PMC_SCSR        0x08 /* System Clock Status Register */
#define PMC_PCER        0x10 /* Peripheral Clock Enable Register */
#define PMC_PCDR        0x14 /* Peripheral Clock Disable Register */
#define PMC_PCSR        0x18 /* Peripheral Clock Status Register */
#define PMC_MOR         0x20 /* Main Oscillator Register */
#define PMC_MCFR        0x24 /* Main Clock Frequency Register */
#define PMC_PLLA        0x28 /* PLL A Register */
#define PMC_PLLB        0x2c /* PLL B Register */
#define PMC_MCKR        0x30 /* Master Clock Register */
#define PMC_PCKR        0x40 /* Programmable Clock Register */
#define PMC_IER         0x60 /* Interrupt Enable Register */
#define PMC_IDR         0x64 /* Interrupt Disable Register */
#define PMC_IMR         0x6c /* Interrupt Mask Register */
#define PMC_SR          0x68 /* Status Register */

#define SR_MOSCS        0x01
#define SR_LOCKA        0x02
#define SR_LOCKB        0x04
#define SR_MCKRDY       0x08
#define SR_PCK0RDY      0x100
#define SR_PCK1RDY      0x200
#define SR_PCK2RDY      0x400
#define SR_PCK3RDY      0x800

#define SO_FREQ         32768

int at91_master_clock_frequency = SO_FREQ;

typedef struct PMCState {
    SysBusDevice busdev;
    MemoryRegion pmc_regs_region;
    qemu_irq parent_irq;
    uint32_t scsr;
    uint32_t pcsr;
    uint32_t mor;
    uint32_t plla;
    uint32_t pllb;
    uint32_t mckr;
    uint32_t pckr[4];
    uint32_t sr;
    uint32_t imr;
    uint32_t mck_freq;
    uint32_t mo_freq;
} PMCState;

static void at91_pmc_update_irq(PMCState *s)
{
    qemu_set_irq(s->parent_irq, !!(s->sr & s->imr));
}

static void at91_update_master_clock(PMCState *s)
{
    int mck_freq = s->mo_freq;

    /* Clock selection */
    switch (s->mckr & 3) {
    case 0: /* Slow */
        mck_freq = SO_FREQ;
        break;
    case 1: /* Main */
        if (!(s->sr & SR_MOSCS))
            mck_freq = 0;
        break;
    case 2: /* PLL A */
        if ((s->plla & 0xff) != 0 &&
            (s->plla & 0x3ff80) != 0) {
            mck_freq /= s->plla & 0xff;
            mck_freq *= ((s->plla >> 16) & 0x7ff) + 1;
        } else {
            mck_freq = 0;
        }
        break;
    case 3: /* PLL B */
        if ((s->pllb & 0xff) != 0 &&
            (s->pllb & 0x3ff80) != 0) {
            mck_freq /= s->pllb & 0xff;
            mck_freq *= ((s->pllb >> 16) & 0x7ff) + 1;
        } else {
            mck_freq = 0;
        }
        break;
    }

    if (mck_freq != 0) {
        mck_freq /= 1 << ((s->mckr >> 2) & 7);
        mck_freq /= 1 << ((s->mckr >> 8) & 3);
        s->mck_freq = mck_freq;
        at91_master_clock_frequency = mck_freq;
        s->sr |= SR_MCKRDY;
    } else {
        s->sr &= ~SR_MCKRDY;
    }
}

static uint64_t at91_pmc_mem_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
    PMCState *s = opaque;
    DPRINTF("read %X\n", offset);
    switch (offset) {
    case PMC_PLLA:
        return s->plla;
    case PMC_PLLB:
        return s->pllb;
    case PMC_SCSR:
        return s->scsr;
    case PMC_PCSR:
        return s->pcsr;
    case PMC_MOR:
        return s->mor;
    case PMC_MCFR:
        if (s->mor & 1)
            return (1 << 16) | (s->mo_freq / SO_FREQ / 16);
        return 0;
    case PMC_PCKR ... PMC_PCKR + 15:
        return s->pckr[(offset - PMC_PCKR) >> 2];
    case PMC_SR:
        DPRINTF("read SR, val %X\n", s->sr);
        return s->sr;
    case PMC_IMR:
        return s->imr;
    case PMC_MCKR:
        return s->mckr;
    default:
        DPRINTF("unsup. read\n");
        return 0;
    }
}

static void at91_pmc_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    PMCState *s = opaque;
    DPRINTF("set %X to %X\n", offset, value);
    switch (offset) {
    case PMC_SCER:
        s->scsr |= value & 0xf80;
        break;
    case PMC_SCDR:
        s->scsr &= ~(value & 0xf80);
        break;
    case PMC_PCER:
        s->pcsr |= value & ~3;
        break;
    case PMC_PCDR:
        s->pcsr &= ~(value & ~3);
        break;
    case PMC_MOR:
        /* Main Oscillator bypassing is not supported, so first two
           bits are ignored. Bits 8-15 specify the OSCOUNT, which is
           also currently ignored. */
        DPRINTF("write to mor\n");
        s->mor = value;
        s->sr |= SR_MOSCS;
        break;
    case PMC_PLLA:
        s->plla = value;
        /* OUTA, PLLACOUNT ignored for now */
        DPRINTF("set plla\n");
        s->sr |= SR_LOCKA;
        break;
    case PMC_PLLB:
        DPRINTF("set pllb\n");
        s->pllb = value;
        /* OUTB, PLLBCOUNT ignored for now */
        s->sr |= SR_LOCKB;
        break;
    case PMC_MCKR:
        s->mckr = value;
        break;
    case PMC_PCKR ... PMC_PCKR + 15:
        s->pckr[(offset - PMC_PCKR) >> 2] = value;
        break;
    case PMC_IER:
        s->imr |= value;
        break;
    case PMC_IDR:
        s->imr &= ~value;
        break;
    default:
        DPRINTF("unsup. write\n");
        return;
    }

    at91_update_master_clock(s);
    at91_pmc_update_irq(s);
}


static const MemoryRegionOps at91_pmc_mmio_ops = {
    .read = at91_pmc_mem_read,
    .write = at91_pmc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};


static int at91_pmc_post_load(void *opaque, int version_id)
{
    PMCState *s = opaque;
    at91_update_master_clock(s);
    return 0;
}

static void at91_pmc_reset(DeviceState *d)
{
    PMCState *s = container_of(d, PMCState, busdev.qdev);;

    s->scsr = 1;
    s->pcsr = 0;
    s->mor = 0;
    s->plla = s->pllb = 0x3f00;
    s->mckr = 0;
    s->pckr[0] = s->pckr[1] = s->pckr[2] = s->pckr[3] = 0;
    s->sr = 8;
    s->imr = 0;
    s->mck_freq = SO_FREQ;
}

static int at91_pmc_init(SysBusDevice *dev)
{
    PMCState *s = FROM_SYSBUS(typeof (*s), dev);

    sysbus_init_irq(dev, &s->parent_irq);

    memory_region_init_io(&s->pmc_regs_region, &at91_pmc_mmio_ops, s,
                          "at91,pmc", PMC_SIZE);
    sysbus_init_mmio(dev, &s->pmc_regs_region);

    return 0;
}

static const VMStateDescription vmstate_at91_pmc = {
    .name = "at91,pmc",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .post_load = at91_pmc_post_load,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(scsr, PMCState),
        VMSTATE_UINT32(pcsr, PMCState),
        VMSTATE_UINT32(mor, PMCState),
        VMSTATE_UINT32(plla, PMCState),
        VMSTATE_UINT32(pllb, PMCState),
        VMSTATE_UINT32(mckr, PMCState),
        VMSTATE_UINT32_ARRAY(pckr, PMCState, 4),
        VMSTATE_UINT32(sr, PMCState),
        VMSTATE_UINT32(imr, PMCState),
        VMSTATE_UINT32(mo_freq, PMCState),
        VMSTATE_END_OF_LIST()
    }
};

static Property at91_pmc_properties[] = {
    DEFINE_PROP_UINT32("mo_freq", PMCState, mo_freq, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void at91_pmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_pmc_init;
    dc->reset = at91_pmc_reset;
    dc->props = at91_pmc_properties;
    dc->vmsd = &vmstate_at91_pmc;
}

static TypeInfo at91_pmc_info = {
    .name = "at91,pmc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PMCState),
    .class_init = at91_pmc_class_init,
};

static void at91_pmc_register_types(void)
{
    type_register_static(&at91_pmc_info);
}

type_init(at91_pmc_register_types)
