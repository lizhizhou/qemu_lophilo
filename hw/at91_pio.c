/*
 * AT91 Parallel I/O Controller
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

/* TODO: Glitch-filter, multi-driver (ie. open drain) */

#include "sysbus.h"

/*
 * Input/Output GPIO pins:
 * 32x PIO device
 * 32x peripheral A
 * 32x peripheral B
 */

#define PIO_SIZE        0x200
#define PIO_PINS        32

#define PIO_PER         0x00 /* PIO Enable Register */
#define PIO_PDR         0x04 /* PIO Disable Register */
#define PIO_PSR         0x08 /* PIO Status Register */
#define PIO_OER         0x10 /* Output Enable Register */
#define PIO_ODR         0x14 /* Output Disable Register */
#define PIO_OSR         0x18 /* Output Status Register */
#define PIO_IFER        0x20 /* Input Filter Enable Register */
#define PIO_IFDR        0x24 /* Input Filter Disable Register */
#define PIO_IFSR        0x28 /* Input Filter Status Register */
#define PIO_SODR        0x30 /* Set Output Data Register */
#define PIO_CODR        0x34 /* Clear Output Data Register */
#define PIO_ODSR        0x38 /* Output Data Status Register */
#define PIO_PDSR        0x3c /* Pin Data Status Register */
#define PIO_IER         0x40 /* Interrupt Enable Register */
#define PIO_IDR         0x44 /* Interrupt Disable Register */
#define PIO_IMR         0x48 /* Interrupt Mask Register */
#define PIO_ISR         0x4c /* Interrupt Status Register */
#define PIO_MDER        0x50 /* Multi-driver Enable Register */
#define PIO_MDDR        0x54 /* Multi-driver Disable Register */
#define PIO_MDSR        0x58 /* Multi-driver Status Register */
#define PIO_PPUDR       0x60 /* Pull-up Disable Register */
#define PIO_PPUER       0x64 /* Pull-up Enable Register */
#define PIO_PPUSR       0x68 /* Pull-up Status Register */
#define PIO_ASR         0x70 /* Select A Register */
#define PIO_BSR         0x74 /* Select B Register */
#define PIO_ABSR        0x78 /* AB Select Status Register */
#define PIO_OWER        0xa0 /* Output Write Enable Register */
#define PIO_OWDR        0xa4 /* Output Write Disable Register */
#define PIO_OWSR        0xa8 /* Output Write Status Register */

typedef struct PIOState {
    SysBusDevice busdev;
    MemoryRegion pio_regs_region;
    qemu_irq out[PIO_PINS * 3];
    qemu_irq parent_irq;
    uint32_t psr;
    uint32_t osr;
    uint32_t ifsr;
    uint32_t odsr;
    uint32_t pdsr;
    uint32_t imr;
    uint32_t isr;
    uint32_t mdsr;
    uint32_t ppusr;
    uint32_t absr;
    uint32_t owsr;
    /* Mask of unknown state of PIO pins, needed for pull-up resistor
       implementation */
    uint32_t unknown_state;
} PIOState;

static void at91_pio_set_pin(void *opaque, int pin, int level)
{
    PIOState *s = opaque;
    int mask = 1 << (pin % PIO_PINS);
    int input_set = pin / PIO_PINS;
    int output_set = !!(s->absr & mask);
    uint32_t saved_pdsr;

    if (input_set == 0) {
        /* PIO pin -> Peripheral / IO */

        /* Skip input if output mode is enabled for the pin */
        if (s->osr & mask)
            return;

        if (s->psr & mask) {
            saved_pdsr = s->pdsr;
            s->pdsr &= ~mask;
            if (level == -1) {
                s->unknown_state |= mask;
            } else if (level) {
                s->unknown_state &= ~mask;
                s->pdsr |= mask;
            }
            if (saved_pdsr != s->pdsr) {
                s->isr |= mask;
                qemu_set_irq(s->parent_irq, !!(s->isr & s->imr));
            }
        } else {
            qemu_set_irq(s->out[PIO_PINS + (output_set * PIO_PINS)], level);
        }
    } else {
        /* Peripheral -> PIO pin */
        if ((~s->psr & mask) && input_set == output_set) {
            qemu_set_irq(s->out[pin & PIO_PINS], level);
        }
    }
}

static uint64_t at91_pio_mem_read(void *opaque, target_phys_addr_t offset, 
    unsigned size)
{
    PIOState *s = opaque;
    int isr;

    offset &= PIO_SIZE - 1;
    switch (offset) {
    case PIO_PSR:
        return s->psr;
    case PIO_OSR:
        return s->osr;
    case PIO_IFSR:
        return s->ifsr;
    case PIO_ODSR:
        return s->odsr;
    case PIO_PDSR:
        return
            (s->pdsr & ~s->unknown_state) |
            (s->ppusr & s->unknown_state);
    case PIO_IMR:
        return s->imr;
    case PIO_ISR:
        isr = s->isr;
        s->isr = 0;
        qemu_set_irq(s->parent_irq, 0);
        return isr;
    case PIO_MDSR:
        return s->mdsr;
    case PIO_PPUSR:
        return s->ppusr;
    case PIO_ABSR:
        return s->absr;
    case PIO_OWSR:
        return s->owsr;
    default:
        return 0;
    }
}

static void at91_pio_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    PIOState *s = opaque;
    int i;

    offset &= PIO_SIZE - 1;
    switch (offset) {
    case PIO_PER:
        s->psr |= value;
        break;
    case PIO_PDR:
        s->psr &= ~value;
        break;
    case PIO_OER:
        s->osr |= value;
        break;
    case PIO_ODR:
        s->osr &= ~value;
        break;
    case PIO_IFER:
        s->ifsr |= value;
        break;
    case PIO_IFDR:
        s->ifsr &= ~value;
        break;
    case PIO_SODR:
        s->odsr |= value;
        for (i = 0; i < PIO_PINS; i++)
            if (value & (1 << i) & s->osr)
                qemu_set_irq(s->out[i], 1);
        break;
    case PIO_CODR:
        s->odsr &= ~value;
        for (i = 0; i < PIO_PINS; i++)
            if (value & (1 << i) & s->osr)
                qemu_set_irq(s->out[i], 0);
        break;
    case PIO_ODSR:
        s->odsr = (s->odsr & ~s->owsr) | (value & s->owsr);
        for (i = 0; i < PIO_PINS; i++)
            if (s->owsr & (1 << i))
                qemu_set_irq(s->out[i], !!(value & (1 << i)));
        break;
    case PIO_IER:
        s->imr |= value;
        break;
    case PIO_IDR:
        s->imr &= ~value;
        break;
    case PIO_MDER:
        s->mdsr |= value;
        break;
    case PIO_MDDR:
        s->mdsr &= ~value;
        break;
    case PIO_PPUER:
        s->ppusr |= value;
        break;
    case PIO_PPUDR:
        s->ppusr &= ~value;
        break;
    case PIO_ASR:
        s->absr &= ~value;
        break;
    case PIO_BSR:
        s->absr |= value;
        break;
    case PIO_OWER:
        s->owsr |= value;
        break;
    case PIO_OWDR:
        s->owsr &= ~value;
        break;
    default:
        return;
    }
}

static void at91_pio_reset(DeviceState *d)
{
    PIOState *s = container_of(d, PIOState, busdev.qdev);

    s->psr = 0xffffffff;
    s->osr = 0;
    s->ifsr = 0;
    s->odsr = 0;
    s->pdsr = 0;
    s->imr = 0;
    s->isr = 0;
    s->mdsr = 0;
    s->ppusr = 0;
    s->absr = 0;
    s->owsr = 0;
    s->unknown_state = 0xffffffff;
}

static const MemoryRegionOps at91_pio_mmio_ops = {
    .read = at91_pio_mem_read,
    .write = at91_pio_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int at91_pio_init(SysBusDevice *dev)
{
    PIOState *s = FROM_SYSBUS(typeof (*s), dev);
    int pio_regs;

    sysbus_init_irq(dev, &s->parent_irq);
    qdev_init_gpio_in(&dev->qdev, at91_pio_set_pin, PIO_PINS * 3);
    qdev_init_gpio_out(&dev->qdev, s->out, PIO_PINS * 3);

    memory_region_init_io(&s->pio_regs_region, &at91_pio_mmio_ops, s,
            "at91,pio", PIO_SIZE);
    sysbus_init_mmio(dev, &s->pio_regs_region);
    return 0;
}

static const VMStateDescription vmstate_at91_pio = {
    .name = "at91,pio",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(psr, PIOState),
        VMSTATE_UINT32(osr, PIOState),
        VMSTATE_UINT32(ifsr, PIOState),
        VMSTATE_UINT32(odsr, PIOState),
        VMSTATE_UINT32(pdsr, PIOState),
        VMSTATE_UINT32(imr, PIOState),
        VMSTATE_UINT32(isr, PIOState),
        VMSTATE_UINT32(mdsr, PIOState),
        VMSTATE_UINT32(ppusr, PIOState),
        VMSTATE_UINT32(absr, PIOState),
        VMSTATE_UINT32(owsr, PIOState),
        VMSTATE_UINT32(unknown_state, PIOState),
        VMSTATE_END_OF_LIST()
    }
};

static int at91_pio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_pio_init;
    dc->reset = at91_pio_reset;
    dc->vmsd = &vmstate_at91_pio;

    return 0;
}

static TypeInfo at91_pio_info = {
    .name          = "at91,pio",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PIOState),
    .class_init    = at91_pio_class_init,
};

static void at91_pio_register_types(void)
{
    type_register_static(&at91_pio_info);
}

type_init(at91_pio_register_types)
