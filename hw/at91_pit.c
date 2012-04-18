/*
 * AT91 Periodic Interval Timer
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
#include "ptimer.h"
#include "at91.h"

#define PIT_SIZE        0x10

#define PIT_MR          0x00 /* Mode Register */
#define PIT_SR          0x04 /* Status Register */
#define PIT_PIVR        0x08 /* Periodic Interval Value Register */
#define PIT_PIIR        0x0c /* Periodic Interval Image Register */

#define PIT_LIMIT(s) \
    (((s)->mr & 0xfffff) + 1)

typedef struct PITState {
    SysBusDevice busdev;
    MemoryRegion pit_regs_region;
    qemu_irq irq;
    ptimer_state *timer;
    uint32_t mr;
    uint32_t sr;
    uint32_t picnt;
} PITState;

static void at91_pit_tick(void *opaque)
{
    PITState *s = opaque;

    s->sr |= 1;
    s->picnt++;
    if (s->mr & 0x2000000) {
        qemu_set_irq(s->irq, 1);
    }
}

static uint64_t at91_pit_mem_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
    PITState *s = opaque;
    uint32_t picnt = s->picnt;

    offset &= 0xf;
    switch (offset) {
    case PIT_MR:
        return s->mr;
    case PIT_SR:
        return s->sr;
    case PIT_PIVR:
        s->sr = 0;
        s->picnt = 0;
        qemu_set_irq(s->irq, 0);
        /* Fall-through */
    case PIT_PIIR:
        return
            ((PIT_LIMIT(s) - ptimer_get_count(s->timer)) & 0xfffff) |
            (picnt << 20);

    default:
        return 0;
    }
}

static void at91_pit_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    PITState *s = opaque;

    offset &= 0xf;
    if (offset == PIT_MR) {
        s->mr = value;
        if (value & 0x1000000) {
            ptimer_set_freq(s->timer, at91_master_clock_frequency / 16);
            ptimer_set_limit(s->timer, PIT_LIMIT(s), 1);
            ptimer_run(s->timer, 0);
        } else {
            ptimer_stop(s->timer);
            /*
              "After the PIT Enable bit is reset (PITEN= 0), the CPIV goes on counting until
              the PIV value is reached, and is then reset"
              Do not wait, set CPIV to right value now.
            */
            ptimer_set_count(s->timer, PIT_LIMIT(s));
        }
    }
}

static const MemoryRegionOps at91_pit_mmio_ops = {
    .read = at91_pit_mem_read,
    .write = at91_pit_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void at91_pit_reset(DeviceState *d)
{
    PITState *s = container_of(d, PITState, busdev.qdev);

    s->mr = 0xfffff;
    s->sr = 0;
    s->picnt = 0;
    ptimer_stop(s->timer);
}

static int at91_pit_init(SysBusDevice *dev)
{
    PITState *s = FROM_SYSBUS(typeof (*s), dev);
    QEMUBH *pit_bh;

    pit_bh = qemu_bh_new(at91_pit_tick, s);
    s->timer = ptimer_init(pit_bh);

    sysbus_init_irq(dev, &s->irq);

    memory_region_init_io(&s->pit_regs_region, &at91_pit_mmio_ops, s,
                          "at91,pmc", PIT_SIZE);
    sysbus_init_mmio(dev, &s->pit_regs_region);

    return 0;
}

static const VMStateDescription vmstate_at91_pit = {
    .name = "at91,pit",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mr, PITState),
        VMSTATE_UINT32(sr, PITState),
        VMSTATE_UINT32(picnt, PITState),
        VMSTATE_PTIMER(timer, PITState),
        VMSTATE_END_OF_LIST()
    }
};

static void at91_pit_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_pit_init;
    dc->reset = at91_pit_reset;
    dc->vmsd = &vmstate_at91_pit;
}

static TypeInfo at91_pit_info = {
    .name = "at91,pit",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(PITState),
    .class_init = at91_pit_class_init,
};

static void at91_pit_register_types(void)
{
    type_register_static(&at91_pit_info);
}

type_init(at91_pit_register_types)
