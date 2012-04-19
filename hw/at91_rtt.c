/*
 * AT91 Real-time Timer
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
#include "qemu-timer.h"

#define RTT_SIZE        0x10

#define RTT_MR          0x00 /* Mode Register */
#define RTT_AR          0x04 /* Alarm Register */
#define RTT_VR          0x08 /* Value Register */
#define RTT_SR          0x0c /* Status Register */

#define MR_ALMIEN       0x10000
#define MR_RTTINCIEN    0x20000
#define MR_RTTRST       0x40000

#define SR_ALMS         0x01
#define SR_RTTINC       0x02

typedef struct RTTState {
    SysBusDevice busdev;
    MemoryRegion rtt_regs_region;
    qemu_irq irq;
    ptimer_state *timer;
    uint32_t mr;
    uint32_t ar;
    uint32_t vr;
    uint32_t sr;
} RTTState;

static void at91_rtt_tick(void *opaque)
{
    RTTState *s = opaque;

    s->vr++;
    s->sr |= SR_RTTINC;
    if (s->ar != ~0 && s->vr == s->ar + 1) {
        s->sr |= SR_ALMS;
    }
    if (((s->sr & SR_RTTINC) && (s->mr & MR_RTTINCIEN)) ||
        ((s->sr & SR_ALMS) && (s->mr & MR_ALMIEN))) {
        qemu_set_irq(s->irq, 1);
    }
}

static uint64_t at91_rtt_mem_read(void *opaque, target_phys_addr_t offset, 
      unsigned size)
{
    RTTState *s = opaque;
    uint32_t sr;

    offset &= RTT_SIZE - 1;
    switch (offset) {
    case RTT_MR:
        return s->mr;
    case RTT_AR:
        return s->ar;
    case RTT_VR:
        return s->vr;
    case RTT_SR:
        sr = s->sr;
        qemu_set_irq(s->irq, 0);
        s->sr = 0;
        return sr;
    default:
        return 0;
    }
}

static void at91_rtt_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    RTTState *s = opaque;

    offset &= RTT_SIZE - 1;
    switch (offset) {
    case RTT_MR:
        if (value & MR_RTTRST) {
            s->vr = 0;
            if ((value & 0xffff) == 0) {
                ptimer_set_freq(s->timer, 1);
            } else {
                ptimer_set_freq(s->timer, 0x8000 / (value & 0xffff));
            }
        }
        s->mr = value;
        break;
    case RTT_AR:
        s->ar = value;
        break;
    }
}

static const MemoryRegionOps at91_rtt_mmio_ops = {
    .read = at91_rtt_mem_read,
    .write = at91_rtt_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void at91_rtt_reset(DeviceState *d)
{
    RTTState *s = container_of(d, RTTState, busdev.qdev);

    s->mr = 0x8000;
    s->ar = ~0;
    s->vr = 0;
    s->sr = 0;
}

static int at91_rtt_init(SysBusDevice *dev)
{
    RTTState *s = FROM_SYSBUS(typeof (*s), dev);
    QEMUBH *bh;

    bh = qemu_bh_new(at91_rtt_tick, s);
    s->timer = ptimer_init(bh);
    ptimer_set_freq(s->timer, 1);
    ptimer_set_limit(s->timer, 1, 1);
    ptimer_run(s->timer, 0);

    sysbus_init_irq(dev, &s->irq);

    memory_region_init_io(&s->rtt_regs_region, &at91_rtt_mmio_ops, s, 
            "at91,rtt", RTT_SIZE);
    sysbus_init_mmio(dev, &s->rtt_regs_region);

    return 0;
}

static const VMStateDescription vmstate_at91_rtt = {
    .name = "at91,rtt",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32(mr, RTTState),
        VMSTATE_UINT32(ar, RTTState),
        VMSTATE_UINT32(vr, RTTState),
        VMSTATE_UINT32(sr, RTTState),
        VMSTATE_PTIMER(timer, RTTState),
        VMSTATE_END_OF_LIST()
    }
};

static void at91_rtt_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_rtt_init;
    dc->reset = at91_rtt_reset;
    dc->vmsd = &vmstate_at91_rtt;
}

static TypeInfo at91_rtt_info = {
    .name  = "at91,rtt",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(RTTState),
    .class_init    = at91_rtt_class_init,
};

static void at91_rtt_register_types(void)
{
    type_register_static(&at91_rtt_info);
}

type_init(at91_rtt_register_types)
