/*
 * AT91 Advanced Interrupt Controller.
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

/* TODO: Inverting external sources based on SRCTYPE in SMR register */

/* #define DEBUG_AIC */

#define AIC_SIZE        0x200

#define AIC_SMR         0x000 /* Source Mode Register */
#define AIC_SVR         0x080 /* Source Vector Register */
#define AIC_IVR         0x100 /* IRQ Vector Register */
#define AIC_FVR         0x104 /* FIQ Vector Register */
#define AIC_ISR         0x108 /* Interrupt Status Register */
#define AIC_IPR         0x10c /* Interrupt Pending Register */
#define AIC_IMR         0x110 /* Interrupt Mask Register */
#define AIC_CISR        0x114 /* Core Interrupt Status Register */
#define AIC_IECR        0x120 /* Interrupt Enable Command Register */
#define AIC_IDCR        0x124 /* Interrupt Disable Command Register */
#define AIC_ICCR        0x128 /* Interrupt Clear Command Register */
#define AIC_ISCR        0x12c /* Interrupt Set Command Register */
#define AIC_EOICR       0x130 /* End of Interrupt Command Register */
#define AIC_SPU         0x134 /* Spurious Vector Register */
#define AIC_DCR         0x138 /* Debug Control Register (Protect) */
#define AIC_FFER        0x140 /* Fast Forcing Enable Register */
#define AIC_FFDR        0x144 /* Fast Forcing Disable Register */
#define AIC_FFSR        0x148 /* Fast Forcing Status Register */

#define DCR_PROT        0x01 /* Protect Mode Enabled */
#define DCR_GMSK        0x02 /* General Mask */

#define AIC_GET_PRIORITY(s, irq) \
    ((int)((s)->smr[(irq)] & 7))
#define AIC_IS_EDGE_TRIGGERED(s, irq) \
    ((int)((s)->smr[(irq)] & (1 << 5)))

typedef struct AICState {
    SysBusDevice busdev;
    MemoryRegion regs_region;
    qemu_irq parent_irq;
    qemu_irq parent_fiq;
    uint32_t smr[32];
    uint32_t svr[32];
    uint32_t ipr;
    uint32_t imr;
    uint32_t cisr;
    uint32_t spu;
    uint32_t dcr;
    uint32_t ffsr;
    uint8_t stack_irq[8];
    uint8_t stack_pri[8];
    int8_t stack_pos;
} AICState;

static uint8_t at91_aic_highest(struct AICState *s, uint8_t *priority)
{
    uint32_t pending_interrupts;
    uint8_t highest_interrupt = 0;
    uint8_t highest_priority = 0;
    int i;

    pending_interrupts = s->ipr & s->imr & ~s->ffsr;
    for (i = 31; i >= 1; i--) {
        if (pending_interrupts & (1 << i)) {
            if (AIC_GET_PRIORITY(s, i) >= highest_priority) {
                highest_interrupt = i;
                highest_priority = AIC_GET_PRIORITY(s, i);
            }
        }
    }

    *priority = highest_priority;
    return highest_interrupt;
}

static void at91_aic_update(AICState *s)
{
    uint32_t requests = s->ipr & s->imr;
    uint32_t fiq_mask = 1 | s->ffsr;

    if (s->dcr & DCR_GMSK) {
        s->cisr = 0;
    } else {
        /* Fast interrupts */
        s->cisr = !!(requests & fiq_mask);
        /* Priority-driven normal interrupts */
        if (requests & ~fiq_mask) {
            uint8_t highest_priority;
            at91_aic_highest(s, &highest_priority);
            if (s->stack_pos < 0 ||
                s->stack_pri[s->stack_pos] < highest_priority) {
                s->cisr |= 2;
            }
        }
    }

    qemu_set_irq(s->parent_fiq, s->cisr & 1);
    qemu_set_irq(s->parent_irq, s->cisr & 2);
}

static void at91_aic_set_irq(void *opaque, int irq, int level)
{
    struct AICState *s = (struct AICState *) opaque;
    int mask = 1 << irq;

    /* TODO: External egde-triggering */
    if (level)
        s->ipr |= mask;
    else if (!AIC_IS_EDGE_TRIGGERED(s, irq))
        s->ipr &= ~mask;

    at91_aic_update(s);
}

static inline void at91_aic_irq_enter(AICState *s, uint8_t irq, uint8_t priority)
{
    if (s->stack_pos < 7 && irq != 0) {
        s->stack_pos++;
        s->stack_irq[s->stack_pos] = irq;
        s->stack_pri[s->stack_pos] = priority;

        if (AIC_IS_EDGE_TRIGGERED(s, irq)) {
            s->ipr &= ~(1 << irq);
            at91_aic_update(s);
        }
    }
}

static uint64_t at91_aic_mem_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
    AICState *s = opaque;
    uint8_t current_irq;
    uint8_t current_pri;

    switch (offset) {
    case AIC_IVR: /* Interrupt vector register */
        current_irq = at91_aic_highest(s, &current_pri);
        if (!(s->dcr & DCR_PROT)) {
            at91_aic_irq_enter(s, current_irq, current_pri);
        }
        return current_irq == 0 ? s->spu : s->svr[current_irq];
    case AIC_FVR: /* FIQ vector register */
        if (s->ipr & 1) {
            s->ipr &= ~1;
            at91_aic_update(s);
            return s->svr[0];
        } else if (s->ipr & s->ffsr) {
            return s->svr[0];
        }
        return s->spu;
    case AIC_ISR: /* Interrupt status register */
        if (s->stack_pos < 0)
            return 0;
        return s->stack_irq[s->stack_pos];
    case AIC_IPR: /* Interrupt pending register */
        return s->ipr;
    case AIC_IMR: /* Interrupt mask register */
        return s->imr;
    case AIC_CISR: /* Core interrupt status register */
        return s->cisr;
    case AIC_SPU: /* Spurious interrupt vector register */
        return s->spu;
    case AIC_DCR:
        return s->dcr;
    case AIC_FFSR:
        return s->ffsr;
    case AIC_SMR ... AIC_SMR + 127:
        return s->smr[(offset - AIC_SMR) >> 2];
    case AIC_SVR ... AIC_SVR + 127:
        return s->svr[(offset - AIC_SVR) >> 2];
    default:
        return 0;
    }
}

static void at91_aic_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    AICState *s = opaque;
    uint8_t current_irq;
    uint8_t current_pri;
    int irq;

    switch (offset) {
    case AIC_IVR:
        if (s->dcr & DCR_PROT) {
            current_irq = at91_aic_highest(s, &current_pri);
            at91_aic_irq_enter(s, current_irq, current_pri);
        }
        break;
    case AIC_IECR:
        s->imr |= value;
        break;
    case AIC_IDCR:
        s->imr &= ~value;
        break;
    case AIC_ICCR:
        for (irq = 0; irq < 32; irq++) {
            if (!AIC_IS_EDGE_TRIGGERED(s, irq))
                value &= ~(1 << irq);
        }
        s->ipr &= value;
        break;
    case AIC_ISCR:
        for (irq = 0; irq < 32; irq++) {
            if (!AIC_IS_EDGE_TRIGGERED(s, irq))
                value &= ~(1 << irq);
        }
        s->ipr |= value;
        break;
    case AIC_EOICR: /* End of interrupt */
        if (s->stack_pos >= 0)
            s->stack_pos--;
        break;
    case AIC_SPU:
        s->spu = value;
        return;
    case AIC_DCR:
        s->dcr = value;
        break;
    case AIC_FFER:
        s->ffsr |= value;
        break;
    case AIC_FFDR:
        s->ffsr &= ~value;
        break;
    case AIC_SMR ... AIC_SMR + 127:
        s->smr[(offset - AIC_SMR) >> 2] = value;
        break;
    case AIC_SVR ... AIC_SVR + 127:
        s->svr[(offset - AIC_SVR) >> 2] = value;
        return;
    default:
        return;
    }

    at91_aic_update(s);
}

#ifdef DEBUG_AIC
static uint64_t at91_aic_mem_read_dbg(void *opaque, target_phys_addr_t offset)
{
    uint64_t value = at91_aic_mem_read(opaque, offset);
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    return value;
}

static void at91_aic_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint64_t value)
{
    printf("%s offset=%x val=%x\n", __func__, offset, value);
    at91_aic_mem_write(opaque, offset, value);
}

#define at91_aic_mem_read at91_aic_mem_read_dbg
#define at91_aic_mem_write at91_aic_mem_write_dbg
#endif

static void at91_aic_reset(DeviceState *d)
{
    AICState *s = container_of(d, AICState, busdev.qdev);
    int i;
    
    for (i = 0; i < 32; i++) {
        s->smr[i] = 0;
        s->svr[i] = 0;
    }
    s->ipr = 0;
    s->imr = 0;
    s->cisr = 0;
    s->spu = 0;
    s->dcr = 0;
    s->ffsr = 0;
    s->stack_pos = -1;
}

static const MemoryRegionOps at91_aic_mmio_ops = {
    .read = at91_aic_mem_read,
    .write = at91_aic_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int at91_aic_init(SysBusDevice *dev)
{
    AICState *s = FROM_SYSBUS(typeof (*s), dev);

    qdev_init_gpio_in(&dev->qdev, at91_aic_set_irq, 32);
    sysbus_init_irq(dev, &s->parent_irq);
    sysbus_init_irq(dev, &s->parent_fiq);

    memory_region_init_io(&s->regs_region, &at91_aic_mmio_ops, s,
            "at91,aic", AIC_SIZE);
    sysbus_init_mmio(dev, &s->regs_region);

    return 0;
}

static const VMStateDescription vmstate_at91_aic = {
    .name = "at91,aic",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(smr, AICState, 32),
        VMSTATE_UINT32_ARRAY(svr, AICState, 32),
        VMSTATE_UINT8_ARRAY(stack_irq, AICState, 8),
        VMSTATE_UINT8_ARRAY(stack_pri, AICState, 8),
        VMSTATE_INT8(stack_pos, AICState),
        VMSTATE_UINT32(ipr, AICState),
        VMSTATE_UINT32(imr, AICState),
        VMSTATE_UINT32(cisr, AICState),
        VMSTATE_UINT32(spu, AICState),
        VMSTATE_UINT32(dcr, AICState),
        VMSTATE_UINT32(ffsr, AICState),
        VMSTATE_END_OF_LIST()
    }
};

static void at91_aic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_aic_init;
    dc->reset = at91_aic_reset;
    dc->vmsd = &vmstate_at91_aic;
}

static TypeInfo at91_aic_info = {
    .name          = "at91,aic",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(AICState),
    .class_init    = at91_aic_class_init,
};

static void at91_aic_register_types(void)
{
    type_register_static(&at91_aic_info);
}

type_init(at91_aic_register_types)
