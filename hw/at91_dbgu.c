/*
 * AT91 Debug Unit
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

/* TODO: Channel mode, ICE pins, FIFO?, overrun, retransmission */

#include "hw.h"
#include "sysbus.h"
#include "qemu-char.h"
#include "at91.h"

//#define DEBUG_DBGU

#define DBGU_SIZE       0x200

#define DBGU_CR         0x00 /* Control Register */
#define DBGU_MR         0x04 /* Mode Register */
#define DBGU_IER        0x08 /* Interrupt Enable Register */
#define DBGU_IDR        0x0c /* Interrupt Disable Register */
#define DBGU_IMR        0x10 /* Interrupt Mask Register */
#define DBGU_SR         0x14 /* Channel Status Register */
#define DBGU_RHR        0x18 /* Receiver Holding Register */
#define DBGU_THR        0x1c /* Transmitter Holding Register */
#define DBGU_BRGR       0x20 /* Baud Rate Generator Register */
#define DBGU_CIDR       0x40 /* Chip ID Register */
#define DBGU_EXID       0x44 /* Chip ID Extension Register */
#define DBGU_FNR        0x48 /* Force NTRST Register */

#define SR_RXRDY        0x01 /* Reciever Ready */
#define SR_TXRDY        0x02 /* Transmitter Ready */
#define SR_ENDRX        0x08 /* End of Receive Transfer */
#define SR_ENDTX        0x10 /* End of Transmit */
#define SR_OVRE         0x20 /* Overrun */
#define SR_FRAME        0x40 /* Framing Error */
#define SR_PARE         0x80 /* Parity Error */
#define SR_TXEMPTY      0x200 /* Transmitter Empty */
#define SR_TXBUFE       0x800 /* Transmission Buffer Empty */
#define SR_RXBUFF       0x1000 /* Receiver Buffer Full */
#define SR_COMM_TX      0x40000000
#define SR_COMM_RX      0x80000000

#define CR_RXEN         0x10
#define CR_RXDIS        0x20
#define CR_TXEN         0x40
#define CR_TXDIS        0x80
#define CR_RSTSTA       0x100

#define DEFAULT_CHIPID  0x275b0940

typedef struct DBGUState {
    SysBusDevice busdev;
    MemoryRegion ser_regs_region;
    CharDriverState *chr;
    qemu_irq irq;
    uint32_t chipid;
    uint32_t chipid_ext;

    uint8_t rx_enabled;
    uint8_t tx_enabled;
    uint32_t mr;
    uint32_t imr;
    uint32_t sr;
    uint8_t rhr;
    uint8_t thr;
    uint16_t brgr;
    uint32_t fnr;
    PDCState *pdc_state;
} DBGUState;

static void at91_dbgu_update_irq(DBGUState *s)
{
    qemu_set_irq(s->irq, !!(s->sr & s->imr));
}

static void at91_dbgu_update_parameters(DBGUState *s)
{
    QEMUSerialSetParams ssp;

    if (s->brgr == 0)
        return;
    if (s->brgr == 1)
        ssp.speed = at91_master_clock_frequency / s->brgr;
    else
        ssp.speed = at91_master_clock_frequency / (16 * s->brgr);
    ssp.parity = 'O';
    ssp.data_bits = 8;
    ssp.stop_bits = 1;
    qemu_chr_fe_ioctl(s->chr, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp);
}

static void at91_dbgu_receive(void *opaque, const uint8_t *buf, int size)
{
    DBGUState *s = opaque;
#ifdef DEBUG_DBGU
    printf("%s: recieve, size %d, char `%c'\n", __func__, size, size ? buf[0] : 0);
#endif
    s->sr |= SR_RXRDY/* | SR_RXBUFF*/;
    s->rhr = (buf[0] & 0xff);
    if (at91_pdc_byte_in(s->pdc_state, buf[0] & 0xFF) == 0)
        at91_dbgu_update_irq(s);
    else
        s->sr &= ~SR_RXRDY;
}

static int at91_dbgu_can_receive(void *opaque)
{
    DBGUState *s = opaque;
#if 0/*def DEBUG_DBGU*/
    printf("%s: rx_en %d, rdy %d\n", __func__, s->rx_enabled,
           !(s->sr & SR_RXRDY));
#endif
    if (s->rx_enabled && !(s->sr & SR_RXRDY))
        return 1;
    return 0;
}

static void at91_dbgu_event(void *opaque, int event)
{
}

static uint64_t at91_dbgu_mem_read(void *opaque, target_phys_addr_t offset, unsigned size)
{
    DBGUState *s = opaque;
    uint32_t value;

    offset &= DBGU_SIZE - 1;
    switch (offset) {
    case DBGU_MR:
        return s->mr;
    case DBGU_IMR:
        return s->imr;
    case DBGU_SR:
        return s->sr;
    case DBGU_RHR:
        value = s->rhr;
        s->sr &= ~(SR_RXRDY/* | SR_RXBUFF*/);
        at91_dbgu_update_irq(s);
        return value;
    case DBGU_BRGR:
        return s->brgr;
    case DBGU_CIDR:
        return s->chipid;
    case DBGU_EXID:
        return s->chipid_ext;
    case DBGU_FNR:
        return s->fnr;
    case 0x100 ... 0x124:
        return at91_pdc_read(s->pdc_state, offset);
    default:
        return 0;
    }
}

static void at91_dbgu_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    DBGUState *s = opaque;
    unsigned char ch = value;

    offset &= DBGU_SIZE - 1;
    switch (offset) {
    case DBGU_CR:
        if (value & CR_RXDIS) {
            s->rx_enabled = 0;
        } else if (value & CR_RXEN) {
            s->rx_enabled = 1;
        }
        if (value & CR_TXDIS) {
            s->tx_enabled = 0;
        } else if (value & CR_TXEN) {
            s->tx_enabled = 1;
        }
        if (value & CR_RSTSTA) {
            s->sr &= ~(SR_PARE | SR_FRAME | SR_OVRE);
        }
        break;
    case DBGU_MR:
        s->mr = value;
        break;
    case DBGU_IER:
        s->imr |= value;
        break;
    case DBGU_IDR:
        s->imr &= ~value;
        break;
    case DBGU_THR:
        if (s->tx_enabled)
        {
            /* TODO: shift register, error checking */
            s->thr = value;
            qemu_chr_fe_write(s->chr, &ch, 1);
            s->sr |= SR_TXRDY | SR_TXBUFE | SR_TXEMPTY;
        }
        break;
    case DBGU_BRGR:
        s->brgr = value;
        at91_dbgu_update_parameters(s);
        break;
    case DBGU_FNR:
        s->fnr = value;
        break;
    case 0x100 ... 0x124:
        at91_pdc_write(s->pdc_state, offset, value);
    default:
        return;
    }

    at91_dbgu_update_irq(s);
}

#ifdef DEBUG_DBGU
static uint64_t at91_dbgu_mem_read_dbg(void *opaque, target_phys_addr_t offset, unsigned size)
{
    uint32_t value = at91_dbgu_mem_read(opaque, offset, size);
    printf("%s offset=%x val=%x\n", __func__, offset & (DBGU_SIZE - 1), value);
    return value;
}

static void at91_dbgu_mem_write_dbg(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    printf("%s offset=%x val=%x\n", __func__, offset & (DBGU_SIZE - 1), value);
    at91_dbgu_mem_write(opaque, offset, value, size);
}

#define at91_dbgu_mem_read at91_dbgu_mem_read_dbg
#define at91_dbgu_mem_write at91_dbgu_mem_write_dbg
#endif

static void at91_dbgu_reset(DeviceState *d)
{
    DBGUState *s = container_of(d, DBGUState, busdev.qdev);

    s->rx_enabled = 0;
    s->tx_enabled = 0;
    s->mr = 0;
    s->imr = 0;
    /* Transmitter begins ready and idle */
    s->sr = SR_TXRDY | SR_TXBUFE | SR_TXEMPTY;
    s->rhr = 0;
    s->thr = 0;
    s->brgr = 0;
    s->fnr = 0;
    at91_pdc_reset(s->pdc_state);
}

static int pdc_start_transfer(void *opaque,
                              target_phys_addr_t tx,
                              unsigned int *tx_len,
                              target_phys_addr_t rx,
                              unsigned int *rx_len,
                              int last_transfer)
{
    DBGUState *s = opaque;
    uint8_t tmp;
    unsigned int i;
#ifdef DEBUG_DBGU
    printf("%s: begin, tx_len %u, rx_len %u\n", __func__, *tx_len, *rx_len);
#endif
    //TODO: implement receiving
    if (*rx_len && *tx_len == 0) {
        if (s->sr & SR_RXRDY)  {
            tmp = s->rhr & 0xFF;
            printf("%s: we save `%c' to %X\n", __func__, tmp, rx);
            cpu_physical_memory_write(rx, &tmp, 1);
            s->sr &= ~SR_RXRDY;
            --*rx_len;
            return 0;
        } 
        return -1;
    }
    for (i = 0; i < *tx_len; ++i) {
        cpu_physical_memory_read(tx + i, &tmp, 1);
        at91_dbgu_mem_write(s, DBGU_THR, tmp, 1);
    }
    *tx_len = 0;

    return 0;
}

static void pdc_state_changed(void *opaque, unsigned int state)
{
    DBGUState *s = opaque;

#ifdef DEBUG_DBGU
    printf("%s: begin\n", __func__);
#endif
    if (state & PDCF_ENDRX) {
        printf("%s: clear rxdry\n", __func__);
        s->sr |=  SR_ENDRX;
//        s->sr &= ~(SR_RXRDY /*| SR_RXBUFF*/);
    }
    s->sr &= ~((state & PDCF_NOT_ENDRX) ? SR_ENDRX : 0);
    s->sr |= (state & PDCF_ENDTX) ? SR_ENDTX : 0;
    s->sr &= ~((state & PDCF_NOT_ENDTX) ? SR_ENDTX : 0);

    s->sr |= (state & PDCF_RXFULL) ? SR_RXBUFF : 0;
    s->sr &= ~((state & PDCF_NOT_RXFULL) ? SR_RXBUFF : 0);

    s->sr |= (state & PDCF_TXFULL) ? SR_TXBUFE : 0;
    s->sr &= ~((state & PDCF_NOT_TXFULL) ? SR_TXBUFE : 0);


#ifdef DEBUG_DBGU
    printf("%s: sr %X\n", __func__, s->sr);
#endif

    at91_dbgu_update_irq(s);
}


static const MemoryRegionOps at91_dbgu_mmio_ops = {
    .read = at91_dbgu_mem_read,
    .write = at91_dbgu_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int at91_dbgu_init(SysBusDevice *dev)
{
    DBGUState *s = FROM_SYSBUS(typeof (*s), dev);

    s->pdc_state = at91_pdc_init(s, pdc_start_transfer, pdc_state_changed);
    sysbus_init_irq(dev, &s->irq);
    memory_region_init_io(&s->ser_regs_region, &at91_dbgu_mmio_ops, s,
            "at91,dbgu", DBGU_SIZE);
    sysbus_init_mmio(dev, &s->ser_regs_region);

    qemu_chr_add_handlers(s->chr, at91_dbgu_can_receive,
                          at91_dbgu_receive, at91_dbgu_event, s);
    return 0;
}

static const VMStateDescription vmstate_at91_dbgu = {
    .name = "at91,dbgu",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
//TODO: save pdc state
    .fields      = (VMStateField[]) {
        VMSTATE_UINT8(rx_enabled, DBGUState),
        VMSTATE_UINT8(tx_enabled, DBGUState),
        VMSTATE_UINT32(mr, DBGUState),
        VMSTATE_UINT32(imr, DBGUState),
        VMSTATE_UINT32(sr, DBGUState),
        VMSTATE_UINT8(rhr, DBGUState),
        VMSTATE_UINT8(thr, DBGUState),
        VMSTATE_UINT16(brgr, DBGUState),
        VMSTATE_UINT32(fnr, DBGUState),
        VMSTATE_END_OF_LIST()
    }
};

static Property at91_dbgu_properties[] = {
    DEFINE_PROP_CHR("chardev", DBGUState, chr),
    DEFINE_PROP_UINT32("chipid", DBGUState, chipid, DEFAULT_CHIPID),
    DEFINE_PROP_UINT32("chipid-ext", DBGUState, chipid_ext, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void at91_dbgu_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_dbgu_init;
    dc->reset = at91_dbgu_reset;
    dc->props = at91_dbgu_properties;
    dc->vmsd = &vmstate_at91_dbgu;
}

static TypeInfo at91_dbgu_info = {
    .name  = "at91,dbgu",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(DBGUState),
    .class_init    = at91_dbgu_class_init,
};

static void at91_dbgu_register_types(void)
{
    type_register_static(&at91_dbgu_info);
}

type_init(at91_dbgu_register_types)
