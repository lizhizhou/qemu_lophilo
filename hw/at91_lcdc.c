/*
 * AT91 SAM9 LCD Controller (LCDC)
 * Written by Evgeniy Dushistov
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "console.h"
#include "pixel_ops.h"

#define LCDC_SIZE 0x100000

#define LCDC_DMABADDR1     0x0
#define LCDC_DMAFRMCFG     0x18
#define LCDC_DMACON        0x1C
#define LCDC_LCDCON1       0x800
#define LCDC_LCDCON2       0x804
#define LCDC_LCDFRMCFG     0x810
#define LCDC_PWRCON        0x83C
#define LCDC_LUT_ENTRY_0   0xC00
#define LCDC_LUT_ENTRY_255 0xFFC

#define DMACON_DMAEN 1

typedef struct LCDCState {
    SysBusDevice busdev;
    MemoryRegion lcdc_regs_region;
    qemu_irq irq;
    DisplayState *ds;
    uint32_t dmacon;
    uint32_t pwrcon;
    uint32_t dmafrmcfg;
    uint32_t lcdcon1;
    uint32_t lcdcon2;
    uint32_t lcdfrmcfg;
    uint32_t dmabaddr1;
    uint16_t lut[256];    
} LCDCState;

//#define AT91_LCDC_DEBUG
#ifdef AT91_LCDC_DEBUG
#define DPRINTF(fmt, ...)                           \
    do {                                            \
        printf("AT91LCD: " fmt , ## __VA_ARGS__);    \
    } while (0)
#else
#define DPRINTF(fmt, ...) do { } while (0)
#endif


static uint64_t at91_lcdc_mem_read(void *opaque, target_phys_addr_t offset,
        unsigned size)
{
    LCDCState *s = opaque;

    offset &= LCDC_SIZE - 1;
    DPRINTF("read from %X\n", offset);
    switch (offset) {
    case LCDC_PWRCON:
        return s->pwrcon;
    case LCDC_DMACON:
        return s->dmacon;
    case LCDC_DMAFRMCFG:
        return s->dmafrmcfg;
    case LCDC_LCDCON1:
        return s->lcdcon1;
    case LCDC_LCDCON2:
        return s->lcdcon2;
    case LCDC_LCDFRMCFG:
        return s->lcdfrmcfg;
    default:
        DPRINTF("unsup. read\n");
        return 0;
    }
}

static void at91_lcdc_mem_write(void *opaque, target_phys_addr_t offset,
                uint64_t value, unsigned size)
{
    LCDCState *s = opaque;
    offset &= LCDC_SIZE - 1;
    DPRINTF("write to %X: %X\n", offset, value);

    switch (offset) {
    case LCDC_DMABADDR1:
        DPRINTF("dma addr %X\n", value);
        s->dmabaddr1 = value;
        break;
    case LCDC_PWRCON:
        s->pwrcon = value;
        break;
    case LCDC_DMACON:
        s->dmacon = value;
        break;
    case LCDC_DMAFRMCFG:
        s->dmafrmcfg = value;
        break;
    case LCDC_LCDCON1:
        s->lcdcon1 = value;
        break;
    case LCDC_LCDCON2:
        DPRINTF("pixel size %u\n", (value >> 5) & 7);
        s->lcdcon2 = value;
        break;
     case LCDC_LCDFRMCFG:
        DPRINTF("lineval %u, linsize %u\n", value & 0x7FF, (value >> 21) & 0x7FF);
        qemu_console_resize(s->ds, ((value >> 21) & 0x7FF) + 1 + 2, (value & 0x7FF) + 1 + 2);
        s->lcdfrmcfg = value;
        break;
    case LCDC_LUT_ENTRY_0 ... LCDC_LUT_ENTRY_255:
        //only the first 16 bits used
        s->lut[(offset - LCDC_LUT_ENTRY_0) / sizeof(uint32_t)] = value & 0xFFFF;
        DPRINTF("lut[%u] = %X\n", (offset - LCDC_LUT_ENTRY_0) / sizeof(uint32_t), s->lut[(offset - LCDC_LUT_ENTRY_0) / sizeof(uint32_t)]);
        break;
    default:
        DPRINTF("unsup. write\n");
    }
}

static void at91_lcdc_reset(DeviceState *d)
{
    LCDCState *s = container_of(d, LCDCState, busdev.qdev);
    s->pwrcon = 0x0000000e;
    s->dmacon = 0;
    s->dmafrmcfg = 0;
    s->lcdcon1 = 0x00002000;
    s->lcdfrmcfg = 0;
}

struct pixel8 {
    unsigned b : 2;
    unsigned g : 3;
    unsigned r : 3;
};

union pixel8u {
    struct pixel8 p;
    uint8_t val;
};

struct pixel16 {
    unsigned b : 5;
    unsigned g : 5;
    unsigned r : 5;
};

union pixel16u {
    struct pixel16 p;
    uint8_t bytes[0];
    uint16_t val;
};


static void at91_lcdc_update_display(void *opaque)
{
    LCDCState *s = opaque;
    uint32_t color;
    int x, y;
    uint8_t *d;
    int width = ((s->lcdfrmcfg >> 21) & 0x7FF) + 1;
    int height = (s->lcdfrmcfg & 0x7FF) + 1;
    int q_bpp = (ds_get_bits_per_pixel(s->ds) + 7) >> 3;
    int bpp;
    int bpp_idx = (s->lcdcon2 >> 5) & 7;
    union pixel8u tmp8;
    union pixel16u tmp16;
    unsigned int r, g, b;

//    DPRINTF("update begin\n");
    if (!(s->dmacon & DMACON_DMAEN))
        return;
//    DPRINTF("update continue\n");
    switch (bpp_idx) {
    case 0 ... 4:
        bpp = 1 << bpp_idx;
        break;
    case 5 ... 6:
        bpp = 24;
        break;
    default:
        fprintf(stderr, "Unknown pixel size\n");
        return;//reserved value, unknown pixel size
    }
    /*TODO: fix this restriction*/
    if (bpp != 8 && bpp != 16) {
        fprintf(stderr, "Unsupported pixel size: %d\n", bpp);
        return;
    }

    //int once = 0;
    for (y = 0; y < height; ++y) {        
        for (x = 0; x < width; ++x) {
            if (bpp == 8) {
                uint32_t dmabaddr1 = s->dmabaddr1/* - 0x10000000*/;
                cpu_physical_memory_read(dmabaddr1 + width * y + x, &tmp8.val, 1);

                tmp16.val =  s->lut[tmp8.val];
                r = tmp16.p.r;
                g = tmp16.p.g;
                b = tmp16.p.b;
            } else {
                uint32_t dmabaddr1 = s->dmabaddr1/* - 0x10000000*/;
                cpu_physical_memory_read(dmabaddr1 + width * y * 2 + x * 2, &tmp16.bytes[0], 2);
                r = tmp16.p.r;
                g = tmp16.p.g;
                b = tmp16.p.b;
            }
#if 0
            if (tmp.val != 0 && once == 0) {
                once = 1;
                DPRINTF("not null %X, %X, %X, bpp %d\n", (unsigned)r << 5, (unsigned)g << 5, (unsigned)b << 6,
                        ds_get_bits_per_pixel(s->ds));
            }
#endif
            switch (ds_get_bits_per_pixel(s->ds)) {
            case 8:
                color = rgb_to_pixel8(r, g, b);
                break;
            case 15:
                color = rgb_to_pixel15(r, g, b);
                break;
            case 16:
                color = rgb_to_pixel16(r, g, b);
                break;
            case 24:
                color = rgb_to_pixel24(r, g, b);
                break;
            case 32:
                if (bpp == 8) {
                    color = rgb_to_pixel32bgr((unsigned)r << 3, (unsigned)g << 3, (unsigned)b << 3);
                } else {
                    color = rgb_to_pixel32bgr((unsigned)r << 3, (unsigned)g << 3, (unsigned)b << 3);
                }
                break;
            default:
                return;
            }

            d = ds_get_data(s->ds) + ds_get_linesize(s->ds) * (y + 1) + q_bpp * (x + 1);
            memcpy(d, &color, ds_get_bits_per_pixel(s->ds) / 8);
        }
    }

    dpy_update(s->ds, 0, 0, width + 2, height + 2);
}

static const MemoryRegionOps at91_lcdc_mmio_ops = {
    .read = at91_lcdc_mem_read,
    .write = at91_lcdc_mem_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int at91_lcdc_init(SysBusDevice *dev)
{
    LCDCState *s = FROM_SYSBUS(typeof(*s), dev);

    sysbus_init_irq(dev, &s->irq);
    memory_region_init_io(&s->lcdc_regs_region, &at91_lcdc_mmio_ops, s,
            "at91,lcdc", LCDC_SIZE);
    sysbus_init_mmio(dev, &s->lcdc_regs_region);    

    s->ds = graphic_console_init(at91_lcdc_update_display, NULL, NULL, NULL, s);

    return 0;
}

static void at91_lcdc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = at91_lcdc_init;
    dc->reset = at91_lcdc_reset;
}

static TypeInfo at91_lcdc_info = {
    .name  = "at91,lcdc",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(LCDCState),
    .class_init    = at91_lcdc_class_init,
};

static void at91_lcdc_register_types(void)
{
    type_register_static(&at91_lcdc_info);
}

type_init(at91_lcdc_register_types)
