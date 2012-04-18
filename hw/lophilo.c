#include "sysbus.h"
#include "arm-misc.h"
#include "devices.h"
#include "net.h"
#include "sysemu.h"
#include "boards.h"
#include "pc.h"
#include "qemu-timer.h"
#include "ptimer.h"
#include "block.h"
#include "flash.h"
#include "console.h"
#include "i2c.h"
#include "blockdev.h"
#include "exec-memory.h"

#define LPL_RAM_DEFAULT_SIZE     128*1024*1024
#define LPL_SRAM_BASE            0x00300000
#define LPL_SRAM_SIZE            1*1024*1024
#define LPL_FLASH_SIZE_MAX       32*1024*1024

#define LPL_UART1_BASE           0x8000C840
#define LPL_UART2_BASE           0x8000C940

#define LPL_GPIO_BASE            0x8000D000
#define LPL_GPIO_SIZE            0x00001000

#define LPL_FLASHCFG_BASE        0x90006000
#define LPL_FLASHCFG_SIZE        0x00001000

//#define LPL_AUDIO_BASE           0x90007000

/* Periodic Interval Timer (PIT)
 * see Datasheet table 15-1 page 113
 * se Datasheet table 6-1 page 15
 */

// 0xFFFF FD30 PITC 
#define LPL_PIT_BASE             0xFFFFFD30
#define LPL_PIT_SIZE             16
// 0x00 Mode Register PIT_MR Read-write 0x000F_FFFF
// 0x04 Status Register PIT_SR Read-only 0x0000_0000
// 0x08 Periodic Interval Value Register PIT_PIVR Read-only 0x0000_0000
// 0x0C Periodic Interval Image Register PIT_PIIR Read-only 0x0000_0000 

/* timers 
 * see Datasheet table 8-1
 */
#define LPL_TIMER1_IRQ           4
#define LPL_TIMER2_IRQ           5
#define LPL_TIMER3_IRQ           6
#define LPL_TIMER4_IRQ           7

//#define LPL_LCD_BASE             0x9000c000
//#define LPL_LCD_SIZE             0x00001000

//#define LPL_EHCI_IRQ             8
//#define LPL_ETH_IRQ              9
#define LPL_UART1_IRQ            11
#define LPL_UART2_IRQ            11
#define LPL_GPIO_IRQ             12
#define LPL_RTC_IRQ              28

/* PIC register offsets 
    See Datasheet table 27-3 page 370
*/
#define LPL_PIC_BASE             0xFFFFF000 // 27.9.1 The AIC is mapped at the address 0xFFFF F000.
#define LPL_PIC_SIZE             4*1024 // 27.9.1 It has a total 4-KByte addressing space.
// 0x00 Source Mode Register 0 AIC_SMR0 Read-write 0x0
// 0x04 Source Mode Register 1 AIC_SMR1 Read-write 0x0
// --- --- --- --- ---
// 0x7C Source Mode Register 31 AIC_SMR31 Read-write 0x0
// 0x80 Source Vector Register 0 AIC_SVR0 Read-write 0x0
// 0x84 Source Vector Register 1 AIC_SVR1 Read-write 0x0
// --- --- --- --- ---
// 0xFC Source Vector Register 31 AIC_SVR31 Read-write 0x0
// 0x100 Interrupt Vector Register AIC_IVR Read-only 0x0
// 0x104 FIQ Interrupt Vector Register AIC_FVR Read-only 0x0
// 0x108 Interrupt Status Register AIC_ISR Read-only 0x0
#define LPL_PIC_STATUS           0x108  
// 0x10C Interrupt Pending Register(2) AIC_IPR Read-only 0x0(1)
// 0x110 Interrupt Mask Register(2) AIC_IMR Read-only 0x0
// 0x114 Core Interrupt Status Register AIC_CISR Read-only 0x0
// 0x118 - 0x11C Reserved --- --- ---
// 0x120 Interrupt Enable Command Register(2) AIC_IECR Write-only ---
#define LPL_PIC_ENABLE_SET       0x120 // 27-3 Interrupt Enable Command Register
// 0x124 Interrupt Disable Command Register(2) AIC_IDCR Write-only ---
// 0x128 Interrupt Clear Command Register(2) AIC_ICCR Write-only ---
#define LPL_PIC_ENABLE_CLR       0x128 // 27-3 Interrupt Clear Command Register
// 0x12C Interrupt Set Command Register(2) AIC_ISCR Write-only ---
// 0x130 End of Interrupt Command Register AIC_EOICR Write-only ---
// 0x134 Spurious Interrupt Vector Register AIC_SPU Read-write 0x0
// 0x138 Debug Control Register AIC_DCR Read-write 0x0
// 0x13C Reserved --- --- ---
// 0x140 Fast Forcing Enable Register(2) AIC_FFER Write-only ---
// 0x144 Fast Forcing Disable Register(2) AIC_FFDR Write-only ---
// 0x148 Fast Forcing Status Register(2) AIC_FFSR Read-only 0x0

/* PIT register offsets */
#define LPL_PIT_TIMER1_LENGTH    0x00
/* ... */
#define LPL_PIT_TIMER4_LENGTH    0x0C
#define LPL_PIT_CONTROL          0x10
#define LPL_PIT_TIMER1_VALUE     0x14
/* ... */
#define LPL_PIT_TIMER4_VALUE     0x20
#define LPL_BOARD_RESET          0x34

#define LPL_BOARD_RESET_MAGIC    0x10000

typedef struct sam9m10_timer_state {
    ptimer_state *ptimer;
    uint32_t limit;
    int freq;
    qemu_irq irq;
} sam9m10_timer_state;

typedef struct sam9m10_pit_state {
    SysBusDevice busdev;
    MemoryRegion iomem;
    sam9m10_timer_state timer[4];
} sam9m10_pit_state;

static void sam9m10_timer_tick(void *opaque)
{
    sam9m10_timer_state *s = opaque;

    qemu_irq_raise(s->irq);
}

static void sam9m10_timer_init(SysBusDevice *dev, sam9m10_timer_state *s,
                                 uint32_t freq)
{
    QEMUBH *bh;

    sysbus_init_irq(dev, &s->irq);
    s->freq = freq;

    bh = qemu_bh_new(sam9m10_timer_tick, s);
    s->ptimer = ptimer_init(bh);
}

static uint64_t sam9m10_pit_read(void *opaque, target_phys_addr_t offset,
                                   unsigned size)
{
    sam9m10_pit_state *s = opaque;
    sam9m10_timer_state *t;

    switch (offset) {
    case LPL_PIT_TIMER1_VALUE ... LPL_PIT_TIMER4_VALUE:
        t = &s->timer[(offset-LPL_PIT_TIMER1_VALUE) >> 2];
        return ptimer_get_count(t->ptimer);

    default:
        return 0;
    }
}

static void sam9m10_pit_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    sam9m10_pit_state *s = opaque;
    sam9m10_timer_state *t;
    int i;

    switch (offset) {
    case LPL_PIT_TIMER1_LENGTH ... LPL_PIT_TIMER4_LENGTH:
        t = &s->timer[offset >> 2];
        t->limit = value;
        if (t->limit > 0) {
            ptimer_set_limit(t->ptimer, t->limit, 1);
        } else {
            ptimer_stop(t->ptimer);
        }
        break;

    case LPL_PIT_CONTROL:
        for (i = 0; i < 4; i++) {
            t = &s->timer[i];
            if (value & 0xf && t->limit > 0) {
                ptimer_set_limit(t->ptimer, t->limit, 0);
                ptimer_set_freq(t->ptimer, t->freq);
                ptimer_run(t->ptimer, 0);
            } else {
                ptimer_stop(t->ptimer);
            }
            value >>= 4;
        }
        break;

    case LPL_BOARD_RESET:
        if (value == LPL_BOARD_RESET_MAGIC) {
            qemu_system_reset_request();
        }
        break;
    }
}

static void sam9m10_pit_reset(DeviceState *d)
{
    sam9m10_pit_state *s = FROM_SYSBUS(sam9m10_pit_state,
                                         sysbus_from_qdev(d));
    int i;

    for (i = 0; i < 4; i++) {
        ptimer_stop(s->timer[i].ptimer);
        s->timer[i].limit = 0;
    }
}

static const MemoryRegionOps sam9m10_pit_ops = {
    .read = sam9m10_pit_read,
    .write = sam9m10_pit_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int sam9m10_pit_init(SysBusDevice *dev)
{
    sam9m10_pit_state *s = FROM_SYSBUS(sam9m10_pit_state, dev);
    int i;

    /* Letting them all run at 1 MHz is likely just a pragmatic
     * simplification. */
    for (i = 0; i < 4; i++) {
        sam9m10_timer_init(dev, &s->timer[i], 1000000);
    }

    memory_region_init_io(&s->iomem, &sam9m10_pit_ops, s,
                          "lophilo-pit", LPL_PIT_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static const VMStateDescription sam9m10_timer_vmsd = {
    .name = "timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_PTIMER(ptimer, sam9m10_timer_state),
        VMSTATE_UINT32(limit, sam9m10_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription sam9m10_pit_vmsd = {
    .name = "sam9m10_pit",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(timer, sam9m10_pit_state, 4, 1,
                             sam9m10_timer_vmsd, sam9m10_timer_state),
        VMSTATE_END_OF_LIST()
    }
};

static void sam9m10_pit_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = sam9m10_pit_init;
    dc->reset = sam9m10_pit_reset;
    dc->vmsd = &sam9m10_pit_vmsd;
}

static TypeInfo sam9m10_pit_info = {
    .name          = "sam9m10_pit",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(sam9m10_pit_state),
    .class_init    = sam9m10_pit_class_init,
};


static struct arm_boot_info lophilo_binfo = {
    .loader_start = 0x0,
    .board_id = 0x20e,
};

static void lophilo_init(ram_addr_t ram_size,
               const char *boot_device,
               const char *kernel_filename, const char *kernel_cmdline,
               const char *initrd_filename, const char *cpu_model)
{
    CPUARMState *env;
    qemu_irq *cpu_pic;
    qemu_irq pic[32];
    DeviceState *dev;
    //DeviceState *i2c_dev;
    //DeviceState *lcd_dev;
    //DeviceState *key_dev;
    //DeviceState *wm8750_dev;
    //SysBusDevice *s;
    //i2c_bus *i2c;
    int i;
    unsigned long flash_size;
    DriveInfo *dinfo;
    MemoryRegion *address_space_mem = get_system_memory();
    MemoryRegion *ram = g_new(MemoryRegion, 1);
    MemoryRegion *sram = g_new(MemoryRegion, 1);

    if (!cpu_model) {
        cpu_model = "arm926";
    }
    env = cpu_init(cpu_model);
    if (!env) {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    cpu_pic = arm_pic_init_cpu(env);

    /* For now we use a fixed - the original - RAM size */
    memory_region_init_ram(ram, "lophilo.ram", LPL_RAM_DEFAULT_SIZE);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(address_space_mem, 0, ram);

    memory_region_init_ram(sram, "lophilo.sram", LPL_SRAM_SIZE);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(address_space_mem, LPL_SRAM_BASE, sram);

    dev = sysbus_create_simple("sam9m10_pic", LPL_PIC_BASE,
                               cpu_pic[ARM_PIC_CPU_IRQ]);
    for (i = 0; i < 32; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }
    sysbus_create_varargs("sam9m10_pit", LPL_PIT_BASE, pic[LPL_TIMER1_IRQ],
                          pic[LPL_TIMER2_IRQ], pic[LPL_TIMER3_IRQ],
                          pic[LPL_TIMER4_IRQ], NULL);

    if (serial_hds[0]) {
        serial_mm_init(address_space_mem, LPL_UART1_BASE, 2, pic[LPL_UART1_IRQ],
                       1825000, serial_hds[0], DEVICE_NATIVE_ENDIAN);
    }
    if (serial_hds[1]) {
        serial_mm_init(address_space_mem, LPL_UART2_BASE, 2, pic[LPL_UART2_IRQ],
                       1825000, serial_hds[1], DEVICE_NATIVE_ENDIAN);
    }

    /* Register flash */
    dinfo = drive_get(IF_PFLASH, 0, 0);
    if (dinfo) {
        flash_size = bdrv_getlength(dinfo->bdrv);
        if (flash_size != 8*1024*1024 && flash_size != 16*1024*1024 &&
            flash_size != 32*1024*1024) {
            fprintf(stderr, "Invalid flash image size\n");
            exit(1);
        }

        /*
         * The original U-Boot accesses the flash at 0xFE000000 instead of
         * 0xFF800000 (if there is 8 MB flash). So remap flash access if the
         * image is smaller than 32 MB.
         */
#ifdef TARGET_WORDS_BIGENDIAN
        pflash_cfi02_register(0-LPL_FLASH_SIZE_MAX, NULL,
                              "lophilo.flash", flash_size,
                              dinfo->bdrv, 0x10000,
                              (flash_size + 0xffff) >> 16,
                              LPL_FLASH_SIZE_MAX / flash_size,
                              2, 0x00BF, 0x236D, 0x0000, 0x0000,
                              0x5555, 0x2AAA, 1);
#else
        pflash_cfi02_register(0-LPL_FLASH_SIZE_MAX, NULL,
                              "lophilo.flash", flash_size,
                              dinfo->bdrv, 0x10000,
                              (flash_size + 0xffff) >> 16,
                              LPL_FLASH_SIZE_MAX / flash_size,
                              2, 0x00BF, 0x236D, 0x0000, 0x0000,
                              0x5555, 0x2AAA, 0);
#endif

    }
    //sysbus_create_simple("lophilo_flashcfg", LPL_FLASHCFG_BASE, NULL);

    //qemu_check_nic_model(&nd_table[0], "lophilo");
    //dev = qdev_create(NULL, "lophilo_eth");
    //qdev_set_nic_properties(dev, &nd_table[0]);
    //qdev_init_nofail(dev);
    //sysbus_mmio_map(sysbus_from_qdev(dev), 0, LPL_ETH_BASE);
    //sysbus_connect_irq(sysbus_from_qdev(dev), 0, pic[LPL_ETH_IRQ]);

    //sysbus_create_simple("lophilo_wlan", LPL_WLAN_BASE, NULL);

    //lophilo_misc_init(sysbus_from_qdev(dev));

    //dev = sysbus_create_simple("lophilo_gpio", LPL_GPIO_BASE, pic[LPL_GPIO_IRQ]);
    //i2c_dev = sysbus_create_simple("gpio_i2c", -1, NULL);
    //i2c = (i2c_bus *)qdev_get_child_bus(i2c_dev, "i2c");

    //lcd_dev = sysbus_create_simple("lophilo_lcd", LPL_LCD_BASE, NULL);
    //key_dev = sysbus_create_simple("lophilo_key", -1, NULL);

    /* I2C read data */
    //qdev_connect_gpio_out(i2c_dev, 0,
    //                      qdev_get_gpio_in(dev, LPL_GPIO_I2C_DATA_BIT));
    /* I2C data */
    //qdev_connect_gpio_out(dev, 3, qdev_get_gpio_in(i2c_dev, 0));
    /* I2C clock */
    //qdev_connect_gpio_out(dev, 4, qdev_get_gpio_in(i2c_dev, 1));

    //for (i = 0; i < 3; i++) {
    //    qdev_connect_gpio_out(dev, i, qdev_get_gpio_in(lcd_dev, i));
    //}
    //for (i = 0; i < 4; i++) {
    //    qdev_connect_gpio_out(key_dev, i, qdev_get_gpio_in(dev, i + 8));
    //}
    //for (i = 4; i < 8; i++) {
    //    qdev_connect_gpio_out(key_dev, i, qdev_get_gpio_in(dev, i + 15));
    //}

    //wm8750_dev = i2c_create_slave(i2c, "wm8750", LPL_WM_ADDR);
    //dev = qdev_create(NULL, "lophilo_audio");
    //s = sysbus_from_qdev(dev);
    //qdev_prop_set_ptr(dev, "wm8750", wm8750_dev);
    //qdev_init_nofail(dev);
    //sysbus_mmio_map(s, 0, LPL_AUDIO_BASE);
    //sysbus_connect_irq(s, 0, pic[LPL_AUDIO_IRQ]);

    lophilo_binfo.ram_size = LPL_RAM_DEFAULT_SIZE;
    lophilo_binfo.kernel_filename = kernel_filename;
    lophilo_binfo.kernel_cmdline = kernel_cmdline;
    lophilo_binfo.initrd_filename = initrd_filename;
    arm_load_kernel(env, &lophilo_binfo);
}

typedef struct sam9m10_pic_state
{
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t level;
    uint32_t enabled;
    qemu_irq parent_irq;
} sam9m10_pic_state;

static void sam9m10_pic_update(sam9m10_pic_state *s)
{
    qemu_set_irq(s->parent_irq, (s->level & s->enabled));
}

static void sam9m10_pic_set_irq(void *opaque, int irq, int level)
{
    sam9m10_pic_state *s = opaque;

    if (level) {
        s->level |= 1 << irq;
    } else {
        s->level &= ~(1 << irq);
    }
    sam9m10_pic_update(s);
}

static uint64_t sam9m10_pic_read(void *opaque, target_phys_addr_t offset,
                                   unsigned size)
{
    sam9m10_pic_state *s = opaque;

    switch (offset) {
    case LPL_PIC_STATUS:
        return s->level & s->enabled;

    default:
        return 0;
    }
}

static void sam9m10_pic_write(void *opaque, target_phys_addr_t offset,
                                uint64_t value, unsigned size)
{
    sam9m10_pic_state *s = opaque;

    switch (offset) {
    case LPL_PIC_ENABLE_SET:
        s->enabled |= value;
        break;

    case LPL_PIC_ENABLE_CLR:
        s->enabled &= ~value;
        s->level &= ~value;
        break;
    }
    sam9m10_pic_update(s);
}

static void sam9m10_pic_reset(DeviceState *d)
{
    sam9m10_pic_state *s = FROM_SYSBUS(sam9m10_pic_state,
                                         sysbus_from_qdev(d));

    s->level = 0;
    s->enabled = 0;
}

static const MemoryRegionOps sam9m10_pic_ops = {
    .read = sam9m10_pic_read,
    .write = sam9m10_pic_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int sam9m10_pic_init(SysBusDevice *dev)
{
    sam9m10_pic_state *s = FROM_SYSBUS(sam9m10_pic_state, dev);

    qdev_init_gpio_in(&dev->qdev, sam9m10_pic_set_irq, 32);
    sysbus_init_irq(dev, &s->parent_irq);
    memory_region_init_io(&s->iomem, &sam9m10_pic_ops, s,
                          "musicpal-pic", LPL_PIC_SIZE);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static const VMStateDescription sam9m10_pic_vmsd = {
    .name = "sam9m10_pic",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(level, sam9m10_pic_state),
        VMSTATE_UINT32(enabled, sam9m10_pic_state),
        VMSTATE_END_OF_LIST()
    }
};

static void sam9m10_pic_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = sam9m10_pic_init;
    dc->reset = sam9m10_pic_reset;
    dc->vmsd = &sam9m10_pic_vmsd;
}

static TypeInfo sam9m10_pic_info = {
    .name          = "sam9m10_pic",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(sam9m10_pic_state),
    .class_init    = sam9m10_pic_class_init,
};

static QEMUMachine lophilo_machine = {
    .name = "lophilo",
    .desc = "Lophilo (ARM926EJ-S) tabby",
    .init = lophilo_init,
};

static void lophilo_machine_init(void)
{
    qemu_register_machine(&lophilo_machine);
}

machine_init(lophilo_machine_init);

static void lophilo_register_types(void)
{
    type_register_static(&sam9m10_pic_info);
    type_register_static(&sam9m10_pit_info);
}
type_init(lophilo_register_types)
