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
#include "at91sam9263_defs.h"

#include "lophilo.h"

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
    qemu_irq pic1[32];
    DeviceState *dev;
    DeviceState *pit;
    DeviceState *pmc;

    int i;

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

    /* MEMORY */
    /* For now we use a fixed - the original - RAM size */
    memory_region_init_ram(ram, "lophilo.ram", LPL_RAM_DEFAULT_SIZE);
    vmstate_register_ram_global(ram);
    memory_region_add_subregion(address_space_mem, 0, ram);

    memory_region_init_ram(sram, "lophilo.sram", LPL_SRAM_SIZE);
    vmstate_register_ram_global(sram);
    memory_region_add_subregion(address_space_mem, LPL_SRAM_BASE, sram);

    /* INTERNAL DEVICES */
    dev = sysbus_create_varargs("at91,aic", AT91_AIC_BASE,
                                cpu_pic[ARM_PIC_CPU_IRQ],
                                cpu_pic[ARM_PIC_CPU_FIQ],
                                NULL);

    for (i = 0; i < 32; i++) {
        pic[i] = qdev_get_gpio_in(dev, i);
    }

    dev = sysbus_create_simple("at91,intor", -1, pic[1]);
    for (i = 0; i < 32; i++) {
        pic1[i] = qdev_get_gpio_in(dev, i);
    }
    sysbus_create_simple("at91,dbgu", AT91_DBGU_BASE, pic1[0]);
    pmc = sysbus_create_simple("at91,pmc", AT91_PMC_BASE, pic1[1]);
    qdev_prop_set_uint32(pmc, "mo_freq", 16000000);
    pit = sysbus_create_simple("at91,pit", AT91_PITC_BASE, pic1[3]);

    /*
     * Boot configuration
     */
    lophilo_binfo.ram_size = LPL_RAM_DEFAULT_SIZE;
    lophilo_binfo.kernel_filename = kernel_filename;
    lophilo_binfo.kernel_cmdline = kernel_cmdline;
    lophilo_binfo.initrd_filename = initrd_filename;

    arm_load_kernel(env, &lophilo_binfo);
}

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
