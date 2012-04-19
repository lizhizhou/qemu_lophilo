#ifndef NAND_H
#define NAND_H

# include "hw.h"
# include "flash.h"
# include "blockdev.h"
# include "sysbus.h"
#include "qemu-error.h"

# define MAX_PAGE		0x800
# define MAX_OOB		0x40

typedef struct NANDFlashState NANDFlashState;
struct NANDFlashState {
    SysBusDevice busdev;
    uint8_t manf_id, chip_id;
    uint8_t buswidth; /* in BYTES */
    int size, pages;
    int page_shift, oob_shift, erase_shift, addr_shift;
    uint8_t *storage;
    BlockDriverState *bdrv;
    int mem_oob;

    uint8_t cle, ale, ce, wp, gnd;

    uint8_t io[MAX_PAGE + MAX_OOB + 0x400];
    uint8_t *ioaddr;
    int iolen;

    uint32_t cmd;
    uint64_t addr;
    int addrlen;
    int status;
    int offset;

    void (*blk_write)(NANDFlashState *s);
    void (*blk_erase)(NANDFlashState *s);
    void (*blk_load)(NANDFlashState *s, uint64_t addr, int offset);

    uint32_t ioaddr_vmstate;
};
#endif
