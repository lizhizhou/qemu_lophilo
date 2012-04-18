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
