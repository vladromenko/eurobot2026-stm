/* Main_Board: MDK-ARM/STM32F407.sct + original Keil map.
 * No bootloader offset. CCM exists but original RW_IRAM2 is empty.
 * Keep DMA buffers in main SRAM. No automatic overflow into CCM.
 */
ENTRY(Reset_Handler)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM (xrw) : ORIGIN = 0x20000000, LENGTH = 128K - 0x400
  STACK (rw) : ORIGIN = 0x2001fc00, LENGTH = 0x400
  CCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}
_estack = ORIGIN(STACK) + LENGTH(STACK);
__initial_sp = _estack;
SECTIONS
{
  .isr_vector : ALIGN(512) {
    KEEP(*(.isr_vector))
  } > FLASH
  .text : ALIGN(4) {
    *(.text .text.*) *(.rodata .rodata.*)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4); _etext = .;
  } > FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
  .ARM.exidx : {
    __exidx_start = .; *(.ARM.exidx* .gnu.linkonce.armexidx.*) __exidx_end = .;
  } > FLASH
  .preinit_array : ALIGN(4) {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } > FLASH
  .init_array : ALIGN(4) {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array))
    PROVIDE_HIDDEN(__init_array_end = .);
  } > FLASH
  .fini_array : ALIGN(4) {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } > FLASH
  .data : ALIGN(4) {
    _sdata = .; *(.data .data.*) *(.RamFunc .RamFunc.*)
    . = ALIGN(4); _edata = .;
  } > RAM AT> FLASH
  _sidata = LOADADDR(.data);
  .bss (NOLOAD) : ALIGN(4) {
    _sbss = .; __bss_start__ = .;
    *(.bss .bss.*) *(COMMON)
    . = ALIGN(4); _ebss = .; __bss_end__ = .;
  } > RAM
  .heap (NOLOAD) : ALIGN(8) {
    __heap_base = .; PROVIDE(end = .); PROVIDE(_end = .);
    /* Keil startup declares 0x200, but its map discards HEAP.
       FreeRTOS is static-only. Retain a bounded 0x200 runtime reserve. */
    . += 0x200; __heap_limit = .;
  } > RAM
  .stack (NOLOAD) : {
    __StackLimit = .; . += LENGTH(STACK); __StackTop = .;
  } > STACK
  /* Existing startup initializes only SRAM .data/.bss. Refuse new CCM
     input instead of silently creating uninitialized variables there. */
  .ccmram (NOLOAD) : { *(.ccmram .ccmram.* .ccm .ccm.*) } > CCMRAM
  .ARM.attributes 0 : { *(.ARM.attributes) }
  /DISCARD/ : { *(.note.GNU-stack) }
}
ASSERT(SIZEOF(.isr_vector) == 392, "STM32F407 vector table must contain 98 entries")
ASSERT(ADDR(.isr_vector) == 0x08000000, "Unexpected vector address")
ASSERT(__heap_limit <= __StackLimit, "SRAM/stack overlap")
ASSERT(SIZEOF(.ccmram) == 0, "CCM needs explicit startup initialization before use")
ASSERT(LOADADDR(.data) + SIZEOF(.data) <= ORIGIN(FLASH) + LENGTH(FLASH), "Flash overflow")
