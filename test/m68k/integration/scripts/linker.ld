/* M68k Integration Test Linker File (for target "m68000" arch) */
/* Integration tester memory regions */
MEMORY {
    /* Rom will start at the beginning at the memory space and be 128K big */
    rom (rx) : org = 0x000000, len = 128K

    /* Ram will start at the last 64K of the memory space */
    ram (!rx) : org = 0xff0000, len = 64K
}

/* Integration test entry */
ENTRY(run)

/* Integration test sections */
SECTIONS {
    .rom : {
        /* M68k vector table */
        LONG(0x00000000)        /* Reset stack pointer (wraps around to end of address space) */
        LONG(ABSOLUTE(run))    /* Reset program counter */
        . = 0x100;

        /* Code and read only data */
        *(.text) *(.text*)
        *(.rodata) *(.rodata*)
    } >rom = 0x00

    /* Initialize the ram with the data */
    .ram : {
        /* Source data segments */
        *(.data) *(.data*)
        *(.bss) *(.bss*)

        /* End of RAM marker, also this is here so that even if there is no source segments
         * there will still be something for objcopy to copy */
        LONG(0xA5A5A5A5)
    } >ram = 0x00
}
