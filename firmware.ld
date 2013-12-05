/* Linker script for the firmware */

MEMORY
{
    UMEM (rwx) : ORIGIN = 0x00000000, LENGTH = 0x00004000
    DMEM (rw) : ORIGIN = 0x00080000, LENGTH = 0x00002000
    LOGBUF (rw) : ORIGIN = 0x00081000, LENGTH = 0x00001000
}

_end_stack = 0x00081000;

SECTIONS
{
    .text :
    {
        _start_text = .;
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
        *(.note*)
        _end_text = .;
    } > UMEM
    .data : AT(ADDR(.text) + SIZEOF(.text))
    {
        _start_data = .;
        *(vtable)
        *(.data*)
        _end_data = .;
    } > DMEM
    .bss :
    {
        _start_bss = .;
        *(.bss*)
        *(COMMON)
        _end_bss = .;
    } > DMEM
    .logbuf :
    {
        _logbuf_start = .;
       . += 0x1000;
        _logbuf_end = .;
    } >LOGBUF
}
