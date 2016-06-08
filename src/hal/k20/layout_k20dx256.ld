__aeabi_unwind_cpp_pr0 = abort;
__aeabi_unwind_cpp_pr1 = abort;
__aeabi_unwind_cpp_pr2 = abort;
__aeabi_memclr4 = __aeabi_memclr;

_data_load = LOADADDR(.data);

INCLUDE iomem.ld

ENTRY(main)

/* For MK20DX256 */
MEMORY
{
    VECT (R)      : ORIGIN = 0x00000000, LENGTH = 0x3FC  /* Vector area */
    FIRC (R)      : ORIGIN = 0x000003FC, LENGTH = 4      /* Custom IRC user trim */
    FCFG (R)      : ORIGIN = 0x00000400, LENGTH = 16     /* Flash config */
    FLASH (RX)    : ORIGIN = 0x00000410, LENGTH = 256K - 0x410 /* Program Flash */
    SRAML (WAIL)  : ORIGIN = 0x20000000 - 32K, LENGTH = 32K /* SRAM_L */
    SRAMU (WAIL)  : ORIGIN = 0x20000000, LENGTH = 32K /* SRAM_L */
}

REGION_ALIAS("vectors", VECT);
REGION_ALIAS("flash_config", FCFG)
REGION_ALIAS("rom", FLASH);
REGION_ALIAS("ram", SRAML);

SECTIONS
{
    .vector : ALIGN(4)
    {
        FILL(0xff)

        KEEP(*(.isr_vector))
        KEEP(*(.isr_vector_nvic))
    } > vectors

    .flashcfg : ALIGN(4)
    {
        KEEP(*(.flash_configuration))
    } > flash_config

    .text : ALIGN(4)
    {
        FILL(0xff)
        *(.text*)
        *(.rodata .rodata.*)
    } > rom
    /* ISR Table space in Ram, NOLOAD because it's initialized by code - just make space for it. */
    .vector_ram (NOLOAD) : ALIGN(1024)
    {
        KEEP(*(.ram_isr_vector))
    } > ram

    .data : ALIGN(4)
    {
        _data = .;

        *(SORT_BY_ALIGNMENT(.data*))
        . = ALIGN(4);

        _edata = .;
    } > ram AT>rom = 0xff

    .bss : ALIGN(4)
    {
        _bss = .;

        *(.bss*)
        *(COMMON)
        . = ALIGN(4);

        _ebss = .;

        . += 4;

        __STACK_LIMIT = .;

        . += 4;

        _eglobals = .;
    } > ram

    /DISCARD/ :
    {
        *(.glue_7*)  /* arm-thumb interworking */
        *(.v4_bx)  /* ARMv4 interworking fixup for missing BX */
        *(.vfp11_veneer)  /* VFP11 bugfixes s.a. http://sourceware.org/ml/binutils/2006-12/msg00196.html */
        *(.iplt .igot.plt)  /* STT_GNU_IFUNC symbols */
        *(.rel.*)  /* dynamic relocations */
        *(.ARM.exidx*) /* index entries for section unwinding */
        *(.ARM.extab*) /* exception unwinding information */
        *(.debug_gdb_scripts)
    }
}

__STACK_BASE  = ORIGIN(ram) + LENGTH(ram);
