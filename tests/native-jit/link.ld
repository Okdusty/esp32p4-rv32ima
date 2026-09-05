PHDRS
{
	all PT_LOAD FLAGS(7);
}

SECTIONS
{
	. = 0x10000;
	.text : { *(.text.startup) *(.text*) } :all
	.rodata : { *(.rodata*) } :all
	.data : { *(.data*) *(.sdata*) } :all
	.bss : { *(.bss*) *(.sbss*) *(COMMON) } :all
	. = ALIGN(16);
	_stack_bottom = .;
	. += 0x10000;
	_stack_top = .;
	/DISCARD/ : { *(.comment) *(.note*) *(.eh_frame*) }
}
