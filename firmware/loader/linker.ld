/* Loader link map — Dante Phase 0.5
 *
 * The loader is baked into the 8 KB BRAM ROM and runs from the CPU reset
 * vector. Its .text/.rodata live in rom; .data has its load address in rom and
 * is copied to sram by crt0; .bss and the stack are in sram.
 *
 * The loader's sram usage is transient: it jumps to the application and never
 * returns, and the application's own crt0 re-initialises .data/.bss. So the two
 * overlapping in sram is harmless by construction.
 */

INCLUDE generated/output_format.ld
ENTRY(_start)

INCLUDE generated/regions.ld

SECTIONS
{
	.text :
	{
		_ftext = .;
		KEEP(*(.isr_vector))
		*crt0*(.text)
		KEEP(*crt0*(.text))
		KEEP(*(.text.isr))

		*(.text .stub .text.* .gnu.linkonce.t.*)
		_etext = .;
	} > rom

	.rodata :
	{
		. = ALIGN(8);
		_frodata = .;
		*(.rodata .rodata.* .gnu.linkonce.r.*)
		*(.rodata1)
		*(.got .got.*)
		*(.toc .toc.*)

		FILL(0);
		. = ALIGN(8);
		_erodata = .;
	} > rom

	.data :
	{
		. = ALIGN(8);
		_fdata = .;
		*(.data .data.* .gnu.linkonce.d.*)
		*(.data1)
		*(.sdata .sdata.* .gnu.linkonce.s.*)

		FILL(0);
		. = ALIGN(8);
		_edata = .;
	} > sram AT > rom

	.bss :
	{
		. = ALIGN(8);
		_fbss = .;
		*(.dynsbss)
		*(.sbss .sbss.* .gnu.linkonce.sb.*)
		*(.scommon)
		*(.dynbss)
		*(.bss .bss.* .gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(8);
		_ebss = .;
		_end = .;
	} > sram

	/DISCARD/ :
	{
		*(.eh_frame)
		*(.comment)
	}
}

PROVIDE(_fstack = ORIGIN(sram) + LENGTH(sram));

PROVIDE(_fdata_rom = LOADADDR(.data));
PROVIDE(_edata_rom = LOADADDR(.data) + SIZEOF(.data));
