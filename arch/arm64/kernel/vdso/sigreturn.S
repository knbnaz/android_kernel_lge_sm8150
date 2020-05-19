/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sigreturn trampoline for returning from a signal when the SA_RESTORER
 * flag is not set. It serves primarily as a hall of shame for crappy
 * unwinders and features an exciting but mysterious NOP instruction.
 *
 * It's also fragile as hell, so please think twice before changing anything
 * in here.
 *
 * Copyright (C) 2012 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 */

#include <linux/linkage.h>
#include <asm/unistd.h>

	.text

/* Ensure that the mysterious NOP can be associated with a function. */
	.cfi_startproc

/*
 * .cfi_signal_frame causes the corresponding Frame Description Entry in the
 * .eh_frame section to be annotated as a signal frame. This allows DWARF
 * unwinders (e.g. libstdc++) to implement _Unwind_GetIPInfo(), which permits
 * unwinding out of the signal trampoline without the need for the mysterious
 * NOP.
 */
	.cfi_signal_frame

/*
 * Tell the unwinder where to locate the frame record linking back to the
 * interrupted context. We don't provide unwind info for registers other
 * than the frame pointer and the link register here; in practice, this
 * is sufficient for unwinding in C/C++ based runtimes and the values in
 * the sigcontext may have been modified by this point anyway. Debuggers
 * already have baked-in strategies for attempting to unwind out of signals.
 */
	.cfi_def_cfa    x29, 0
	.cfi_offset     x29, 0 * 8
	.cfi_offset     x30, 1 * 8

/*
 * This mysterious NOP is required for some unwinders (e.g. libc++) that
 * unconditionally subtract one from the result of _Unwind_GetIP() in order to
 * identify the calling function.
 * Hack borrowed from arch/powerpc/kernel/vdso64/sigtramp.S.
 */
	nop	// Mysterious NOP

SYM_FUNC_START(__kernel_rt_sigreturn)
	mov	x8, #__NR_rt_sigreturn
	svc	#0
	.cfi_endproc
SYM_FUNC_END(__kernel_rt_sigreturn)
