/*
 * First userspace program for SANTONI bring-up: mockfs serves the RAM disk
 * itself as /sbin/launchd, so this static binary is PID 1. It proves that
 * exec, the user/kernel boundary, devfs and the console all work on the
 * Cortex-A53 target, then stays alive (init exiting panics the kernel).
 *
 * tools/mkinit.py wraps the assembled __text into a static Mach-O, so this
 * must stay position independent with no relocations and no writable data:
 * every reference is adr (PC-relative), the PID digit lives on the stack.
 *
 * Raw XNU syscalls: number in x16, svc #0x80, carry set on error.
 */
#define SYS_write	4
#define SYS_open	5
#define SYS_read	3
#define SYS_close	6
#define SYS_getpid	20
#define SYS_select	93
#define O_RDONLY	0
#define O_RDWR		2

	.text
	.globl	_start
	.p2align 2
_start:
	/*
	 * The kernel enters a static binary with sp at argc, aligned only to
	 * what the argument/apple strings happened to need. Using a misaligned
	 * sp as a base register raises an SP alignment fault (SIGBUS) on real
	 * hardware (QEMU does not check), so align it like crt0/dyld do.
	 */
	mov	x9, sp
	and	x9, x9, #~15
	mov	sp, x9

	adr	x0, path
	mov	x1, #O_RDWR
	mov	x2, #0
	mov	x16, #SYS_open
	svc	#0x80
	b.cs	hang			/* no console: nothing to report to */
	mov	x19, x0

	adr	x1, hello
	adr	x2, hello_tail
	sub	x2, x2, x1
	bl	write_console

	mov	x16, #SYS_getpid
	svc	#0x80
	add	w0, w0, #'0'		/* PID 1 expected: one digit */
	sub	sp, sp, #16
	strb	w0, [sp]
	mov	x1, sp
	mov	x2, #1
	bl	write_console
	add	sp, sp, #16

	adr	x1, hello_tail
	adr	x2, alive
	sub	x2, x2, x1
	bl	write_console

	/*
	 * eMMC root probe: read LBA 0 of /dev/remmcroot0 (the raw character
	 * device, present because IOKitBSDInit probed the controller) and print
	 * the first 8 bytes. Missing device (rd=md0) just skips this.
	 */
	adr	x0, emmc_path
	mov	x1, #O_RDONLY
	mov	x2, #0
	mov	x16, #SYS_open
	svc	#0x80
	b.cs	emmc_done

	mov	x20, x0			/* fd */

	sub	sp, sp, #544		/* [0..511] read buf, [512..535] hex */
	stp	xzr, xzr, [sp]
	stp	xzr, xzr, [sp, #512]
	mov	x0, x20
	mov	x1, sp
	mov	x2, #512
	mov	x16, #SYS_read
	svc	#0x80
	b.cs	emmc_close

	adr	x1, emmc_msg
	adr	x2, emmc_msg_end
	sub	x2, x2, x1
	bl	write_console

	/* hex-format sp[0..7] as "xx xx xx xx xx xx xx xx" at sp+512 */
	add	x5, sp, #512
	mov	x6, x5			/* cursor */
	mov	x4, sp			/* source */
	mov	x3, #8			/* counter */
emmc_hex:
	ldrb	w7, [x4], #1
	lsr	w9, w7, #4
	and	w9, w9, #0xf
	cmp	w9, #9
	b.ls	1f
	add	w9, w9, #39		/* 'a' - '0' - 10 */
1:	add	w9, w9, #'0'
	strb	w9, [x6], #1
	and	w9, w7, #0xf
	cmp	w9, #9
	b.ls	2f
	add	w9, w9, #39
2:	add	w9, w9, #'0'
	strb	w9, [x6], #1
	mov	w9, #' '
	strb	w9, [x6], #1
	subs	x3, x3, #1
	b.ne	emmc_hex
	mov	w9, #0
	strb	w9, [x6, #-1]		/* drop the trailing space */

	mov	x1, x5
	mov	x2, #23
	bl	write_console

	adr	x1, emmc_tail
	adr	x2, emmc_tail_end
	sub	x2, x2, x1
	bl	write_console

emmc_close:
	add	sp, sp, #544
	mov	x0, x20
	mov	x16, #SYS_close
	svc	#0x80

emmc_done:
tick:
	/* select(0, NULL, NULL, NULL, &5s) as a portable sleep */
	mov	x0, #0
	mov	x1, #0
	mov	x2, #0
	mov	x3, #0
	adr	x4, five_sec
	mov	x16, #SYS_select
	svc	#0x80

	adr	x1, alive
	adr	x2, alive_end
	sub	x2, x2, x1
	bl	write_console
	b	tick

/* write(x19, x1, x2) */
write_console:
	mov	x0, x19
	mov	x16, #SYS_write
	svc	#0x80
	ret

hang:
	wfe
	b	hang

	.p2align 3
five_sec:
	.quad	5			/* tv_sec */
	.quad	0			/* tv_usec (int32 + padding) */
path:
	.asciz	"/dev/console"
hello:
	.ascii	"\n========================================\n"
	.ascii	"*** RIVA userspace: init running as PID "
hello_tail:
	.ascii	" ***\n"
alive:
	.ascii	"RIVA init: still alive (tick)\n"
alive_end:
emmc_path:
	.asciz	"/dev/remmcroot0"
emmc_msg:
	.ascii	"RIVA userspace: read eMMC LBA 0: ["
emmc_msg_end:
emmc_tail:
	.ascii	"]\n"
emmc_tail_end:
