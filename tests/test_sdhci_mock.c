/*
 * Host unit test for the Qualcomm SDHCI MSM v4 (SDCC1) block driver.
 *
 * santoni's internal eMMC 5.1 hangs off SDCC1. The controller exposes the
 * standard SDHCI register file (HC) at 0x07824900 (0x500 bytes) and the
 * Qualcomm vendor core registers (CORE) at 0x07824000 (0x800 bytes); both fit
 * in one 0x1000 page window, HC at CORE + 0x900 (msm8937.dtsi sdhc_1).
 *
 * The routines below are the same command framing / PIO code that
 * bsd/dev/santoni_sdhci.c runs on hardware, exercised against a mock
 * controller that models the register file, the command handshake
 * (PRESENT_STATE inhibit -> COMMAND write -> INT_STATUS completion), the
 * buffer-data-ready handshake and the 512-byte PIO FIFO.
 *
 * Build and run on the host:
 *   clang -O2 -Wall -I. tests/test_sdhci_mock.c -o out/test_sdhci_mock
 *   ./out/test_sdhci_mock
 */
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ---------------------------------------------------------------- addresses */

/* Physical MMIO windows of SDCC1 (resources/linux-msm89x7-7.1.3
 * arch/arm64/boot/dts/qcom/msm8937.dtsi, sdhc_1). */
#define SDHCI_CORE_PHYS  0x07824000u
#define SDHCI_HC_PHYS    0x07824900u
#define SDHCI_CORE_SIZE  0x1000u
#define SDHCI_HC_OFFSET  (SDHCI_HC_PHYS - SDHCI_CORE_PHYS)  /* 0x900 */
#define SDHCI_HC_SIZE    0x500u

/* ------------------------------------------------- SDHCI standard registers */

#define SDHCI_BLOCK_SIZE       0x04    /* 16-bit */
#define SDHCI_BLOCK_COUNT      0x06    /* 16-bit */
#define SDHCI_ARGUMENT         0x08
#define SDHCI_TRANSFER_MODE    0x0C    /* 16-bit */
#define SDHCI_COMMAND          0x0E    /* 16-bit */
#define SDHCI_RESPONSE         0x10    /* 0x10..0x1C */
#define SDHCI_BUFFER           0x20
#define SDHCI_PRESENT_STATE    0x24
#define SDHCI_HOST_CONTROL     0x28    /* 8-bit */
#define SDHCI_POWER_CONTROL    0x29    /* 8-bit, never written here */
#define SDHCI_CLOCK_CONTROL    0x2C    /* 16-bit, never written here */
#define SDHCI_SOFTWARE_RESET   0x2F    /* 8-bit */
#define SDHCI_INT_STATUS       0x30
#define SDHCI_INT_ENABLE       0x34
#define SDHCI_SIGNAL_ENABLE    0x38

#define SDHCI_MAKE_BLKSZ(dma, blksz) (((dma) << 12) | ((blksz) & 0xfff))

/* TRANSFER_MODE bits */
#define SDHCI_TRNS_DMA         0x0001
#define SDHCI_TRNS_BLK_CNT_EN  0x0002
#define SDHCI_TRNS_AUTO_CMD12  0x0004
#define SDHCI_TRNS_READ        0x0010
#define SDHCI_TRNS_MULTI       0x0020

/* COMMAND bits */
#define SDHCI_CMD_RESP_NONE        0x00
#define SDHCI_CMD_RESP_LONG        0x01
#define SDHCI_CMD_RESP_SHORT       0x02
#define SDHCI_CMD_RESP_SHORT_BUSY  0x03
#define SDHCI_CMD_CRC              0x08
#define SDHCI_CMD_INDEX            0x10
#define SDHCI_CMD_DATA             0x20
#define SDHCI_MAKE_CMD(c, f)       ((((c) & 0xff) << 8) | ((f) & 0xff))

/* PRESENT_STATE bits */
#define SDHCI_CMD_INHIBIT      0x00000001u
#define SDHCI_DATA_INHIBIT     0x00000002u
#define SDHCI_DATA_AVAILABLE   0x00000800u
#define SDHCI_SPACE_AVAILABLE  0x00000400u

/* HOST_CONTROL: PIO needs the DMA select field cleared, bus width preserved */
#define SDHCI_CTRL_DMA_MASK    0x18u
#define SDHCI_CTRL_8BITBUS     0x20u

/* SOFTWARE_RESET */
#define SDHCI_RESET_ALL        0x01
#define SDHCI_RESET_CMD        0x02
#define SDHCI_RESET_DATA       0x04

/* INT_STATUS bits */
#define SDHCI_INT_CC           0x00000001u  /* command complete */
#define SDHCI_INT_TC           0x00000002u  /* transfer complete */
#define SDHCI_INT_DMA_END      0x00000008u
#define SDHCI_INT_BWR          0x00000010u  /* buffer write ready */
#define SDHCI_INT_BRR          0x00000020u  /* buffer read ready */
#define SDHCI_INT_ERR          0x00008000u
/* Specific error bits behind SDHCI_INT_ERR (SDHCI 3.0, dword at 0x30). */
#define SDHCI_INT_CMD_INDEX    0x00800000u
#define SDHCI_INT_CMD_END_BIT  0x01000000u
#define SDHCI_INT_CMD_CRC      0x02000000u
#define SDHCI_INT_CMD_TIMEOUT  0x04000000u

#define SDHCI_HOST_VERSION     0xfe
#define SDHCI_CLOCK_INT_EN     0x0001u
#define SDHCI_CLOCK_INT_STABLE 0x0002u
#define SDHCI_CLOCK_CARD_EN    0x0004u

/* ------------------------------------------ Qualcomm MSM v4 core registers */

#define CORE_MCI_VERSION       0x50
#define CORE_GENERICS          0x70
#define CORE_HC_MODE           0x78
#define CORE_HC_MODE_EN        0x00000001u
#define CORE_FF_CLK_SW_RST_DIS 0x00002000u
#define CORE_PWRCTL_STATUS     0xdc
#define CORE_PWRCTL_MASK       0xe0
#define CORE_PWRCTL_CLEAR      0xe4
#define CORE_PWRCTL_CTL        0xe8
#define CORE_PWRCTL_BUS_OFF    0x00000001u
#define CORE_PWRCTL_BUS_ON     0x00000002u
#define CORE_PWRCTL_IO_LOW     0x00000004u
#define CORE_PWRCTL_IO_HIGH    0x00000008u
#define CORE_PWRCTL_INT_MASK   0x0000000fu
#define CORE_PWRCTL_BUS_SUCCESS 0x00000001u
#define CORE_PWRCTL_IO_SUCCESS 0x00000004u
#define CORE_IO_PAD_PWR_SWITCH_EN 0x00008000u    /* BIT(15) */
#define CORE_IO_PAD_PWR_SWITCH 0x00010000u    /* BIT(16) */

#define SDHCI_POWER_CONTROL    0x29
#define SDHCI_TIMEOUT_CONTROL  0x2e
#define SDHCI_HOST_CONTROL2    0x3e
#define SDHCI_CTRL_4BITBUS     0x02u
#define SDHCI_CTRL_VDD_180     0x0008u
#define SDHCI_POWER_ON         0x01u
#define SDHCI_POWER_180        0x0au
#define SDHCI_POWER_CONTROL_VAL (SDHCI_POWER_180 | SDHCI_POWER_ON) /* 0x0b */

#define CORE_VENDOR_SPEC       0x10c
#define CORE_VENDOR_SPEC_POR_VAL (0x00000a9cu | CORE_IO_PAD_PWR_SWITCH_EN | CORE_IO_PAD_PWR_SWITCH)
#define CORE_HC_MCLK_SEL_MASK  (3u << 8)
#define CORE_HC_MCLK_SEL_DFLT  (2u << 8)
#define CORE_HC_SELECT_IN_EN   0x00040000u
#define CORE_HC_SELECT_IN_MASK (7u << 19)
#define CORE_DLL_CONFIG        0x100
#define CORE_DLL_STATUS        0x108
#define CORE_DLL_LOCK          0x00000080u
#define CORE_DLL_EN            0x00010000u
#define CORE_CK_OUT_EN         0x00040000u
#define CORE_DLL_PDN           0x20000000u
#define CORE_DLL_RST           0x40000000u
#define CORE_DLL_OFF           (CORE_DLL_RST | CORE_DLL_PDN)
#define CORE_MCI_POWER         0x000
#define CORE_MCI_CLK           0x004

/*
 * The relative card address LK/ABL assigned in CMD3 (MMC_RCA in
 * bootloader_lk include/mmc.h). Addressed commands must carry it in [31:16];
 * the mock card only answers when the argument matches.
 */
#define SANTONI_CARD_RCA       2u

/* --------------------------------------------------------- card / protocol */

#define SDHCI_BLOCK      512u
#define SDHCI_BOUNCE     (1u << 20)
#define SDHCI_MAX_BLOCKS (SDHCI_BOUNCE / SDHCI_BLOCK)

#define MOCK_CARD_BLOCKS 16u
#define CMD_TIMEOUT_US   50000u
#define CMD_INHIBIT_US   50000u

/* -------------------------------------------------------------- host timing */

static uint64_t
host_now_ns(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* The kernel uses delay(us); a short sleep keeps this test from spinning. */
static void
host_delay_us(unsigned us)
{
	struct timespec ts = { 0, (long)us * 1000 };

	nanosleep(&ts, NULL);
}

/* ------------------------------------------------------------ mock hardware */

static uint8_t mock_hc[SDHCI_CORE_SIZE];
static uint8_t mock_core[SDHCI_CORE_SIZE];

static struct mock {
	uint32_t int_status;
	uint32_t present_state;
	int inhibit_polls;
	int stall_cmd;
	int fail_cmd;
	int fail_data;
	int clock_stalled;
	unsigned state_reads;
	unsigned clock_writes;
	unsigned reset_writes;
	uint16_t reset_value;
	int hc_mode_write_ignored;      /* HC_MODE_EN never latches */
	/* card model: one eMMC at m.rca, which only answers addressed commands
	 * whose argument carries that RCA in bits 31:16 */
	uint32_t rca;
	int card_asleep;                /* CMD13 is ignored until CMD5 awake */
	int card_absent;                /* nothing answers, only CMD0 completes */
	uint16_t blk_size;
	uint16_t blk_count;
	uint16_t tr_mode;
	uint32_t argument;
	unsigned cmd_count;
	struct {
		uint16_t cmd;
		uint16_t word;
		uint32_t arg;
		uint16_t tr_mode;
	} trace[32];
	/* data phase */
	int data_active;
	int data_read;          /* 1 = card -> host */
	uint32_t cur_lba;
	uint32_t blocks_left;
	uint32_t word;
	uint8_t block[SDHCI_BLOCK];
} m;

static uint8_t
mock_card[MOCK_CARD_BLOCKS * SDHCI_BLOCK];

static uint16_t
mock_rd16(uint32_t off)
{
	return (uint16_t)(mock_hc[off] | (mock_hc[off + 1] << 8));
}

static void
mock_wr16(uint32_t off, uint16_t val)
{
	mock_hc[off] = (uint8_t)val;
	mock_hc[off + 1] = (uint8_t)(val >> 8);
}

static uint32_t
mock_rd32(uint32_t off)
{
	return (uint32_t)mock_hc[off] | ((uint32_t)mock_hc[off + 1] << 8) |
	    ((uint32_t)mock_hc[off + 2] << 16) | ((uint32_t)mock_hc[off + 3] << 24);
}

static void
mock_wr32(uint32_t off, uint32_t val)
{
	mock_hc[off] = (uint8_t)val;
	mock_hc[off + 1] = (uint8_t)(val >> 8);
	mock_hc[off + 2] = (uint8_t)(val >> 16);
	mock_hc[off + 3] = (uint8_t)(val >> 24);
}

/* Load the current block of the fake card into the FIFO model. */
static void
mock_load_block(void)
{
	memcpy(m.block, mock_card + (uint64_t)m.cur_lba * SDHCI_BLOCK, SDHCI_BLOCK);
	m.word = 0;
	m.int_status |= m.data_read ? SDHCI_INT_BRR : SDHCI_INT_BWR;
}

static inline uint32_t core_rd(uint32_t off);
static inline uint32_t hc_rd(uint32_t off);
static inline void hc_wr(uint32_t off, uint32_t val);

/*
 * Is the HC datapath clocked? The standard register file always answers (it is
 * on the bus clock), which is why the photographed boot log could read
 * HOST_VERSION/CLOCK_CONTROL while nothing worked. The command/data state
 * machine additionally needs:
 *   1. VENDOR_SPEC[9:8] = 2, the free-running MCLK the HC block runs on, and
 *   2. a live source for the card clock: CK_OUT_EN=0 is the bypass path and
 *      always works; CK_OUT_EN=1 hands SDCLK to the DLL, which therefore has to
 *      be enabled, out of reset/power-down and locked.
 * That is exactly the state of the df4395e hardware log: HC_MODE 1,
 * CLOCK_CONTROL 0x0007, VENDOR_SPEC 0x0, and every command - including CMD0,
 * which needs no card - stuck with CMD_INHIBIT 1 and INT_STATUS 0.
 */
static int
mock_hc_clocked(void)
{
	uint32_t vs = hc_rd(CORE_VENDOR_SPEC);
	uint32_t dll = hc_rd(CORE_DLL_CONFIG);
	uint32_t status = hc_rd(CORE_DLL_STATUS);

	if ((vs & CORE_HC_MCLK_SEL_MASK) != CORE_HC_MCLK_SEL_DFLT) {
		return 0;
	}
	if ((dll & CORE_CK_OUT_EN) && (!(dll & CORE_DLL_EN) ||
	    (dll & CORE_DLL_OFF) || !(status & CORE_DLL_LOCK))) {
		return 0;
	}
	return 1;
}

static void
mock_start_data(uint32_t lba, uint16_t count, int read)
{
	m.cur_lba = lba;
	m.blocks_left = count;
	m.data_read = read;
	m.data_active = 1;
	if (read) {
		mock_load_block();
	} else {
		memset(m.block, 0, sizeof(m.block));
		m.word = 0;
		m.int_status |= SDHCI_INT_BWR;
	}
}

static uint32_t
mock_buf_read(void)
{
	uint32_t w;

	if (!m.data_active) {
		return 0xffffffffu;
	}
	if (m.fail_data) {
		m.int_status |= SDHCI_INT_ERR;
		m.data_active = 0;
		return 0xdeadbeefu;
	}
	w = (uint32_t)m.block[m.word * 4] | ((uint32_t)m.block[m.word * 4 + 1] << 8) |
	    ((uint32_t)m.block[m.word * 4 + 2] << 16) |
	    ((uint32_t)m.block[m.word * 4 + 3] << 24);
	m.word++;
	if (m.word == SDHCI_BLOCK / 4) {
		if (!m.data_read) {
			/* A write block is only committed once fully pushed. */
			memcpy(mock_card + (uint64_t)m.cur_lba * SDHCI_BLOCK, m.block,
			    SDHCI_BLOCK);
		}
		m.blocks_left--;
		m.cur_lba++;
		if (m.blocks_left) {
			if (m.data_read) {
				mock_load_block();
			} else {
				memset(m.block, 0, sizeof(m.block));
				m.word = 0;
				m.int_status |= SDHCI_INT_BWR;
			}
		} else {
			m.data_active = 0;
			m.int_status |= SDHCI_INT_TC;
		}
	}
	return w;
}

static void
mock_buf_write(uint32_t val)
{
	if (!m.data_active || m.data_read) {
		return;
	}
	m.block[m.word * 4] = (uint8_t)val;
	m.block[m.word * 4 + 1] = (uint8_t)(val >> 8);
	m.block[m.word * 4 + 2] = (uint8_t)(val >> 16);
	m.block[m.word * 4 + 3] = (uint8_t)(val >> 24);
	m.word++;
	if (m.word == SDHCI_BLOCK / 4) {
		memcpy(mock_card + (uint64_t)m.cur_lba * SDHCI_BLOCK, m.block,
		    SDHCI_BLOCK);
		m.blocks_left--;
		m.cur_lba++;
		if (m.blocks_left) {
			memset(m.block, 0, sizeof(m.block));
			m.word = 0;
			m.int_status |= SDHCI_INT_BWR;
		} else {
			m.data_active = 0;
			m.int_status |= SDHCI_INT_TC;
		}
	}
}

static void
mock_command(uint16_t val)
{
	uint8_t index = (uint8_t)(val >> 8);
	uint8_t flags = (uint8_t)val;

	if (m.cmd_count < sizeof(m.trace) / sizeof(m.trace[0])) {
		m.trace[m.cmd_count].cmd = index;
		m.trace[m.cmd_count].word = val;
		m.trace[m.cmd_count].arg = m.argument;
		m.trace[m.cmd_count].tr_mode = m.tr_mode;
	}
	m.cmd_count++;

	/*
	 * No functional clock or bus power off: the 48 bits never leave the shifter, the
	 * controller keeps CMD_INHIBIT asserted and never posts Command
	 * Complete - the card being absent is irrelevant here,
	 * CMD0 behaves the same.
	 */
	if (!mock_hc_clocked() || !(mock_hc[SDHCI_POWER_CONTROL] & SDHCI_POWER_ON)) {
		m.present_state |= SDHCI_CMD_INHIBIT;
		return;
	}
	if (m.stall_cmd) {
		return;                 /* no completion: driver must time out */
	}
	if (m.fail_cmd) {
		m.int_status |= SDHCI_INT_ERR;
		return;
	}
	if (m.card_absent && index != 0 && index != 12) {
		return;                 /* nothing on the bus: no response */
	}

	switch (index) {
	case 13:                        /* SEND_STATUS: R1, card in TRAN */
		if (m.card_asleep || (m.argument >> 16) != m.rca) {
			/* A sleeping or unselected card is absent on the bus. */
			return;
		}
		mock_wr32(SDHCI_RESPONSE, 0x00000900u);
		break;
	case 5:                         /* SLEEP_AWAKE: R1b, RCA repeated */
		if ((m.argument >> 16) != m.rca) {
			return;
		}
		m.card_asleep = (m.argument & 0x8000u) != 0;
		mock_wr32(SDHCI_RESPONSE, (m.rca << 16) | 0x00000900u);
		break;
	case 7:                         /* SELECT_CARD: R1b */
		if ((m.argument >> 16) != m.rca) {
			return;
		}
		m.card_asleep = 0;
		mock_wr32(SDHCI_RESPONSE, 0x00000900u);
		break;
	case 10:                        /* SEND_CID: 128-bit response */
		if ((m.argument >> 16) != m.rca) {
			return;
		}
		/* The MSM core strips the CRC: payload right-aligned, 3 bytes
		 * in the most-significant word. */
		mock_wr32(SDHCI_RESPONSE + 0, 0x34567821u);
		mock_wr32(SDHCI_RESPONSE + 4, 0x34520112u);
		mock_wr32(SDHCI_RESPONSE + 8, 0x38475446u);
		mock_wr32(SDHCI_RESPONSE + 12, 0x00150100u);
		break;
	case 9:                         /* SEND_CSD: 128-bit response */
		if ((m.argument >> 16) != m.rca) {
			return;
		}
		mock_wr32(SDHCI_RESPONSE + 0, 0x00000000u);
		mock_wr32(SDHCI_RESPONSE + 4, 0xb93b9acau);
		mock_wr32(SDHCI_RESPONSE + 8, 0x0a0f5a83u);
		mock_wr32(SDHCI_RESPONSE + 12, 0x00d00f59u);
		break;
	case 17:                        /* READ_SINGLE_BLOCK */
		mock_start_data(m.argument, 1, 1);
		break;
	case 18:                        /* READ_MULTIPLE_BLOCK */
		mock_start_data(m.argument, m.blk_count, 1);
		break;
	case 24:                        /* WRITE_BLOCK */
		mock_start_data(m.argument, 1, 0);
		break;
	case 25:                        /* WRITE_MULTIPLE_BLOCK */
		mock_start_data(m.argument, m.blk_count, 0);
		break;
	case 12:                        /* STOP_TRANSMISSION */
		m.data_active = 0;
		break;
	default:
		break;
	}
	m.int_status |= SDHCI_INT_CC;
	(void)flags;
}

static uint32_t
mock_hc_read(uint32_t off)
{
	switch (off) {
	case SDHCI_PRESENT_STATE:
		m.state_reads++;
		if (m.inhibit_polls > 0) {
			m.inhibit_polls--;
			return SDHCI_CMD_INHIBIT;
		}
		return m.present_state;
	case SDHCI_INT_STATUS:
		/*
		 * Strict model: a status bit is only visible while it is
		 * enabled at 0x34/0x36. The SDHCI spec sets the bit when the
		 * event happens regardless of the enable registers (QEMU's
		 * sdhci_read() returns the raw status too), but there are MSM/
		 * DWC cores where the enable gates the read, so the driver must
		 * not depend on the looser behaviour. The driver therefore
		 * leaves 0x34 all ones and only 0x38 at zero (no IRQ routed).
		 */
		return m.int_status & mock_rd32(SDHCI_INT_ENABLE);
	case SDHCI_BUFFER:
		return mock_buf_read();
	default:
		return mock_rd32(off);
	}
}

static void
mock_hc_write(uint32_t off, uint32_t val)
{
	switch (off) {
	case SDHCI_ARGUMENT:
		m.argument = val;
		mock_wr32(off, val);
		break;
	case SDHCI_TRANSFER_MODE:
		m.tr_mode = (uint16_t)val;
		mock_wr16(off, (uint16_t)val);
		break;
	case SDHCI_BLOCK_SIZE:
		m.blk_size = (uint16_t)val;
		mock_wr16(off, (uint16_t)val);
		break;
	case SDHCI_BLOCK_COUNT:
		m.blk_count = (uint16_t)val;
		mock_wr16(off, (uint16_t)val);
		break;
	case SDHCI_COMMAND:
		mock_wr16(off, (uint16_t)val);
		mock_command((uint16_t)val);
		break;
	case SDHCI_BUFFER:
		mock_buf_write(val);
		break;
	case SDHCI_INT_STATUS:
		m.int_status &= ~val;
		break;
	case SDHCI_SOFTWARE_RESET:
		m.reset_writes++;
		m.reset_value = (uint16_t)val;
		/* A command-path reset releases a stuck CMD_INHIBIT. */
		if (val & (SDHCI_RESET_CMD | SDHCI_RESET_ALL)) {
			m.present_state &= ~SDHCI_CMD_INHIBIT;
		}
		if (val & SDHCI_RESET_ALL) {
			/* RESET_ALL clears power and raises BUS_OFF request */
			mock_hc[SDHCI_POWER_CONTROL] = 0;
			mock_core[CORE_PWRCTL_STATUS] |= CORE_PWRCTL_BUS_OFF;
		}
		mock_hc[off] = 0;       /* reset self-clears when complete */
		break;
	case SDHCI_POWER_CONTROL:
		mock_hc[off] = (uint8_t)val;
		if (val & SDHCI_POWER_ON) {
			mock_core[CORE_PWRCTL_STATUS] |= CORE_PWRCTL_BUS_ON;
		}
		break;
	case SDHCI_CLOCK_CONTROL:
		m.clock_writes++;       /* the default hand-off must need none */
		if (m.clock_stalled) {
			mock_wr16(off, (uint16_t)val);
		} else if (val & SDHCI_CLOCK_INT_EN) {
			mock_wr16(off, (uint16_t)(val | SDHCI_CLOCK_INT_STABLE));
		} else {
			mock_wr16(off, (uint16_t)val);
		}
		break;
	default:
		mock_wr32(off, val);
		break;
	}
}

/* ------------------------------------------- driver-side accessors (tested) */

static inline uint32_t
hc_rd(uint32_t off)
{
	return mock_hc_read(off);
}

static inline void
hc_wr(uint32_t off, uint32_t val)
{
	mock_hc_write(off, val);
}

static inline uint16_t
hc_rd16(uint32_t off)
{
	return mock_rd16(off);
}

static inline void
hc_wr16(uint32_t off, uint16_t val)
{
	mock_hc_write(off, val);
}

static inline uint8_t
hc_rd8(uint32_t off)
{
	return mock_hc[off];
}

static inline void
hc_wr8(uint32_t off, uint8_t val)
{
	if (off == SDHCI_SOFTWARE_RESET || off == SDHCI_POWER_CONTROL ||
	    off == SDHCI_TIMEOUT_CONTROL || off == SDHCI_HOST_CONTROL) {
		mock_hc[off] = val;
		if (off == SDHCI_SOFTWARE_RESET) {
			mock_hc_write(off, val);
		} else if (off == SDHCI_POWER_CONTROL) {
			mock_hc_write(off, val);
		}
		return;
	}
	mock_hc[off] = val;
}

static inline uint32_t
core_rd(uint32_t off)
{
	uint32_t v = (uint32_t)mock_core[off] | ((uint32_t)mock_core[off + 1] << 8) |
	    ((uint32_t)mock_core[off + 2] << 16) | ((uint32_t)mock_core[off + 3] << 24);
	return v;
}

static inline void
core_wr(uint32_t off, uint32_t val)
{
	if (off == CORE_HC_MODE && m.hc_mode_write_ignored) {
		return;
	}
	if (off == CORE_PWRCTL_CLEAR) {
		uint32_t cur = (uint32_t)mock_core[CORE_PWRCTL_STATUS] |
		    ((uint32_t)mock_core[CORE_PWRCTL_STATUS + 1] << 8) |
		    ((uint32_t)mock_core[CORE_PWRCTL_STATUS + 2] << 16) |
		    ((uint32_t)mock_core[CORE_PWRCTL_STATUS + 3] << 24);
		cur &= ~val;
		mock_core[CORE_PWRCTL_STATUS] = (uint8_t)cur;
		mock_core[CORE_PWRCTL_STATUS + 1] = (uint8_t)(cur >> 8);
		mock_core[CORE_PWRCTL_STATUS + 2] = (uint8_t)(cur >> 16);
		mock_core[CORE_PWRCTL_STATUS + 3] = (uint8_t)(cur >> 24);
		return;
	}
	mock_core[off] = (uint8_t)val;
	mock_core[off + 1] = (uint8_t)(val >> 8);
	mock_core[off + 2] = (uint8_t)(val >> 16);
	mock_core[off + 3] = (uint8_t)(val >> 24);
}

/* The vendor core answers only while its clock branch is on. */
static int
sdhci_core_check(void)
{
	uint32_t mci_version = core_rd(CORE_MCI_VERSION);

	if (mci_version == 0 || mci_version == 0xffffffffu) {
		return ENXIO;
	}
	return 0;
}

/*
 * LK leaves the core in legacy MCI mode (HC_MODE 0x0), so the SDHCI window at
 * +0x900 is not decoded until HC_MODE_EN is written. FF_CLK_SW_RST_DIS is a
 * second write, after bit 0 has latched, exactly like Linux sdhci_msm_probe():
 * a single combined write was measured on hardware to drop bit 13. If bit 0 is
 * write-only on the core, a live HC window is the proof the switch happened.
 */
static int
sdhci_enter_hc_mode(void)
{
	uint32_t hc_mode = core_rd(CORE_HC_MODE);
	uint16_t host_version;

	if ((hc_mode & (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS)) ==
	    (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS)) {
		return 0;
	}
	core_wr(CORE_HC_MODE, (hc_mode & ~CORE_HC_MODE_EN) | CORE_HC_MODE_EN);
	hc_mode = core_rd(CORE_HC_MODE);
	if (!(hc_mode & CORE_HC_MODE_EN)) {
		host_version = hc_rd16(SDHCI_HOST_VERSION);
		if (host_version == 0 || host_version == 0xffffu) {
			return ENXIO;
		}
	}
	core_wr(CORE_HC_MODE, hc_mode | CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS);
	return 0;
}

/* Linux msm_hc_select_default(): free-running MCLK, HC_SELECT_IN off. */
static void
sdhci_select_hc_clock(void)
{
	uint32_t vs = hc_rd(CORE_VENDOR_SPEC);

	vs &= ~CORE_HC_MCLK_SEL_MASK;
	vs |= CORE_HC_MCLK_SEL_DFLT;
	vs &= ~(CORE_HC_SELECT_IN_EN | CORE_HC_SELECT_IN_MASK);
	hc_wr(CORE_VENDOR_SPEC, vs);
}

/* Card-clock output follows the DLL: live+locked -> its output, else bypass. */
static void
sdhci_clock_output(void)
{
	uint32_t dll = hc_rd(CORE_DLL_CONFIG);
	uint32_t status = hc_rd(CORE_DLL_STATUS);
	bool dll_live = (dll & CORE_DLL_EN) && !(dll & CORE_DLL_OFF) &&
	    (status & CORE_DLL_LOCK);
	uint32_t want = dll_live ? (dll | CORE_CK_OUT_EN) :
	    (dll & ~CORE_CK_OUT_EN);

	if (want != dll) {
		hc_wr(CORE_DLL_CONFIG, want);
	}
}

static void
sdhci_pwrctl_ack(const char *what)
{
	uint64_t deadline = host_now_ns() + 5000000ull; /* 5 ms */
	uint32_t ack, status = 0;
	int retry;

	while (host_now_ns() <= deadline) {
		status = core_rd(CORE_PWRCTL_STATUS) & CORE_PWRCTL_INT_MASK;
		if (status) {
			break;
		}
		host_delay_us(1);
	}
	if (!status) {
		(void)what;
		return;
	}

	ack = 0;
	if (status & (CORE_PWRCTL_BUS_ON | CORE_PWRCTL_BUS_OFF)) {
		ack |= CORE_PWRCTL_BUS_SUCCESS;
	}
	if (status & (CORE_PWRCTL_IO_LOW | CORE_PWRCTL_IO_HIGH)) {
		ack |= CORE_PWRCTL_IO_SUCCESS;
	}
	for (retry = 0; retry < 10; retry++) {
		core_wr(CORE_PWRCTL_CLEAR, status);
		if (!(core_rd(CORE_PWRCTL_STATUS) & status)) {
			break;
		}
		host_delay_us(1);
	}
	core_wr(CORE_PWRCTL_CTL, ack);
}

/* Take the controller over from the bootloader. */
static int
sdhci_take_over(void)
{
	uint16_t clk;
	uint8_t ctrl, to_ctrl;
	uint64_t deadline;

	if (sdhci_core_check() != 0) {
		return ENXIO;
	}
	core_wr(CORE_PWRCTL_MASK, 0);

	/* Vendor clock register before HC mode, Linux's order. */
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_POR_VAL);
	if (sdhci_enter_hc_mode() != 0) {
		return ENXIO;
	}

	sdhci_pwrctl_ack("stale");
	ctrl = hc_rd8(SDHCI_HOST_CONTROL);

	hc_wr8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_ALL);
	deadline = host_now_ns() + 100000000ull;
	while (hc_rd8(SDHCI_SOFTWARE_RESET) & SDHCI_RESET_ALL) {
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(5);
	}
	sdhci_pwrctl_ack("BUS_OFF");

	sdhci_select_hc_clock();

	hc_wr8(SDHCI_POWER_CONTROL, SDHCI_POWER_CONTROL_VAL);
	sdhci_pwrctl_ack("BUS_ON");

	hc_wr16(SDHCI_CLOCK_CONTROL, 0);
	hc_wr16(SDHCI_CLOCK_CONTROL, SDHCI_CLOCK_INT_EN);
	deadline = host_now_ns() + 150000000ull;
	while (!(hc_rd16(SDHCI_CLOCK_CONTROL) & SDHCI_CLOCK_INT_STABLE)) {
		if (host_now_ns() > deadline) {
			return ENXIO;
		}
		host_delay_us(5);
	}
	hc_wr16(SDHCI_CLOCK_CONTROL,
	    SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_INT_STABLE | SDHCI_CLOCK_CARD_EN);
	clk = hc_rd16(SDHCI_CLOCK_CONTROL);
	if (!(clk & SDHCI_CLOCK_CARD_EN)) {
		return ENXIO;
	}
	sdhci_clock_output();

	hc_wr8(SDHCI_HOST_CONTROL, (uint8_t)(ctrl & ~SDHCI_CTRL_DMA_MASK));
	to_ctrl = hc_rd8(SDHCI_TIMEOUT_CONTROL);
	if (to_ctrl == 0) {
		hc_wr8(SDHCI_TIMEOUT_CONTROL, 0x0eu);
	}

	/* Polled operation: status enabled, interrupt signal not routed. */
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr(SDHCI_SIGNAL_ENABLE, 0);
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));

	return 0;
}

/* Wait until PRESENT_STATE no longer reports the given inhibit bits. */
static int
sdhci_poll_state(uint32_t mask, uint64_t timeout_us)
{
	uint64_t deadline = host_now_ns() + timeout_us * 1000ull;

	while (hc_rd(SDHCI_PRESENT_STATE) & mask) {
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(10);
	}
	return 0;
}

static int
sdhci_cmd(uint8_t index, uint32_t arg, uint16_t tr_mode, uint16_t flags,
    uint32_t *resp)
{
	int error = sdhci_poll_state(SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT,
	    CMD_INHIBIT_US);
	if (error) {
		return EBUSY;
	}

	/* A stale Command Complete from the bootloader must not be read as
	 * this command's result. */
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));
	hc_wr(SDHCI_ARGUMENT, arg);
	hc_wr16(SDHCI_TRANSFER_MODE, tr_mode);
	hc_wr16(SDHCI_COMMAND, (uint16_t)SDHCI_MAKE_CMD(index, flags));

	uint64_t deadline = host_now_ns() + CMD_TIMEOUT_US * 1000ull;
	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);

		if (st & SDHCI_INT_ERR) {
			hc_wr(SDHCI_INT_STATUS, st);
			return EIO;
		}
		if (st & SDHCI_INT_CC) {
			hc_wr(SDHCI_INT_STATUS, SDHCI_INT_CC);
			if (resp) {
				*resp = hc_rd(SDHCI_RESPONSE);
			}
			return 0;
		}
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(20);
	}
}

static int
sdhci_read_response128(uint8_t index, uint32_t arg, uint32_t out[4])
{
	int error = sdhci_cmd(index, arg, 0,
	    SDHCI_CMD_RESP_LONG | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
	if (error) {
		return error;
	}
	/* RESPONSE[0] is bits 31:0; out[0] must be bits 127:96. The core
	 * stripped the CRC, so each word is shifted left by 8 and the top
	 * byte of the next word is folded in (Linux sdhci_read_rsp_136). */
	for (int i = 0; i < 4; i++) {
		uint32_t lo = hc_rd(SDHCI_RESPONSE + 4 * (3 - i));
		uint32_t hi = (i == 3) ? 0u :
		    hc_rd(SDHCI_RESPONSE + 4 * (2 - i));

		out[i] = (lo << 8) | (hi >> 24);
	}
	return 0;
}

static int
sdhci_read_cid_csd(uint32_t rca, uint32_t cid[4], uint32_t csd[4])
{
	int error = sdhci_read_response128(10, rca << 16, cid);
	if (error) {
		return error;
	}
	return sdhci_read_response128(9, rca << 16, csd);
}

/* Poll one data interrupt (BRR for reads, BWR for writes) and clear it. */
static int
sdhci_wait_data(uint32_t bit, int *saw_error)
{
	uint64_t deadline = host_now_ns() + CMD_TIMEOUT_US * 1000ull;

	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);

		if (st & SDHCI_INT_ERR) {
			hc_wr(SDHCI_INT_STATUS, st);
			*saw_error = 1;
			return EIO;
		}
		if (st & bit) {
			hc_wr(SDHCI_INT_STATUS, bit);
			return 0;
		}
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(20);
	}
}

static int
sdhci_wait_transfer_complete(void)
{
	uint64_t deadline = host_now_ns() + CMD_TIMEOUT_US * 1000ull;

	for (;;) {
		uint32_t st = hc_rd(SDHCI_INT_STATUS);

		if (st & SDHCI_INT_ERR) {
			hc_wr(SDHCI_INT_STATUS, st);
			return EIO;
		}
		if (st & SDHCI_INT_TC) {
			hc_wr(SDHCI_INT_STATUS, SDHCI_INT_TC);
			return 0;
		}
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(20);
	}
}

static int
sdhci_stop_transmission(void)
{
	return sdhci_cmd(12, 0, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
}

/* Reset the command and data paths; the core self-clears when done. */
static int
sdhci_reset_paths(void)
{
	hc_wr8(SDHCI_SOFTWARE_RESET, SDHCI_RESET_CMD | SDHCI_RESET_DATA);
	uint64_t deadline = host_now_ns() + CMD_INHIBIT_US * 1000ull;
	while (hc_rd8(SDHCI_SOFTWARE_RESET) & (SDHCI_RESET_CMD | SDHCI_RESET_DATA)) {
		if (host_now_ns() > deadline) {
			return ETIMEDOUT;
		}
		host_delay_us(5);
	}
	hc_wr(SDHCI_INT_STATUS, hc_rd(SDHCI_INT_STATUS));
	return 0;
}

/* Reset, best-effort CMD12, reset again: a timed-out CMD12 leaves
 * CMD_INHIBIT set, and the next command needs a clean command path. */
static int
sdhci_recover(void)
{
	(void)sdhci_reset_paths();
	(void)sdhci_stop_transmission();
	(void)sdhci_reset_paths();
	return 0;
}

/* ------------------------------------------------ RCA ladder (driver logic) */

static uint32_t drv_rca;

static int
sdhci_send_status(uint32_t rca, uint32_t *r1)
{
	int error = sdhci_cmd(13, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, r1);
	if (error) {
		sdhci_recover();
	}
	return error;
}

static int
sdhci_select_card(uint32_t rca)
{
	return sdhci_cmd(7, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, NULL);
}

static int
sdhci_sleep_awake(uint32_t rca, uint32_t *r1)
{
	return sdhci_cmd(5, rca << 16, 0,
	    SDHCI_CMD_RESP_SHORT_BUSY | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, r1);
}

static int
sdhci_restore_state(uint32_t *r1)
{
	for (int step = 0; step < 3; step++) {
		uint32_t state = (*r1 >> 9) & 0xf;

		if (state == 4) {               /* TRAN */
			return 0;
		}
		if (state == 3) {
			if (sdhci_select_card(drv_rca) != 0) {
				return EIO;
			}
		} else if (state == 5) {
			if (sdhci_sleep_awake(drv_rca, r1) != 0) {
				return EIO;
			}
		} else {
			return ENXIO;
		}
		if (sdhci_send_status(drv_rca, r1) != 0) {
			return EIO;
		}
	}
	return ENXIO;
}

static int
sdhci_claim_card(void)
{
	static const uint32_t candidates[] = { SANTONI_CARD_RCA, 1u };
	uint32_t r1 = 0;
	int error = ETIMEDOUT;
	unsigned i;

	for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
		error = sdhci_send_status(candidates[i], &r1);
		if (!error) {
			break;
		}
	}
	if (!error) {
		drv_rca = candidates[i];
		return sdhci_restore_state(&r1);
	}

	/* Rung 3: Linux sdhci_msm_probe() resets HC + 0x10c to POR. */
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_POR_VAL);
	(void)sdhci_reset_paths();
	if (sdhci_send_status(SANTONI_CARD_RCA, &r1) == 0) {
		drv_rca = SANTONI_CARD_RCA;
		return sdhci_restore_state(&r1);
	}

	/* Rung 4: a sleeping card answers CMD5 but not CMD13. */
	for (i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++) {
		error = sdhci_sleep_awake(candidates[i], &r1);
		if (error) {
			(void)sdhci_reset_paths();
			continue;
		}
		drv_rca = candidates[i];
		if (sdhci_send_status(drv_rca, &r1) != 0) {
			continue;
		}
		return sdhci_restore_state(&r1);
	}

	/* Rung 5: CMD0 needs no response; last because it resets the card. */
	(void)sdhci_cmd(0, 0, 0, 0, NULL);
	return ENXIO;
}

static int
sdhci_read_blocks_pio(uint64_t lba, uint32_t count, void *buf)
{
	if (count == 0 || count > SDHCI_MAX_BLOCKS || lba > 0xffffffffull) {
		return EINVAL;
	}
	hc_wr16(SDHCI_BLOCK_SIZE, SDHCI_MAKE_BLKSZ(0, SDHCI_BLOCK));
	hc_wr16(SDHCI_BLOCK_COUNT, (uint16_t)count);

	int error = sdhci_cmd(18, (uint32_t)lba,
	    SDHCI_TRNS_MULTI | SDHCI_TRNS_READ | SDHCI_TRNS_BLK_CNT_EN,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, NULL);
	if (error) {
		sdhci_recover();
		return error;
	}

	uint32_t *dst = (uint32_t *)buf;
	for (uint32_t b = 0; b < count; b++) {
		int saw_error = 0;

		error = sdhci_wait_data(SDHCI_INT_BRR, &saw_error);
		if (error) {
			sdhci_recover();
			return error;
		}
		for (uint32_t i = 0; i < SDHCI_BLOCK / 4; i++) {
			dst[b * (SDHCI_BLOCK / 4) + i] = hc_rd(SDHCI_BUFFER);
		}
	}
	error = sdhci_wait_transfer_complete();
	if (error) {
		sdhci_recover();
		return error;
	}
	/* BLK_CNT_EN stops the card at the block count: no CMD12 on success. */
	return 0;
}

static int
sdhci_write_blocks_pio(uint64_t lba, uint32_t count, const void *buf)
{
	if (count == 0 || count > SDHCI_MAX_BLOCKS || lba > 0xffffffffull) {
		return EINVAL;
	}
	hc_wr16(SDHCI_BLOCK_SIZE, SDHCI_MAKE_BLKSZ(0, SDHCI_BLOCK));
	hc_wr16(SDHCI_BLOCK_COUNT, (uint16_t)count);

	int error = sdhci_cmd(25, (uint32_t)lba,
	    SDHCI_TRNS_MULTI | SDHCI_TRNS_BLK_CNT_EN,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_DATA, NULL);
	if (error) {
		sdhci_recover();
		return error;
	}

	const uint32_t *src = (const uint32_t *)buf;
	for (uint32_t b = 0; b < count; b++) {
		int saw_error = 0;

		error = sdhci_wait_data(SDHCI_INT_BWR, &saw_error);
		if (error) {
			sdhci_recover();
			return error;
		}
		for (uint32_t i = 0; i < SDHCI_BLOCK / 4; i++) {
			hc_wr(SDHCI_BUFFER, src[b * (SDHCI_BLOCK / 4) + i]);
		}
	}
	error = sdhci_wait_transfer_complete();
	if (error) {
		sdhci_recover();
		return error;
	}
	/* BLK_CNT_EN stops the card at the block count: no CMD12 on success. */
	return 0;
}

/* -------------------------------------------------------------- test suite */

static void
reset_mock(void)
{
	memset(&m, 0, sizeof(m));
	memset(mock_hc, 0, sizeof(mock_hc));
	memset(mock_core, 0, sizeof(mock_core));

	/* LK hand-off: SDCC1 runs through the legacy MCI register set, so HC
	 * mode is off and the standard window is not decoded yet. The legacy
	 * path left the card clock running at a valid divider and the eMMC on
	 * an 8-bit bus. */
	core_wr(CORE_MCI_VERSION, 0x1000004du);          /* measured on hardware */
	core_wr(CORE_GENERICS, 0x24100041u);
	core_wr(CORE_HC_MODE, 0x00000000u);
	core_wr(CORE_PWRCTL_MASK, 0x00000000u);
	/* LK never writes the vendor clock register (0x00000000 on hardware):
	 * MCLK select 0, so the HC datapath has no functional clock until
	 * takeover selects it. */
	hc_wr(CORE_VENDOR_SPEC, 0x00000000u);
	/* DLL untouched by LK: neither enabled nor selected (bypass). */
	hc_wr(CORE_DLL_CONFIG, 0x00000000u);
	hc_wr(CORE_DLL_STATUS, 0x00000000u);
	core_wr(CORE_MCI_CLK, 0x90000000u);              /* legacy MCI clock */
	core_wr(CORE_MCI_POWER, 0x00000041u);
	m.rca = SANTONI_CARD_RCA;
	mock_wr16(SDHCI_HOST_VERSION, 0x4d02);           /* vendor 0x4d, 3.0 */
	mock_wr16(SDHCI_CLOCK_CONTROL,
	    SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_INT_STABLE | SDHCI_CLOCK_CARD_EN);
	mock_hc[SDHCI_HOST_CONTROL] = SDHCI_CTRL_8BITBUS | SDHCI_CTRL_DMA_MASK;

	for (uint32_t b = 0; b < MOCK_CARD_BLOCKS; b++) {
		for (uint32_t i = 0; i < SDHCI_BLOCK; i++) {
			mock_card[b * SDHCI_BLOCK + i] =
			    (uint8_t)(0x5a ^ (b * 7u) ^ i);
		}
	}
}

static void
test_registers(void)
{
	/* Standard SDHCI register file */
	assert(SDHCI_BLOCK_SIZE == 0x04);
	assert(SDHCI_BLOCK_COUNT == 0x06);
	assert(SDHCI_ARGUMENT == 0x08);
	assert(SDHCI_TRANSFER_MODE == 0x0C);
	assert(SDHCI_COMMAND == 0x0E);
	assert(SDHCI_RESPONSE == 0x10);
	assert(SDHCI_BUFFER == 0x20);
	assert(SDHCI_PRESENT_STATE == 0x24);
	assert(SDHCI_HOST_CONTROL == 0x28);
	assert(SDHCI_CLOCK_CONTROL == 0x2C);
	assert(SDHCI_SOFTWARE_RESET == 0x2F);
	assert(SDHCI_INT_STATUS == 0x30);

	/* Qualcomm MSM v4 vendor core registers (sdhci_msm_mci_offset) */
	assert(CORE_MCI_VERSION == 0x50);
	assert(CORE_GENERICS == 0x70);
	assert(CORE_HC_MODE == 0x78);
	assert(CORE_PWRCTL_STATUS == 0xdc);
	assert(CORE_PWRCTL_MASK == 0xe0);
	assert(CORE_PWRCTL_CLEAR == 0xe4);
	assert(CORE_PWRCTL_CTL == 0xe8);
	assert(CORE_PWRCTL_BUS_OFF == 0x01);
	assert(CORE_PWRCTL_BUS_ON == 0x02);
	assert(CORE_PWRCTL_BUS_SUCCESS == 0x01);
	assert(CORE_PWRCTL_IO_SUCCESS == 0x04);
	assert(CORE_HC_MODE_EN == (1u << 0));
	assert(CORE_FF_CLK_SW_RST_DIS == (1u << 13));

	/* HC-mode takeover registers */
	assert(SDHCI_HOST_VERSION == 0xfe);
	assert(SDHCI_POWER_CONTROL == 0x29);
	assert(SDHCI_POWER_CONTROL_VAL == 0x0b);
	assert(SDHCI_CLOCK_INT_EN == 0x1);
	assert(SDHCI_CLOCK_INT_STABLE == 0x2);
	assert(SDHCI_CLOCK_CARD_EN == 0x4);

	/* Vendor clock / DLL registers (sdhci_msm_mci_offset) */
	assert(CORE_DLL_CONFIG == 0x100);
	assert(CORE_DLL_STATUS == 0x108);
	assert(CORE_DLL_LOCK == (1u << 7));
	assert(CORE_DLL_EN == (1u << 16));
	assert(CORE_CK_OUT_EN == (1u << 18));
	assert(CORE_DLL_PDN == (1u << 29));
	assert(CORE_DLL_RST == (1u << 30));
	assert(CORE_HC_SELECT_IN_EN == (1u << 18));
	assert((CORE_VENDOR_SPEC_POR_VAL & CORE_HC_MCLK_SEL_MASK) ==
	    CORE_HC_MCLK_SEL_DFLT);
	assert((CORE_VENDOR_SPEC_POR_VAL & CORE_HC_SELECT_IN_EN) == 0);

	/* One 0x1000 MMIO window maps core + HC (0x900) including its 0x500 */
	assert(SDHCI_HC_OFFSET == 0x900);
	assert(SDHCI_HC_OFFSET + SDHCI_HC_SIZE <= SDHCI_CORE_SIZE);

	/* Command framing constants used by the driver */
	assert(SDHCI_MAKE_CMD(18, 0x3a) == 0x123a);
	assert(SDHCI_MAKE_CMD(12, 0x1b) == 0x0c1b);
	assert(SDHCI_MAKE_CMD(13, 0x1a) == 0x0d1a);
	assert(SDHCI_TRNS_READ | SDHCI_TRNS_MULTI | SDHCI_TRNS_BLK_CNT_EN == 0x32);
	assert(SDHCI_CMD_DATA | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	    SDHCI_CMD_RESP_SHORT == 0x3a);
}

static void
test_take_over(void)
{
	reset_mock();
	/* LK leaves the core in MCI mode: the driver performs the switch. */
	assert(core_rd(CORE_HC_MODE) == 0);
	assert(sdhci_take_over() == 0);
	assert((core_rd(CORE_HC_MODE) &
	    (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS)) ==
	    (CORE_HC_MODE_EN | CORE_FF_CLK_SW_RST_DIS));
	/* RESET_ALL issued once and acknowledged */
	assert(m.reset_value == SDHCI_RESET_ALL);
	assert(m.reset_writes == 1);
	/* Power turned on (0x0B = 1.8V + bus on) and acknowledged */
	assert(hc_rd8(SDHCI_POWER_CONTROL) == SDHCI_POWER_CONTROL_VAL);
	assert(core_rd(CORE_PWRCTL_CTL) == CORE_PWRCTL_BUS_SUCCESS);
	/* Clock stabilized and card clock gated on */
	assert(mock_rd16(SDHCI_CLOCK_CONTROL) ==
	    (SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_INT_STABLE | SDHCI_CLOCK_CARD_EN));
	/* 8-bit bus preserved, DMA select cleared for PIO. */
	assert((hc_rd8(SDHCI_HOST_CONTROL) & SDHCI_CTRL_8BITBUS) != 0);
	assert((hc_rd8(SDHCI_HOST_CONTROL) & SDHCI_CTRL_DMA_MASK) == 0);
	/* Polled: status enabled so the loop can see completions, but the
	 * interrupt signal is not routed anywhere. */
	assert(hc_rd(SDHCI_INT_ENABLE) == 0xffffffffu);
	assert(hc_rd(SDHCI_SIGNAL_ENABLE) == 0);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
	assert(hc_rd16(SDHCI_BLOCK_SIZE) == 0);

	/* The vendor clock register is at its POR value before HC mode is
	 * enabled, and free-running MCLK is selected for the HC datapath. */
	assert(hc_rd(CORE_VENDOR_SPEC) == CORE_VENDOR_SPEC_POR_VAL);
	assert((hc_rd(CORE_VENDOR_SPEC) & CORE_HC_MCLK_SEL_MASK) ==
	    CORE_HC_MCLK_SEL_DFLT);
	assert((hc_rd(CORE_VENDOR_SPEC) & CORE_HC_SELECT_IN_EN) == 0);
	assert(mock_hc_clocked());

	/* Dead GCC card clock: fail closed, RAM-disk boot survives. */
	reset_mock();
	m.clock_stalled = 1;
	assert(sdhci_take_over() == ENXIO);

	/* An unclocked vendor core is not ours. */
	reset_mock();
	core_wr(CORE_MCI_VERSION, 0xffffffffu);
	assert(sdhci_take_over() == ENXIO);

	/* HC_MODE_EN does not latch and the HC window is dead. */
	reset_mock();
	m.hc_mode_write_ignored = 1;
	mock_wr16(SDHCI_HOST_VERSION, 0x0000);
	assert(sdhci_take_over() == ENXIO);

	/* Bit 0 is write-only but the switch worked: proceed. */
	reset_mock();
	m.hc_mode_write_ignored = 1;
	assert(sdhci_take_over() == 0);
}

static void
test_clock_path(void)
{
	uint32_t resp = 0;
	const uint16_t cmd13 = (uint16_t)SDHCI_MAKE_CMD(13,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX);

	/*
	 * Model check against the photographed hand-off: VENDOR_SPEC 0x0 means
	 * MCLK select 0, and a command then never leaves the shifter. CMD0 is
	 * the witness - it needs no response and no card, so a timeout here is
	 * the controller, not the card.
	 */
	reset_mock();
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr(SDHCI_INT_STATUS, 0xffffffffu);
	hc_wr16(SDHCI_ARGUMENT, 0);
	hc_wr16(SDHCI_TRANSFER_MODE, 0);
	hc_wr16(SDHCI_COMMAND, (uint16_t)SDHCI_MAKE_CMD(0, 0));
	assert((hc_rd(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT) != 0);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
	reset_mock();

	/*
	 * Photographed bug test: writing CORE_VENDOR_SPEC_POR_VAL to core_wr
	 * lands in legacy MCI space where bits 9:8 are RO 0, leaving mock_hc
	 * unclocked. CMD0 still times out with CMD_INHIBIT, while mock_hc is untouched.
	 */
	core_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_POR_VAL);
	assert(hc_rd(CORE_VENDOR_SPEC) == 0);
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr16(SDHCI_COMMAND, (uint16_t)SDHCI_MAKE_CMD(0, 0));
	assert((hc_rd(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT) != 0);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
	reset_mock();

	/* Same with the HC clock selected but SDCLK taken from a DLL that is
	 * not running: still no clock, still no completion. */
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_POR_VAL);
	hc_wr(CORE_DLL_CONFIG, CORE_CK_OUT_EN);
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr8(SDHCI_POWER_CONTROL, SDHCI_POWER_CONTROL_VAL);
	hc_wr16(SDHCI_COMMAND, (uint16_t)SDHCI_MAKE_CMD(0, 0));
	assert((hc_rd(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT) != 0);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);

	/* Bus power off: command never completes, CMD_INHIBIT stuck */
	reset_mock();
	hc_wr(CORE_VENDOR_SPEC, CORE_VENDOR_SPEC_POR_VAL);
	hc_wr(SDHCI_INT_ENABLE, 0xffffffffu);
	hc_wr8(SDHCI_POWER_CONTROL, 0);
	hc_wr16(SDHCI_COMMAND, (uint16_t)SDHCI_MAKE_CMD(0, 0));
	assert((hc_rd(SDHCI_PRESENT_STATE) & SDHCI_CMD_INHIBIT) != 0);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);

	/* Takeover selects free-running MCLK via hc_wr before the first command, so the
	 * same CMD13 that used to time out now completes. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	assert((hc_rd(CORE_VENDOR_SPEC) & CORE_HC_MCLK_SEL_MASK) ==
	    CORE_HC_MCLK_SEL_DFLT);
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == 0);
	assert(resp == 0x00000900u);

	/* A locked DLL left running by LK: its output is the card clock. */
	reset_mock();
	hc_wr(CORE_DLL_CONFIG, CORE_DLL_EN);
	hc_wr(CORE_DLL_STATUS, CORE_DLL_LOCK);
	assert(sdhci_take_over() == 0);
	assert((hc_rd(CORE_DLL_CONFIG) & CORE_CK_OUT_EN) != 0);
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == 0);

	/* A DLL held in reset must not be the clock source: fall back to
	 * bypass instead of driving a dead clock to the card. */
	reset_mock();
	hc_wr(CORE_DLL_CONFIG, CORE_CK_OUT_EN | CORE_DLL_RST);
	assert(sdhci_take_over() == 0);
	assert((hc_rd(CORE_DLL_CONFIG) & CORE_CK_OUT_EN) == 0);
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == 0);

	/* An enabled but unlocked DLL is not usable either. */
	reset_mock();
	hc_wr(CORE_DLL_CONFIG, CORE_DLL_EN | CORE_CK_OUT_EN);
	assert(sdhci_take_over() == 0);
	assert((hc_rd(CORE_DLL_CONFIG) & CORE_CK_OUT_EN) == 0);

	/* The status model gates 0x30 on 0x34, so the driver must leave the
	 * enable all ones; clearing it reproduces the "INT_STATUS 0x00000000"
	 * of the hardware log and every command times out. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	assert(hc_rd(SDHCI_INT_ENABLE) == 0xffffffffu);
	assert(hc_rd(SDHCI_SIGNAL_ENABLE) == 0);
	hc_wr(SDHCI_INT_ENABLE, 0);
	hc_wr16(SDHCI_COMMAND, cmd13);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) ==
	    ETIMEDOUT);
}

static void
test_cmd13_status(void)
{
	uint32_t resp = 0;
	int error;

	reset_mock();
	assert(sdhci_take_over() == 0);

	m.inhibit_polls = 3;    /* controller busy for the first three polls */
	error = sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp);
	assert(error == 0);
	assert(m.state_reads >= 4);
	assert(resp == 0x00000900u);
	assert(((resp >> 9) & 0xf) == 4);       /* TRAN */

	/* ARGUMENT / TRANSFER_MODE / COMMAND bytes actually hit the FIFO, and
	 * CMD13 carries the RCA in bits 31:16 like every addressed command. */
	assert(m.cmd_count == 1);
	assert(m.trace[0].cmd == 13);
	assert(m.trace[0].arg == SANTONI_CARD_RCA << 16);
	assert(m.trace[0].tr_mode == 0);
	assert(mock_rd32(SDHCI_ARGUMENT) == SANTONI_CARD_RCA << 16);
	assert(m.trace[0].word == 0x0d1a);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);   /* CC acknowledged */
}

static void
test_cid_csd(void)
{
	uint32_t cid[4] = { 0 }, csd[4] = { 0 };
	char pnm[7];

	reset_mock();
	assert(sdhci_take_over() == 0);
	assert(sdhci_read_cid_csd(SANTONI_CARD_RCA, cid, csd) == 0);

	assert(m.cmd_count == 2);
	assert(m.trace[0].cmd == 10);
	assert(m.trace[0].arg == SANTONI_CARD_RCA << 16);
	assert(m.trace[1].cmd == 9);
	assert(m.trace[1].arg == SANTONI_CARD_RCA << 16);

	/* The core strips the CRC and right-aligns the 120-bit payload, so the
	 * driver shifts each word by 8 (Linux sdhci_read_rsp_136): the decoded
	 * words are the real CID/CSD, not the raw register contents. */
	assert(cid[0] == 0x15010038u);
	assert(cid[1] == 0x47544634u);
	assert(cid[2] == 0x52011234u);
	assert(cid[3] == 0x56782100u);
	assert(csd[0] == 0xd00f590au);
	assert(csd[1] == 0x0f5a83b9u);
	assert(csd[2] == 0x3b9aca00u);
	assert(csd[3] == 0x00000000u);

	/* eMMC CID: MID[127:120], PNM[103:56] (6 ASCII), PRV[55:48],
	 * PSN[47:16], MDT[15:8]. */
	pnm[0] = (char)(cid[0] & 0xff);
	pnm[1] = (char)(cid[1] >> 24);
	pnm[2] = (char)(cid[1] >> 16);
	pnm[3] = (char)(cid[1] >> 8);
	pnm[4] = (char)cid[1];
	pnm[5] = (char)(cid[2] >> 24);
	pnm[6] = 0;
	assert(((cid[0] >> 24) & 0xff) == 0x15);        /* MID */
	assert(strcmp(pnm, "8GTF4R") == 0);             /* 6-character PNM */
	assert(((cid[2] >> 20) & 0xf) == 0);            /* PRV major */
	assert(((cid[2] >> 16) & 0xf) == 1);            /* PRV minor */
	assert((((cid[2] & 0xffffu) << 16) | (cid[3] >> 16)) == 0x12345678u);
	assert(((cid[3] >> 8) & 0xff) == 0x21);         /* MDT */
}

static void
test_card_ladder(void)
{
	int saw_cmd5;

	/* RCA 2 answers: the LK/ABL MMC_RCA, tried first. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.cmd_count = 0;
	assert(sdhci_claim_card() == 0);
	assert(drv_rca == SANTONI_CARD_RCA);
	assert(m.trace[0].cmd == 13);
	assert(m.trace[0].arg == SANTONI_CARD_RCA << 16);

	/* A card on RCA 1: the first rung gets no response, the ladder must
	 * recover the command path and then succeed on the second rung. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.rca = 1;
	m.cmd_count = 0;
	assert(sdhci_claim_card() == 0);
	assert(drv_rca == 1);
	assert(m.trace[0].cmd == 13);
	assert(m.trace[0].arg == SANTONI_CARD_RCA << 16);
	assert(m.trace[1].cmd == 12);           /* recover: best-effort CMD12 */
	assert(m.trace[2].cmd == 13);
	assert(m.trace[2].arg == 1u << 16);

	/* Sleep: CMD13 is ignored, the vendor rung runs (no answer), then CMD5
	 * wakes the card and the RCA it reports is used. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.card_asleep = 1;
	m.cmd_count = 0;
	assert(sdhci_claim_card() == 0);
	assert(drv_rca == SANTONI_CARD_RCA);
	assert(m.card_asleep == 0);
	assert(hc_rd(CORE_VENDOR_SPEC) == CORE_VENDOR_SPEC_POR_VAL);
	saw_cmd5 = 0;
	for (unsigned i = 0; i < m.cmd_count; i++) {
		if (m.trace[i].cmd == 5) {
			saw_cmd5 = 1;
			assert(m.trace[i].arg == SANTONI_CARD_RCA << 16);
		}
	}
	assert(saw_cmd5);

	/* Standby: LK deselected the card with CMD7 arg 0 before sleeping. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	{
		/* CMD13 reported Standby: the repair is CMD7 select. */
		uint32_t standby = 0x00000600u;

		drv_rca = SANTONI_CARD_RCA;
		assert(sdhci_restore_state(&standby) == 0);
	}
	assert(m.trace[0].cmd == 7);
	assert(m.trace[0].arg == SANTONI_CARD_RCA << 16);
	assert(m.trace[1].cmd == 13);

	/* Nothing on the bus: every rung runs and CMD0 (no response) is the
	 * final liveness test, after which the probe fails closed. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.card_absent = 1;
	m.cmd_count = 0;
	assert(sdhci_claim_card() == ENXIO);
	assert(m.trace[m.cmd_count - 1].cmd == 0);
}

static void
test_read_pio(void)
{
	static uint8_t buf[3 * SDHCI_BLOCK];
	int error;

	reset_mock();
	assert(sdhci_take_over() == 0);
	memset(buf, 0, sizeof(buf));
	error = sdhci_read_blocks_pio(2, 3, buf);
	assert(error == 0);

	/* Block geometry and multi-block read framing. */
	assert(m.blk_size == SDHCI_BLOCK);
	assert(m.blk_count == 3);
	assert(m.trace[0].cmd == 18);
	assert(m.trace[0].arg == 2);           /* sector, not byte, addressing */
	assert(m.trace[0].tr_mode == 0x32);
	/* CMD18: R1 + CRC + index + data (0x3a) */
	assert(m.trace[0].word == 0x123a);

	/* Data really came from the addressed card sectors. */
	assert(memcmp(buf, mock_card + 2 * SDHCI_BLOCK, sizeof(buf)) == 0);

	/* BLK_CNT_EN stops the card itself: no CMD12 on the success path. */
	assert(m.cmd_count == 1);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
}

static void
test_write_pio(void)
{
	static uint8_t buf[2 * SDHCI_BLOCK];
	int error;

	reset_mock();
	assert(sdhci_take_over() == 0);
	for (uint32_t i = 0; i < sizeof(buf); i++) {
		buf[i] = (uint8_t)(0xa5 ^ (i * 3u));
	}
	error = sdhci_write_blocks_pio(7, 2, buf);
	assert(error == 0);

	assert(m.trace[0].cmd == 25);
	assert(m.trace[0].arg == 7);
	assert(m.trace[0].tr_mode == 0x22);     /* MULTI | BLK_CNT_EN, no READ */
	assert(m.trace[0].word == 0x193a);
	assert(memcmp(mock_card + 7 * SDHCI_BLOCK, buf, sizeof(buf)) == 0);
	/* No CMD12 after a successful BLK_CNT_EN transfer. */
	assert(m.cmd_count == 1);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);
}

static void
test_bounds(void)
{
	static uint8_t buf[SDHCI_BLOCK];

	reset_mock();
	assert(sdhci_take_over() == 0);

	assert(sdhci_read_blocks_pio(0, 0, buf) == EINVAL);
	assert(sdhci_read_blocks_pio(0, SDHCI_MAX_BLOCKS + 1, buf) == EINVAL);
	assert(sdhci_read_blocks_pio(0x100000000ull, 1, buf) == EINVAL);
	assert(sdhci_write_blocks_pio(0, 0, buf) == EINVAL);
	assert(sdhci_write_blocks_pio(0, SDHCI_MAX_BLOCKS + 1, buf) == EINVAL);
	/* Rejected requests never touch the controller. */
	assert(m.cmd_count == 0);
	assert(mock_rd16(SDHCI_COMMAND) == 0);
}

static void
test_errors(void)
{
	uint32_t resp = 0;

	/* Command error: INT_STATUS error bit, driver clears it and reports EIO */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.fail_cmd = 1;
	assert(sdhci_cmd(13, 0, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == EIO);
	assert(hc_rd(SDHCI_INT_STATUS) == 0);

	/* Data error during a read: recover resets CMD|DATA, then a best-effort
	 * CMD12 leaves the card out of its data state. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	{
		static uint8_t buf[2 * SDHCI_BLOCK];

		m.fail_data = 1;
		assert(sdhci_read_blocks_pio(0, 2, buf) == EIO);
		assert(m.reset_writes >= 1);
		assert(m.trace[m.cmd_count - 1].cmd == 12);
	}

	/* Timeout: the command never completes, but we must not hang */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.stall_cmd = 1;
	{
		uint64_t start = host_now_ns();
		int error = sdhci_cmd(13, 0, 0,
		    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp);

		assert(error == ETIMEDOUT);
		assert(host_now_ns() - start >= (uint64_t)CMD_TIMEOUT_US * 1000ull);
	}

	/* A stale Command Complete from the bootloader must not be read as this
	 * command's result: the driver clears INT_STATUS before it writes
	 * COMMAND, so a stalled command still times out. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.int_status = SDHCI_INT_CC;
	m.stall_cmd = 1;
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) ==
	    ETIMEDOUT);

	/* Inhibit that clears: the driver retries rather than failing */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.inhibit_polls = 5;
	assert(sdhci_cmd(13, SANTONI_CARD_RCA << 16, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == 0);

	/* DATA_INHIBIT (PRESENT_STATE bit 1) is waited on as well: a stuck data
	 * path must fail closed with EBUSY, not issue the command. */
	reset_mock();
	assert(sdhci_take_over() == 0);
	m.present_state = SDHCI_DATA_INHIBIT;
	assert(sdhci_cmd(13, 0, 0,
	    SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX, &resp) == EBUSY);
	assert(m.cmd_count == 0);
}

int
main(void)
{
	test_registers();
	test_take_over();
	test_clock_path();
	test_cmd13_status();
	test_cid_csd();
	test_card_ladder();
	test_read_pio();
	test_write_pio();
	test_bounds();
	test_errors();

	printf("test_sdhci_mock: all assertions passed\n");
	return 0;
}
