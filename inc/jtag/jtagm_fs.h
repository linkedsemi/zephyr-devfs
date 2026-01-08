/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (c) 2018 Mellanox Technologies. All rights reserved. */
/* Copyright (c) 2018 Oleksandr Shamray <oleksandrs@mellanox.com> */
/* Copyright (c) 2019 Intel Corporation */

#ifndef __UAPI_LINUX_JTAG_H
#define __UAPI_LINUX_JTAG_H

#include <linux/ioctl.h>

/*
 * JTAG_XFER_MODE: JTAG transfer mode. Used to set JTAG controller transfer mode
 * This is bitmask for feature param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_XFER_MODE 0
/*
 * JTAG_CONTROL_MODE: JTAG controller mode. Used to set JTAG controller mode
 * This is bitmask for feature param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_CONTROL_MODE 1
/*
 * JTAG_TCK_CYCLE_DELAY_COUNT: JTAG delay counter for aspeed_jtag_tck_cycle. Used
 * set the number of jtag_tck_cycle delays repetitions.
 * This is bitmask for feature param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_TCK_CYCLE_DELAY_COUNT 2
/*
 * JTAG_CONTROLLER_OUTPUT_DISABLE: JTAG controller mode output disable, it is
 * used to enable other devices to own the JTAG bus.
 * This is bitmask for mode param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_CONTROLLER_OUTPUT_DISABLE 0
/*
 * JTAG_CONTROLLER_MODE: JTAG controller mode. Used to set JTAG controller in
 * host mode.
 * This is bitmask for mode param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_CONTROLLER_MODE 1
/*
 * JTAG_XFER_HW_MODE: JTAG hardware mode. Used to set HW drived or bitbang
 * mode. This is bitmask for mode param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_XFER_HW_MODE 1
/*
 * JTAG_XFER_SW_MODE: JTAG software mode. Used to set SW drived or bitbang
 * mode. This is bitmask for mode param in jtag_mode for ioctl JTAG_SIOCMODE
 */
#define  JTAG_XFER_SW_MODE 0

/**
 * enum jtag_tapstate:
 *
 * @JTAG_STATE_TLRESET: JTAG state machine Test Logic Reset state
 * @JTAG_STATE_IDLE: JTAG state machine IDLE state
 * @JTAG_STATE_SELECTDR: JTAG state machine SELECT_DR state
 * @JTAG_STATE_CAPTUREDR: JTAG state machine CAPTURE_DR state
 * @JTAG_STATE_SHIFTDR: JTAG state machine SHIFT_DR state
 * @JTAG_STATE_EXIT1DR: JTAG state machine EXIT-1 DR state
 * @JTAG_STATE_PAUSEDR: JTAG state machine PAUSE_DR state
 * @JTAG_STATE_EXIT2DR: JTAG state machine EXIT-2 DR state
 * @JTAG_STATE_UPDATEDR: JTAG state machine UPDATE DR state
 * @JTAG_STATE_SELECTIR: JTAG state machine SELECT_IR state
 * @JTAG_STATE_CAPTUREIR: JTAG state machine CAPTURE_IR state
 * @JTAG_STATE_SHIFTIR: JTAG state machine SHIFT_IR state
 * @JTAG_STATE_EXIT1IR: JTAG state machine EXIT-1 IR state
 * @JTAG_STATE_PAUSEIR: JTAG state machine PAUSE_IR state
 * @JTAG_STATE_EXIT2IR: JTAG state machine EXIT-2 IR state
 * @JTAG_STATE_UPDATEIR: JTAG state machine UPDATE IR state
 * @JTAG_STATE_CURRENT: JTAG current state, saved by driver
 */
enum jtag_tapstate {
	JTAG_STATE_TLRESET,
	JTAG_STATE_IDLE,
	JTAG_STATE_SELECTDR,
	JTAG_STATE_CAPTUREDR,
	JTAG_STATE_SHIFTDR,
	JTAG_STATE_EXIT1DR,
	JTAG_STATE_PAUSEDR,
	JTAG_STATE_EXIT2DR,
	JTAG_STATE_UPDATEDR,
	JTAG_STATE_SELECTIR,
	JTAG_STATE_CAPTUREIR,
	JTAG_STATE_SHIFTIR,
	JTAG_STATE_EXIT1IR,
	JTAG_STATE_PAUSEIR,
	JTAG_STATE_EXIT2IR,
	JTAG_STATE_UPDATEIR,
	JTAG_STATE_CURRENT
};

/**
 * enum jtag_reset:
 *
 * @JTAG_NO_RESET: JTAG run TAP from current state
 * @JTAG_FORCE_RESET: JTAG force TAP to reset state
 */
enum jtag_reset {
	JTAG_NO_RESET = 0,
	JTAG_FORCE_RESET = 1,
};

/**
 * enum jtag_xfer_type:
 *
 * @JTAG_SIR_XFER: SIR transfer
 * @JTAG_SDR_XFER: SDR transfer
 */
enum jtag_xfer_type {
	JTAG_SIR_XFER = 0,
	JTAG_SDR_XFER = 1,
};

/**
 * enum jtag_xfer_direction:
 *
 * @JTAG_READ_XFER: read transfer
 * @JTAG_WRITE_XFER: write transfer
 * @JTAG_READ_WRITE_XFER: read & write transfer
 */
enum jtag_xfer_direction {
	JTAG_READ_XFER = 1,
	JTAG_WRITE_XFER = 2,
	JTAG_READ_WRITE_XFER = 3,
};

/**
 * struct jtag_tap_state - forces JTAG state machine to go into a TAPC
 * state
 *
 * @reset: 0 - run IDLE/PAUSE from current state
 *         1 - go through TEST_LOGIC/RESET state before  IDLE/PAUSE
 * @from: initital jtag state
 * @endstate: jtag end state
 * @tck: clock counter
 *
 * Structure provide interface to JTAG device for JTAG set state execution.
 */
struct jtag_tap_state {
	uint8_t	reset;
	uint8_t	from;
	uint8_t	endstate;
	uint32_t	tck;
};

/**
 * union pad_config - Padding Configuration:
 *
 * @type: transfer type
 * @pre_pad_number: Number of prepadding bits bit[11:0]
 * @post_pad_number: Number of prepadding bits bit[23:12]
 * @pad_data : Bit value to be used by pre and post padding bit[24]
 * @int_value: unsigned int packed padding configuration value bit[32:0]
 *
 * Structure provide pre and post padding configuration in a single uint32_t
 */
union pad_config {
	struct {
		uint32_t pre_pad_number	: 12;
		uint32_t post_pad_number	: 12;
		uint32_t pad_data		: 1;
		uint32_t rsvd		: 7;
	};
	uint32_t int_value;
};

/**
 * struct jtag_xfer - jtag xfer:
 *
 * @type: transfer type
 * @direction: xfer direction
 * @from: xfer current state
 * @endstate: xfer end state
 * @padding: xfer padding
 * @length: xfer bits length
 * @tdio : xfer data array
 *
 * Structure provide interface to JTAG device for JTAG SDR/SIR xfer execution.
 */
struct jtag_xfer {
	uint8_t	type;
	uint8_t	direction;
	uint8_t	from;
	uint8_t	endstate;
	uint32_t	padding;
	uint32_t	length;
	uint64_t	tdio;
};

/**
 * struct bitbang_packet - jtag bitbang array packet:
 *
 * @data:   JTAG Bitbang struct array pointer(input/output)
 * @length: array size (input)
 *
 * Structure provide interface to JTAG device for JTAG bitbang bundle execution
 */
struct bitbang_packet {
	struct tck_bitbang *data;
	uint32_t	length;
} __attribute__((__packed__));

/**
 * struct jtag_bitbang - jtag bitbang:
 *
 * @tms: JTAG TMS
 * @tdi: JTAG TDI (input)
 * @tdo: JTAG TDO (output)
 *
 * Structure provide interface to JTAG device for JTAG bitbang execution.
 */
struct tck_bitbang {
	uint8_t	tms;
	uint8_t	tdi;
	uint8_t	tdo;
} __attribute__((__packed__));

/**
 * struct jtag_mode - jtag mode:
 *
 * @feature: 0 - JTAG feature setting selector for JTAG controller HW/SW
 *           1 - JTAG feature setting selector for controller bus mode
 *               output (enable / disable).
 * @mode:    (0 - SW / 1 - HW) for JTAG_XFER_MODE feature(0)
 *           (0 - output disable / 1 - output enable) for JTAG_CONTROL_MODE
 *                                                    feature(1)
 *
 * Structure provide configuration modes to JTAG device.
 */
struct jtag_mode {
	uint32_t	feature;
	uint32_t	mode;
};

/* ioctl interface */
#define __JTAG_IOCTL_MAGIC	0xb2

#define JTAG_SIOCSTATE	_IOW(__JTAG_IOCTL_MAGIC, 0, struct jtag_tap_state)
#define JTAG_SIOCFREQ	_IOW(__JTAG_IOCTL_MAGIC, 1, unsigned int)
#define JTAG_GIOCFREQ	_IOR(__JTAG_IOCTL_MAGIC, 2, unsigned int)
#define JTAG_IOCXFER	_IOWR(__JTAG_IOCTL_MAGIC, 3, struct jtag_xfer)
#define JTAG_GIOCSTATUS _IOWR(__JTAG_IOCTL_MAGIC, 4, enum jtag_tapstate)
#define JTAG_SIOCMODE	_IOW(__JTAG_IOCTL_MAGIC, 5, unsigned int)
#define JTAG_IOCBITBANG	_IOW(__JTAG_IOCTL_MAGIC, 6, unsigned int)
#define JTAG_SIOCTRST	_IOW(__JTAG_IOCTL_MAGIC, 7, unsigned int)

/**
 * struct tms_cycle - This structure represents a tms cycle state.
 *
 * @tmsbits: is the bitwise representation of the needed tms transitions to
 *           move from one state to another.
 * @count:   number of jumps needed to move to the needed state.
 *
 */
struct tms_cycle {
	unsigned char tmsbits;
	unsigned char count;
};

#endif /* __UAPI_LINUX_JTAG_H */
