// SPDX-License-Identifier: GPL-2.0-only

/*
 * Driver for Xilinx Platform Cable USB (II)
 *
 * Inspired by UrJtag xpc driver implementation
 *
 * Copyright (C) 2024 Hansem Ro <hansemro@outlook.com>
 * Copyright (C) 2008 Kolja Waschk
 * Copyright (C) 2002, 2003 ETC s.r.o.
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>

#include <jtag/jtag.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_helper.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/log.h>
#include <helper/types.h>

/** Vendor ID */
#define XILINX_VID				0x03fd

/** Product ID for Platform Cable USB (II) with firmware loaded */
#define PLATFORM_CABLE_PID		0x0008

/** XPC base clock frequency */
#define XPC_BASE_FREQUENCY		24000000UL

/** Size of an XPC command frame in bytes */
#define XPC_FRAME_SIZE			sizeof(uint16_t)

/** Maximum number of operations per JTAG command frame */
#define XPC_MAX_OPS_PER_FRAME	4UL

/** Number of operations per block */
#define XPC_NUM_OPS_PER_BLOCK	0x10000UL

/** Maximum number of operations that can be queued and flushed */
#define XPC_MAX_PENDING_OPS		(256UL * XPC_NUM_OPS_PER_BLOCK)

/** Maximum number of pending TDO bits that can be queued and flushed */
#define XPC_MAX_PENDING_TDO_BITS	0x2000UL

/** Maximum command queue flush size in bytes */
#define XPC_BUF_SIZE			(XPC_MAX_PENDING_OPS / XPC_FRAME_SIZE)

/** Maximum bulk write transfer size */
#define XPC_BULK_WRITE_SIZE		0x4000UL

/** Maximum number of frames that can fit in the command queue */
#define XPC_MAX_CMD_FRAMES		(XPC_BUF_SIZE / XPC_FRAME_SIZE)

/** Assert TDI high */
#define XPC_TDI					BIT(0)
/** Assert TMS high */
#define XPC_TMS					BIT(4)
/** Generate rising-then-falling TCK pulse */
#define XPC_TCK					BIT(8)
/** Shift out TDO on rising edge of TCK */
#define XPC_TDO					BIT(12)

/** XPC command queue */
struct xpc_usb_cmd_buf {
	/* Command packet with JTAG operations sent to the adapter. */
	uint8_t *cmds;
	/* Current number of pending operations. */
	size_t num_pending_ops;
	/* Current number of TDO bits to shift out. */
	size_t num_pending_tdo_bits;
	/* Points to a memory-allocated array of 32-bit packed TDO bits after
	 * flushing queue with (XPC_TDO|XPC_TCK) operation(s) or NULL otherwise. */
	uint32_t *tdo_bits;
	/* Number of TDO bits shifted out. */
	size_t num_tdo_bits;
};

/** XPC adapter */
struct xpc_usb {
	unsigned int ep_in;
	unsigned int ep_out;
	struct libusb_device_handle *dev;

	struct xpc_usb_cmd_buf *cmd_buf;
};

/**************************** Function Prototypes *****************************/

/** USB helper functions */
static int xpc_usb_open(struct xpc_usb **device);
static int xpc_usb_close(struct xpc_usb **device);

/** XPC-specific functions */
static int xpc_usb_set_prescaler(struct xpc_usb *device, int value);
static int xpc_usb_output_enable(struct xpc_usb *device, int enable);
static int xpc_usb_write_gpio(struct xpc_usb *device, uint8_t bits);
static int xpc_usb_read_gpio(struct xpc_usb *device, uint8_t *bits);
static int xpc_usb_read_firmware_version(struct xpc_usb *device, uint16_t *version);
static int xpc_usb_read_pld_version(struct xpc_usb *device, uint16_t *version);
static int xpc_usb_jtag_transfer(struct xpc_usb *device, size_t num_ops,
		uint8_t *cmds, size_t num_tdo_bits, uint32_t *tdo_bits);

/** JTAG queue functions */
static int xpc_usb_queue_cmd(struct xpc_usb *device, uint16_t cmd);
static int xpc_usb_clear_queue(struct xpc_usb *device, bool clear_tdo_bits);
static int xpc_usb_flush_queue(struct xpc_usb *device);
static int xpc_usb_queue_statemove(struct xpc_usb *device, int skip);
static int xpc_usb_queue_pathmove(struct xpc_usb *device, struct jtag_command *cmd);
static int xpc_usb_queue_scan(struct xpc_usb *device, struct jtag_command *cmd);
static void xpc_usb_queue_reset(struct xpc_usb *device, struct jtag_command *cmd);
static int xpc_usb_queue_runtest(struct xpc_usb *device, struct jtag_command *cmd);
static int xpc_usb_queue_sleep(struct xpc_usb *device, struct jtag_command *cmd);
static int xpc_usb_queue_stableclocks(struct xpc_usb *device, struct jtag_command *cmd);
static int xpc_usb_queue_tms(struct xpc_usb *device, struct jtag_command *cmd);

/** adapter functions */
static int xpc_usb_queue_command(struct jtag_command *cmd);
static int xpc_usb_execute_queue(struct jtag_command *cmd_queue);
static int xpc_usb_init(void);
static int xpc_usb_quit(void);
static int xpc_usb_speed(int speed);
static int xpc_usb_speed_div(int speed, int *khz);
static int xpc_usb_khz(int khz, int *jtag_speed);

/****************************** Global variables ******************************/

static struct xpc_usb *xpc_usb_handle;

/**************************** USB helper functions ****************************/

/**
 * Opens and claims an adapter.
 *
 * @param device pointer to XPC handle
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_open(struct xpc_usb **device)
{
	const uint16_t vids[] = { XILINX_VID, 0 };
	const uint16_t pids[] = { PLATFORM_CABLE_PID, 0 };
	struct libusb_device_handle *dev;

	if (jtag_libusb_open(vids, pids, NULL, &dev, NULL) != ERROR_OK)
		return ERROR_FAIL;

	*device = calloc(1, sizeof(struct xpc_usb));
	(*device)->dev = dev;

	return ERROR_OK;
}

/**
 * Closes and releases the adapter.
 *
 * @param device pointer to XPC handle
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_close(struct xpc_usb **device)
{
	if (device && *device && (*device)->dev) {
		if (libusb_release_interface((*device)->dev, 0) != 0)
			return ERROR_FAIL;
		libusb_close((*device)->dev);
		(*device)->dev = NULL;
	}
	return ERROR_OK;
}

/*************************** XPC-specific functions ***************************/

/**
 * Configure maximum TCK frequency by adjusting prescaler value.
 *
 * Maximum TCK Frequency = 24000000 / (2^(value - 0xf)) Hz
 *
 * @param device XPC adapter handle
 * @param value prescaler value (0xf-0x14)
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_set_prescaler(struct xpc_usb *device, int value)
{
	if (value < 0xf || value > 0x14)
		return ERROR_FAIL;
	return jtag_libusb_control_transfer(device->dev, 0x40, 0xB0, 0x0028,
			value, NULL, 0, 1000, NULL);
}

static int xpc_usb_output_enable(struct xpc_usb *device, int enable)
{
	return jtag_libusb_control_transfer(device->dev, 0x40, 0xB0,
			enable ? 0x18 : 0x10, 0, NULL, 0, 1000, NULL);
}

static int xpc_usb_write_gpio(struct xpc_usb *device, uint8_t bits)
{
	return jtag_libusb_control_transfer(device->dev, 0x40, 0xB0, 0x0030,
			bits, NULL, 0, 1000, NULL);
}

static int xpc_usb_read_gpio(struct xpc_usb *device, uint8_t *bits)
{
	return jtag_libusb_control_transfer(device->dev, 0xC0, 0xB0, 0x0038,
			0, (char *)bits, 1, 1000, NULL);
}

static int xpc_usb_read_firmware_version(struct xpc_usb *device, uint16_t *version)
{
	uint8_t buf[2] = {0, 0};
	int err = jtag_libusb_control_transfer(device->dev, 0xC0, 0xB0, 0x0050,
			0x0000, (char *)buf, 2, 1000, NULL);
	if (err == ERROR_OK)
		*version = le_to_h_u16(buf);
	return err;
}

static int xpc_usb_read_pld_version(struct xpc_usb *device, uint16_t *version)
{
	uint8_t buf[2] = {0, 0};
	int err = jtag_libusb_control_transfer(device->dev, 0xC0, 0xB0, 0x0050,
				0x0001, (char *)buf, 2, 1000, NULL);
	if (err == ERROR_OK)
		*version = le_to_h_u16(buf);
	return err;
}

/**
 * Perform bulk JTAG operation(s) on XPC's external scan chain.
 *
 * A JTAG operation describes the following:
 * - Whether to assert TDI high/low in a TCK cycle.
 *   - Output set whether or not TCK is toggling.
 * - Whether to assert TMS high/low in a TCK cycle.
 *   - Output set whether or not TCK is toggling.
 * - Whether to toggle TCK high-then-low or remain low.
 *   - TCK period/frequency adjusted by xpc_usb_set_prescaler.
 * - Whether to read back a shifted TDO bit.
 *   - If TCK is not toggling, then no TDO bit is read back.
 *
 * Up to 0x1000000 operations can be executed in a single batch by initiating
 * a (0xA6) control transfer, then performing one or more bulk write transfers
 * with 16-bit-aligned commands, and then followed by a bulk read transfer if
 * TDO bits were shifted out and read. (If TDO bits need to be shifted and read
 * back to the host, then only up to 0x3000 of those operations can be TDO shift
 * operations.)
 *
 * Each 16-bit command can encode up-to 4 JTAG operations by reserving one bit
 * in each of the 4 (4-bit) nibbles for each operation. The operations within
 * the command are executed in the order of the lowest bit of each nibble.
 *
 * Little endian 16-bit command format:
 * - bits 15-12: TDO read bits
 * - bits 11-8: TCK pulse bits
 * - bits 7-4: TMS bits
 * - bits 3-0: TDI bits
 *
 * @param device XPC adapter handle
 * @param num_ops total number of JTAG operations (0x1-0x1000000)
 * @param cmds pointer to packet with JTAG operations (in little endian)
 * @param num_tdo_bits number of bits to shift out from TDO (0x0-0x2000)
 * @param tdo_bits pointer to store 32-bit packed TDO samples (in host endian)
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_jtag_transfer(struct xpc_usb *device, size_t num_ops, uint8_t *cmds,
		size_t num_tdo_bits, uint32_t *tdo_bits)
{
	size_t num_blocks;
	size_t left;
	size_t left_size;
	size_t rd_size;
	uint8_t *cmd_ptr;
	uint8_t *rd_buf;
	uint8_t *rd_ptr;
	int tdo_idx;
	int err;
	int actual;

	if (num_ops == 0 || num_ops > XPC_MAX_PENDING_OPS)
		return ERROR_FAIL;

	// up-to 0x3000 TDO bits can be shifted out per jtag transfer
	if (num_tdo_bits > XPC_MAX_PENDING_TDO_BITS)
		return ERROR_FAIL;

	// non-zero number of operations in last bulk transfer (0x1-0x10000)
	left = ((num_ops - 1) % XPC_NUM_OPS_PER_BLOCK) + 1;
	// remaining number of blocks with 0x10000 operations (0x0-0xFF)
	assert((num_ops - left) % XPC_NUM_OPS_PER_BLOCK == 0);
	num_blocks = (num_ops - left) / XPC_NUM_OPS_PER_BLOCK;
	assert(num_blocks < 0x100);
	left_size = DIV_ROUND_UP(left, XPC_MAX_OPS_PER_FRAME) * XPC_FRAME_SIZE;
	assert(left_size > 0);
	LOG_DEBUG("num_ops=0x%zx, num_blocks=0x%zx, left=0x%zx", num_ops, num_blocks, left);

	// MSB of wValue = number of blocks (0x00-0xFF)
	// LSB of wValue = 0xA6
	// wIndex = number of operations in last bulk transfer - 1 (0x0000-0xFFFF)
	// Total number of operations = (MSB of wValue)*(0x10000) + (wIndex + 1)
	if (jtag_libusb_control_transfer(device->dev, 0x40, 0xB0,
				(int)(0x00A6 | (num_blocks << 8)),
				(int)(left - 1),
				NULL, 0, 1000, NULL) != ERROR_OK) {
		LOG_ERROR("wValue=0x%zx, index=0x%zx",
				0x00A6 | (num_blocks << 8),
				left - 1);
		return ERROR_FAIL;
	}

	// transfer commands in 0x4000 byte chunks
	cmd_ptr = cmds;
	for (size_t i = 0; i < num_blocks * 2; i++) {
		err = jtag_libusb_bulk_write(device->dev, device->ep_out,
				(char *)cmd_ptr, XPC_BULK_WRITE_SIZE, 6000, &actual);
		if (err != ERROR_OK)
			return err;
		assert(actual == XPC_BULK_WRITE_SIZE);
		cmd_ptr += XPC_BULK_WRITE_SIZE;
	}

	// transfer remaining (<= 0x10000) ops
	err = jtag_libusb_bulk_write(device->dev, device->ep_out,
			(char *)cmd_ptr, (int)left_size, 6000, &actual);
	if (err != ERROR_OK)
		return err;

	if (num_tdo_bits > 0) {
		rd_size = DIV_ROUND_UP(num_tdo_bits, 32) * sizeof(uint32_t);

		rd_buf = malloc(rd_size);
		if (!rd_buf)
			return ERROR_FAIL;

		rd_ptr = rd_buf;
		err = jtag_libusb_bulk_read(device->dev, device->ep_in,
				(char *)rd_ptr, (int)rd_size, 6000, &actual);
		if (err != ERROR_OK) {
			free(rd_buf);
			return err;
		}

		// Process little endian u32/u16 TDO bits and write out to destination
		// in host endian u32
		rd_ptr = rd_buf;
		tdo_idx = 0;
		left = num_tdo_bits;
		// every set of 32 TDO samples are packed in u32
		while (left >= 32) {
			tdo_bits[tdo_idx++] = le_to_h_u32(rd_ptr);
			rd_ptr += sizeof(uint32_t);
			left -= 32;
		}
		// last set of TDO samples are shifted from bit 15 or 31
		if (left > 16)
			tdo_bits[tdo_idx++] = le_to_h_u32(rd_ptr) >> (32 - left);
		else if (left > 0)
			tdo_bits[tdo_idx++] = le_to_h_u16(rd_ptr) >> (16 - left);

		free(rd_buf);
	}

	return ERROR_OK;
}

/**************************** JTAG queue functions ****************************/

/**
 * Append XPC JTAG command to queue.
 *
 * @param device XPC adapter handle
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_cmd(struct xpc_usb *device, uint16_t cmd)
{
	uint8_t *cmd_ptr;
	int frame_idx;
	uint16_t value;
	int bit_offset;
	int err;

	if (!device || !device->cmd_buf)
		return ERROR_FAIL;

	if (device->cmd_buf->num_pending_ops >= XPC_MAX_PENDING_OPS) {
		// Flush if not shifting out of TDO
		if (device->cmd_buf->num_pending_tdo_bits > 0)
			return ERROR_FAIL;
		err = xpc_usb_flush_queue(device);
		if (err != ERROR_OK)
			return err;
	}

	assert(device->cmd_buf->num_pending_tdo_bits <= device->cmd_buf->num_pending_ops);
	if (device->cmd_buf->num_pending_tdo_bits >= XPC_MAX_PENDING_TDO_BITS) {
		if ((cmd & XPC_TCK) && (cmd & XPC_TDO)) {
			LOG_ERROR("Cannot queue any more TDO shift-out operations");
			return ERROR_FAIL;
		}
	}

	frame_idx = device->cmd_buf->num_pending_ops / XPC_MAX_OPS_PER_FRAME;
	cmd_ptr = device->cmd_buf->cmds + (frame_idx * XPC_FRAME_SIZE);
	bit_offset = device->cmd_buf->num_pending_ops % XPC_MAX_OPS_PER_FRAME;
	value = le_to_h_u16(cmd_ptr);
	value |= (cmd & 0x1111) << bit_offset;
	h_u16_to_le(cmd_ptr, value);

	device->cmd_buf->num_pending_ops++;
	if ((cmd & XPC_TCK) && (cmd & XPC_TDO))
		device->cmd_buf->num_pending_tdo_bits++;

	return ERROR_OK;
}

/**
 * Clear XPC command queue.
 *
 * @param device XPC adapter handle
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_clear_queue(struct xpc_usb *device, bool clear_tdo_bits)
{
	device->cmd_buf->num_pending_ops = 0;
	device->cmd_buf->num_pending_tdo_bits = 0;
	memset(device->cmd_buf->cmds, 0x00, XPC_BUF_SIZE * sizeof(uint8_t));
	if (clear_tdo_bits) {
		if (device->cmd_buf->tdo_bits) {
			free(device->cmd_buf->tdo_bits);
			device->cmd_buf->tdo_bits = NULL;
		}
		device->cmd_buf->num_tdo_bits = 0;
	}
	return ERROR_OK;
}

/**
 * Execute and flush XPC command queue.
 *
 * A buffer for TDO bits that were shifted out will be allocated but should
 * eventually be cleared by caller with xpc_usb_clear_queue(device, true).
 *
 * @param device XPC adapter handle
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_flush_queue(struct xpc_usb *device)
{
	size_t out_len = 0;
	int err;

	if (!device || !device->cmd_buf)
		return ERROR_FAIL;

	if (device->cmd_buf->num_pending_ops == 0)
		return ERROR_OK;

	assert(device->cmd_buf->num_pending_ops <= XPC_MAX_PENDING_OPS);
	assert(device->cmd_buf->num_pending_tdo_bits <= XPC_MAX_PENDING_TDO_BITS);

	if (device->cmd_buf->num_pending_tdo_bits > 0) {
		if (device->cmd_buf->tdo_bits) {
			LOG_DEBUG_IO("Discarding/freeing previous tdo_bits before transfer");
			free(device->cmd_buf->tdo_bits);
			device->cmd_buf->tdo_bits = NULL;
		}
		out_len = DIV_ROUND_UP(device->cmd_buf->num_pending_tdo_bits, 32) * sizeof(uint32_t);
		device->cmd_buf->tdo_bits = calloc(out_len, sizeof(uint32_t));
		device->cmd_buf->num_tdo_bits = 0;
		if (!device->cmd_buf->tdo_bits) {
			LOG_DEBUG("Failed to allocate %zu bytes for %zu bits", out_len,
					device->cmd_buf->num_pending_tdo_bits);
			return ERROR_FAIL;
		}
	}

	err = xpc_usb_jtag_transfer(device, device->cmd_buf->num_pending_ops,
			device->cmd_buf->cmds, device->cmd_buf->num_pending_tdo_bits,
			device->cmd_buf->tdo_bits);

	if (err == ERROR_OK) {
		device->cmd_buf->num_tdo_bits = device->cmd_buf->num_pending_tdo_bits;
		xpc_usb_clear_queue(device, false);
	}

	return err;
}

/**
 * Queue a sequence of TMS operations.
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be queued
 * @param skip number of starting TMS operations to skip
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_statemove(struct xpc_usb *device, int skip)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
			tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
			tap_get_end_state());
	int err;

	assert(skip >= 0);

	LOG_DEBUG("queueing statemove starting at (skip: %d) %s end in %s", skip,
			tap_state_name(tap_get_state()),
			tap_state_name(tap_get_end_state()));

	for (int i = skip; i < tms_count; i++) {
		err = xpc_usb_queue_cmd(device, XPC_TCK | (tms_scan & (1 << i) ? XPC_TMS : 0x00));
		if (err != ERROR_OK)
			return err;
	}

	tap_set_state(tap_get_end_state());

	return ERROR_OK;
}

/**
 * Queue a sequence of TMS operations.
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be queued
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_pathmove(struct xpc_usb *device, struct jtag_command *cmd)
{
	unsigned int num_states;
	tap_state_t *path;
	int err;

	num_states = cmd->cmd.pathmove->num_states;
	path = cmd->cmd.pathmove->path;

	for (unsigned int i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false)) {
			err = xpc_usb_queue_cmd(device, XPC_TCK);
		} else if (path[i] == tap_state_transition(tap_get_state(), true)) {
			err = xpc_usb_queue_cmd(device, XPC_TCK | XPC_TMS);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()),
				  tap_state_name(path[i]));
			err = ERROR_JTAG_QUEUE_FAILED;
		}
		if (err != ERROR_OK)
			return err;
		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());

	return ERROR_OK;
}

/**
 * Shift in/out bits while in Shift-DR or Shift-IR state and queue operation to
 * restore previous end state (if necessary).
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be performed
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_scan(struct xpc_usb *device, struct jtag_command *cmd)
{
	enum scan_type type = jtag_scan_type(cmd->cmd.scan);
	tap_state_t saved_end_state = cmd->cmd.scan->end_state;
	bool ir_scan = cmd->cmd.scan->ir_scan;
	uint32_t tdi, tms;
	uint8_t *buf, *rd_ptr, *conv_ptr;
	int err, scan_size;
	size_t write, left, conv;

	if (xpc_usb_handle->cmd_buf->num_pending_tdo_bits > 0) {
		LOG_ERROR("Must not have pending TDO transfers (%zu)",
				xpc_usb_handle->cmd_buf->num_pending_tdo_bits);
		return ERROR_FAIL;
	}

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buf);
	LOG_DEBUG("%s scan type %d %d bits; starts in %s end in %s",
		  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", type, scan_size,
		  tap_state_name(tap_get_state()),
		  tap_state_name(cmd->cmd.scan->end_state));

	if (ir_scan && tap_get_state() != TAP_IRSHIFT) {
		tap_set_end_state(TAP_IRSHIFT);
		err = xpc_usb_queue_statemove(device, 0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	} else if (!ir_scan && tap_get_state() != TAP_DRSHIFT) {
		tap_set_end_state(TAP_DRSHIFT);
		err = xpc_usb_queue_statemove(device, 0);
		if (err != ERROR_OK)
			goto out_err;
		tap_set_end_state(saved_end_state);
	}
	err = xpc_usb_flush_queue(device);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to flush queue before shifting in/out bits: %d", err);
		goto out_err;
	}

	assert(!device->cmd_buf->tdo_bits);
	assert(device->cmd_buf->num_pending_tdo_bits == 0);
	assert(device->cmd_buf->num_tdo_bits == 0);

	// Shift in/out data with TMS asserted for last bit
	rd_ptr = buf;
	conv_ptr = buf;
	left = scan_size;
	while (left > 0) {
		write = MIN(32, left);
		tdi = (type != SCAN_IN) ? buf_get_u32(rd_ptr, 0, write) : 0;
		tms = left <= 32 ? BIT(write - 1) : 0;
		for (size_t i = 0; i < write; i++) {
			xpc_usb_queue_cmd(device, XPC_TCK |
					(type != SCAN_OUT ? XPC_TDO : 0) |
					(tms & BIT(i) ? XPC_TMS : 0) |
					(tdi & BIT(i) ? XPC_TDI : 0));
		}
		left -= write;
		rd_ptr += sizeof(uint32_t);
		// flush queue if one of the following thresholds are met:
		// - at maximum pending TDO shift-out operation threshold
		// - at maximum pending JTAG operation threshold
		// - queued last shift operations (below above thresholds)
		if (device->cmd_buf->num_pending_tdo_bits >= XPC_MAX_PENDING_TDO_BITS ||
				device->cmd_buf->num_pending_ops >= XPC_MAX_PENDING_OPS ||
				left == 0) {
			LOG_DEBUG("Flushing 0x%zx ops with 0x%zx remaining",
					device->cmd_buf->num_pending_ops, left);
			err = xpc_usb_flush_queue(device);
			if (err != ERROR_OK)
				goto out_err;
			if (type != SCAN_OUT) {
				for (size_t i = 0; conv_ptr < rd_ptr; i++) {
					conv = MIN(32, device->cmd_buf->num_tdo_bits);
					buf_set_u32(conv_ptr, 0, conv,
							device->cmd_buf->tdo_bits[i]);
					device->cmd_buf->num_tdo_bits -= conv;
					conv_ptr += sizeof(uint32_t);
				}
				free(device->cmd_buf->tdo_bits);
				device->cmd_buf->tdo_bits = NULL;
				device->cmd_buf->num_tdo_bits = 0;
			}
		}
	}

	err = jtag_read_buffer(buf, cmd->cmd.scan);
	free(buf);

	if (tap_get_state() != tap_get_end_state())
		err = xpc_usb_queue_statemove(device, 1);

	return err;

out_err:
	free(buf);
	return ERROR_FAIL;
}

static void xpc_usb_queue_reset(struct xpc_usb *device, struct jtag_command *cmd)
{
	LOG_DEBUG("reset trst: %i srst: %i", cmd->cmd.reset->trst,
			cmd->cmd.reset->srst);
}

/**
 * Queue operations to generate TCK cycles in Run-Test/Idle state.
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be queued
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_runtest(struct xpc_usb *device, struct jtag_command *cmd)
{
	tap_state_t saved_end_state;
	size_t num_cycles;
	int err = ERROR_OK;

	LOG_DEBUG("runtest %u cycles, end in %i",
		  cmd->cmd.runtest->num_cycles,
		  cmd->cmd.runtest->end_state);

	saved_end_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		tap_set_end_state(TAP_IDLE);
		err = xpc_usb_queue_statemove(device, 0);
		if (err != ERROR_OK)
			return err;
	};

	num_cycles = cmd->cmd.runtest->num_cycles;

	for (size_t i = 0; i < num_cycles; i++) {
		err = xpc_usb_queue_cmd(device, XPC_TCK);
		if (err != ERROR_OK)
			return err;
	}

	tap_set_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		err = xpc_usb_queue_statemove(device, 0);

	return err;
}

/**
 * Sleep for a specific amount of time (by flushing the queue then sleeping).
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be executed
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_sleep(struct xpc_usb *device, struct jtag_command *cmd)
{
	int err;

	err = xpc_usb_flush_queue(device);
	if (err != ERROR_OK)
		return ERROR_FAIL;

	LOG_DEBUG("sleep %" PRIu32 "", cmd->cmd.sleep->us);
	usleep(cmd->cmd.sleep->us);
	return ERROR_OK;
}

/**
 * Queue operations to generate TCK cycles while remaining in a stable state.
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be queued
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_stableclocks(struct xpc_usb *device, struct jtag_command *cmd)
{
	unsigned int num_cycles;
	int xpc_usb_cmd;
	int err;

	if (!tap_is_state_stable(tap_get_state())) {
		LOG_ERROR("Not in a stable state (%s)", tap_state_name(tap_get_state()));
		return ERROR_FAIL;
	}

	num_cycles = cmd->cmd.stableclocks->num_cycles;
	xpc_usb_cmd = XPC_TCK | (tap_get_state() == TAP_RESET ? XPC_TMS : 0);

	while (num_cycles > 0) {
		err = xpc_usb_queue_cmd(device, xpc_usb_cmd);
		if (err != ERROR_OK)
			return err;
		num_cycles--;
	}

	return ERROR_OK;
}

/**
 * Queue a sequence of TMS operations.
 *
 * @param device XPC adapter handle
 * @param cmd pointer to the command that shall be queued
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_tms(struct xpc_usb *device, struct jtag_command *cmd)
{
	const size_t num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	size_t left, write;
	uint32_t tms;
	int err;

	LOG_DEBUG("execute tms %zu", num_bits);

	left = num_bits;
	while (left) {
		write = MIN(32, left);
		tms = buf_get_u32(bits, 0, write);
		for (size_t i = 0; i < write; i++) {
			err = xpc_usb_queue_cmd(device, XPC_TCK |
					(tms & BIT(i) ? XPC_TMS : 0));
		}
		if (err != ERROR_OK)
			return err;
		left -= write;
		bits += sizeof(uint32_t);
	};

	return ERROR_OK;
}

/****************************** adapter functions *****************************/

/**
 * Enqueues a single JTAG command.
 *
 * @param cmd JTAG command to add to queue
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_queue_command(struct jtag_command *cmd)
{
	LOG_DEBUG("cmd->type: %u", cmd->type);
	switch (cmd->type) {
	case JTAG_STABLECLOCKS:
		return xpc_usb_queue_stableclocks(xpc_usb_handle, cmd);
	case JTAG_RUNTEST:
		return xpc_usb_queue_runtest(xpc_usb_handle, cmd);
	case JTAG_TLR_RESET:
		tap_set_end_state(cmd->cmd.statemove->end_state);
		return xpc_usb_queue_statemove(xpc_usb_handle, 0);
	case JTAG_PATHMOVE:
		return xpc_usb_queue_pathmove(xpc_usb_handle, cmd);
	case JTAG_SCAN:
		return xpc_usb_queue_scan(xpc_usb_handle, cmd);
	case JTAG_RESET:
		xpc_usb_queue_reset(xpc_usb_handle, cmd);
		break;
	case JTAG_SLEEP:
		return xpc_usb_queue_sleep(xpc_usb_handle, cmd);
	case JTAG_TMS:
		return xpc_usb_queue_tms(xpc_usb_handle, cmd);
	default:
		LOG_ERROR("BUG: Unknown JTAG command type encountered.");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return ERROR_OK;
}

/**
 * Executes the JTAG command queue.
 *
 * @param cmd_queue A queue of JTAG commands
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue;
	int err;

	while (cmd) {
		err = xpc_usb_queue_command(cmd);
		if (err != ERROR_OK)
			return err;
		cmd = cmd->next;
	}

	err = xpc_usb_flush_queue(xpc_usb_handle);

	return err;
}

/**
 * Connects to and initializes XPC adapter.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_init(void)
{
	uint8_t no_op[2] = {0x00, 0x00};
	uint16_t version;
	uint8_t gpio_buf;
	int err;

	err = xpc_usb_open(&xpc_usb_handle);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to open XPC device");
		goto out_err;
	}

	err = jtag_libusb_choose_interface(xpc_usb_handle->dev,
			&xpc_usb_handle->ep_in,
			&xpc_usb_handle->ep_out, -1, -1, -1, -1);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_set_prescaler(xpc_usb_handle, 0x11);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_write_gpio(xpc_usb_handle, 8);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_read_firmware_version(xpc_usb_handle, &version);
	if (err != ERROR_OK)
		goto out_err;
	LOG_INFO("XPC-USB firmware version %" PRIu16, version);

	err = xpc_usb_read_pld_version(xpc_usb_handle, &version);
	if (err != ERROR_OK)
		goto out_err;
	LOG_INFO("XPC-USB PLD version %" PRIu16, version);

	// check if vref is connected
	err = xpc_usb_read_gpio(xpc_usb_handle, &gpio_buf);
	if (err != ERROR_OK)
		goto out_err;
	if (gpio_buf == 0)
		LOG_WARNING("VREF not detected; is target connected? (0x%x)", gpio_buf);

	err = xpc_usb_output_enable(xpc_usb_handle, 0);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_set_prescaler(xpc_usb_handle, 0x11);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_output_enable(xpc_usb_handle, 1);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_jtag_transfer(xpc_usb_handle, 2, no_op, 0, NULL);
	if (err != ERROR_OK)
		goto out_err;

	err = xpc_usb_set_prescaler(xpc_usb_handle, 0x12);
	if (err != ERROR_OK)
		goto out_err;

	xpc_usb_handle->cmd_buf = calloc(1, sizeof(struct xpc_usb_cmd_buf));
	xpc_usb_handle->cmd_buf->cmds = calloc(XPC_BUF_SIZE, sizeof(uint8_t));

	return ERROR_OK;

out_err:
	xpc_usb_quit();
	return err;
}

/**
 * Closes the USB handle for the XPC adapter.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_quit(void)
{
	int err;
	if (xpc_usb_handle) {
		err = xpc_usb_close(&xpc_usb_handle);
		if (xpc_usb_handle->cmd_buf) {
			if (xpc_usb_handle->cmd_buf->cmds) {
				free(xpc_usb_handle->cmd_buf->cmds);
				xpc_usb_handle->cmd_buf->cmds = NULL;
			}
			free(xpc_usb_handle->cmd_buf);
			xpc_usb_handle->cmd_buf = NULL;
		}
		free(xpc_usb_handle);
		xpc_usb_handle = NULL;
		return err;
	}
	return ERROR_OK;
}

/**
 * Set the TCK frequency of the XPC adapter.
 *
 * @param speed desired TCK frequency in Hz
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_speed(int speed)
{
	int prescaler;

	if (speed == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	if (speed >= 24000000 /* Hz*/)
		prescaler = 0xf;
	else if (speed >= 12000000 /* Hz */)
		prescaler = 0x10;
	else if (speed >= 6000000 /* Hz */)
		prescaler = 0x11;
	else if (speed >= 3000000 /* Hz */)
		prescaler = 0x12;
	else if (speed >= 1500000 /* Hz */)
		prescaler = 0x13;
	else
		prescaler = 0x14;

	LOG_INFO("Desired frequency = %d Hz; Actual frequency = %ld Hz",
			speed, XPC_BASE_FREQUENCY / (1 << (prescaler - 0xf)));
	xpc_usb_set_prescaler(xpc_usb_handle, prescaler);
	return ERROR_OK;
}

/**
 * Convert speed value to corresponding TCK frequency in kHz.
 *
 * @param speed desired TCK frequency in Hz
 * @param khz where to store corresponding TCK frequency in kHz
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_speed_div(int speed, int *khz)
{
	if (speed == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}
	*khz = speed / 1000;
	return ERROR_OK;
}

/**
 * Set the TCK frequency of the XPC adapter.
 *
 * @param khz desired TCK frequency in kHz
 * @param jtag_speed where to store corresponding adapter-specific speed value
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int xpc_usb_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static struct jtag_interface xpc_usb_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = xpc_usb_execute_queue,
};

struct adapter_driver xpc_usb_adapter_driver = {
	.name = "xpc_usb",
	.transports = jtag_only,

	.init = xpc_usb_init,
	.quit = xpc_usb_quit,
	.speed = xpc_usb_speed,
	.speed_div = xpc_usb_speed_div,
	.khz = xpc_usb_khz,

	.jtag_ops = &xpc_usb_interface,
};
