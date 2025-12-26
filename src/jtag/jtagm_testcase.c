/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/dirent.h>
#include <zephyr/shell/shell.h>
#include <linux/ioctl.h>
#include <jtag/jtagm_fs.h>

static int jtag_shift_ir(int fd, uint32_t ir_value, int ir_len)
{
    struct jtag_xfer xfer = {0};

    xfer.type = JTAG_SIR_XFER;
    xfer.length = ir_len;
    xfer.direction = JTAG_WRITE_XFER;
    xfer.tdio = (uintptr_t)&ir_value;
    xfer.endstate = JTAG_STATE_IDLE;

    return ioctl(fd, JTAG_IOCXFER, &xfer);
}

static int jtag_shift_dr_read32(int fd, uint32_t *out)
{
    uint32_t dummy = 0;
    struct jtag_xfer xfer = {0};

    xfer.type = JTAG_SDR_XFER;
    xfer.length = 32;
    xfer.direction = JTAG_READ_XFER;
    xfer.tdio = (uintptr_t)&dummy;
    xfer.endstate = JTAG_STATE_IDLE;

    int ret = ioctl(fd, JTAG_IOCXFER, &xfer);
    if (ret < 0)
        return ret;

    *out = dummy;
    return 0;
}

static int jtag_set_tap_state(int fd, enum jtag_tapstate tap_state)
{
    struct jtag_tap_state state = {
        .endstate = tap_state,
        .from = JTAG_STATE_IDLE,
        .reset = JTAG_NO_RESET,
        .tck = 0
    };

    int ret = ioctl(fd, JTAG_SIOCSTATE, &state);
    if (ret < 0)
        return ret;

    return 0;
}

static enum jtag_tapstate jtag_get_tap_state(int fd)
{
    enum jtag_tapstate tap_state;

    int ret = ioctl(fd, JTAG_GIOCSTATUS, &tap_state);
    if (ret < 0)
        return ret;

    return tap_state;
}

static int jtag_set_freq(int fd, uint32_t freq)
{
    int ret = ioctl(fd, JTAG_SIOCFREQ, &freq);
    if (ret < 0)
        return ret;

    return 0;
}

static int jtag_get_freq(int fd)
{
    uint32_t freq;
    int ret = ioctl(fd, JTAG_GIOCFREQ, &freq);
    if (ret < 0)
        return ret;

    return freq;
}

static int jtag_fs_test(void)
{
    enum jtag_tapstate state;
    uint32_t id_code = 0;
    int fd = open("/dev/jtag1", O_RDWR);

    if (fd > 0)
    {
        printk("cur freq: %d\n", jtag_get_freq(fd));
        printk("set freq: %d\n", 5000000);
        jtag_set_freq(fd, 5000000);
        printk("get freq: %d\n", jtag_get_freq(fd));

        jtag_set_tap_state(fd, JTAG_STATE_IDLE);
        state = jtag_get_tap_state(fd);

        jtag_shift_ir(fd, 1, 5);

        jtag_set_tap_state(fd, JTAG_STATE_SHIFTDR);
        jtag_shift_dr_read32(fd, &id_code);
        printk("id_code: 0x%x\n", id_code);

        close(fd);
    }

    return 0;
}
SHELL_CMD_REGISTER(jtag_fs_test, NULL, "jtag_fs_test", jtag_fs_test);
 