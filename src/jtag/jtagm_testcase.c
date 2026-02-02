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
#include <stdlib.h>
#include <stdio.h>
#include "ls_soc_gpio.h"

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

static int jtag_shift_dr_read32(int fd, uint64_t *out)
{
    uint64_t dummy = 0xdeadbeefbad4f00d;
    struct jtag_xfer xfer = {0};

    xfer.type = JTAG_SDR_XFER;
    xfer.length = 64;
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

static int jtag_wait_cycle(int fd, uint32_t cycles)
{
    struct tck_bitbang bitbang[cycles];
    memset(bitbang, 0, sizeof (bitbang));

    struct bitbang_packet packet = {
        .length = cycles,
        .data = bitbang
    };

    for (int i = 0; i < cycles; i++)
    {
        bitbang[i].tdi = i % 2;
    }

    return ioctl(fd, JTAG_IOCBITBANG, &packet);
}

static int jtag_bitbang(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2)
    {
        printf("Used:\n");
        printf("    %s [num_bit] \n", argv[0]);
        return 0;
    }

    uint32_t num_bit = strtoul(argv[1], NULL, 0);
    printf("num_bit = %d\n", num_bit);
    per_func0_set(PD13, 28);

    int fd = open("/dev/jtag1", O_RDWR);

    jtag_wait_cycle(fd, num_bit);

    close(fd);

    return 0;
}
SHELL_CMD_REGISTER(jtag_bitbang, NULL, "jtag_bitbang", jtag_bitbang);

static int jtag_ir_padding(const struct shell *sh, size_t argc, char **argv)
{
    struct jtag_xfer xfer = {0};
    uint32_t tdio = 0xaa;
    per_func0_set(PD13, 28);

    int fd = open("/dev/jtag1", O_RDWR);

    union pad_config padding;
    padding.pre_pad_number = 5;
    padding.post_pad_number = 5;
    padding.pad_data = 1;

    xfer.type = JTAG_SIR_XFER;
    xfer.length = 5;
    xfer.direction = JTAG_WRITE_XFER;
    xfer.tdio = (uintptr_t)&tdio;
    xfer.endstate = JTAG_STATE_IDLE;
    xfer.padding = padding.int_value;

    ioctl(fd, JTAG_IOCXFER, &xfer);
    close(fd);

    return 0;
}
SHELL_CMD_REGISTER(jtag_ir_padding, NULL, "jtag_ir_padding", jtag_ir_padding);

static int jtag_dr_padding(const struct shell *sh, size_t argc, char **argv)
{
    struct jtag_xfer xfer = {0};
    uint32_t tdio = 0xaa;

    per_func0_set(PD13, 28);

    int fd = open("/dev/jtag1", O_RDWR);

    union pad_config padding;

    padding.pre_pad_number = 5;
    padding.post_pad_number = 5;
    padding.pad_data = 1;

    xfer.type = JTAG_SDR_XFER;
    xfer.length = 5;
    xfer.direction = JTAG_WRITE_XFER;
    xfer.tdio = (uintptr_t)&tdio;
    xfer.endstate = JTAG_STATE_IDLE;
    xfer.padding = padding.int_value;

    ioctl(fd, JTAG_IOCXFER, &xfer);
    close(fd);

    return 0;
}
SHELL_CMD_REGISTER(jtag_dr_padding, NULL, "jtag_dr_padding", jtag_dr_padding);

static int jtag_fs_read_idcode(const struct shell *sh, size_t argc, char **argv)
{
    enum jtag_tapstate state;
    uint64_t id_code = 0;

    if (argc < 3)
    {
        printf("Used:\n");
        printf("    %s [num_bit] [command]\n", argv[0]);
        return 0;
    }

    uint32_t num_bit = strtoul(argv[1], NULL, 0);
    uint32_t command = strtoul(argv[2], NULL, 16);
    printf("num_bit = %d, command = %x\n", num_bit, command);

    per_func0_set(PD13, 28);

    int fd = open("/dev/jtag1", O_RDWR);
    if (fd > 0)
    {
        printf("cur freq: %d\n", jtag_get_freq(fd));
        printf("set freq: %d\n", 15000000);
        jtag_set_freq(fd, 15000000);
        printf("get freq: %d\n", jtag_get_freq(fd));

        jtag_set_tap_state(fd, JTAG_STATE_IDLE);
        state = jtag_get_tap_state(fd);
        jtag_shift_ir(fd, command, num_bit);

        jtag_set_tap_state(fd, JTAG_STATE_SHIFTDR);
        jtag_shift_dr_read32(fd, &id_code);
        printf("id_code: 0x%x 0x%x\n", ((uint32_t *)&id_code)[0], ((uint32_t *)&id_code)[1]);

        close(fd);
    }

    return 0;
}
SHELL_CMD_REGISTER(jtag_fs_read_idcode, NULL, "jtag_fs_read_idcode", jtag_fs_read_idcode);
 