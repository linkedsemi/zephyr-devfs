/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/drivers/jtag.h>
#include <zephyr/fs/devfs.h>
#include <linux/ioctl.h>
#include <jtag/jtagm_fs.h>
#include "ls_soc_gpio.h"

#define DT_DRV_COMPAT linkedsemi_ls_jtag

LOG_MODULE_REGISTER(jtagm_dev, LOG_LEVEL_INF);
#define JTAGM_NUM       DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

struct jtagm_device
{
    const struct device *dev;
    const char *label;
    atomic_t is_open;
};

#define JTAGM_DEVICE_PROCESS(inst)                            \
    {                                                         \
        .dev = DEVICE_DT_GET(DT_INST(inst, DT_DRV_COMPAT)),   \
        .label = DT_INST_PROP(inst, label),                   \
        .is_open = ATOMIC_INIT(0)                             \
    },

static struct jtagm_device jtag_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(JTAGM_DEVICE_PROCESS)
};

static int jtagm_open(struct fs_file_t *zfp, const char *name, fs_mode_t flags)
{
    struct jtagm_device *jtag = NULL;

    for (int i = 0; i < JTAGM_NUM; i++)
    {
        if (strcmp(jtag_devices[i].label, name) == 0)
        {
            jtag = &jtag_devices[i];
            break;
        }
    }


    if (!jtag || !device_is_ready(jtag->dev))
    {
        LOG_ERR("Device %s not ready", name);
        return -ENODEV;
    }

    if (!atomic_cas(&jtag->is_open, 0, 1))
    {
        LOG_ERR("Device %s already opened", name);
        return -EBUSY;
    }

    if(jtag_tap_set(jtag->dev, TAP_RESET))
    {
        LOG_ERR("Device %s reset fail", name);
        return -EBUSY; 
    }

    if (strcmp(name, "jtag1") == 0)
    {
        /* workround */
        per_func0_set(PD13, 28);
    }

    zfp->filep = jtag;

    return 0;
}

static int jtagm_close(struct fs_file_t *zfp)
{
    struct jtagm_device *jtag = zfp->filep;
    if (jtag)
    {
        atomic_set(&jtag->is_open, 0);
        jtag_tap_set(jtag->dev, TAP_RESET);
    }
    return 0;
}

static enum tap_state usr_state_2_kernel_state[JTAG_STATE_CURRENT] = {
    TAP_RESET,TAP_IDLE,
    TAP_DRSELECT,TAP_DRCAPTURE,TAP_DRSHIFT,TAP_DREXIT1,TAP_DRPAUSE,TAP_DREXIT2,TAP_DRUPDATE,
    TAP_IRSELECT,TAP_IRCAPTURE,TAP_IRSHIFT,TAP_IREXIT1,TAP_IRPAUSE,TAP_IREXIT2,TAP_IRUPDATE
};

static enum jtag_tapstate kernel_state_2_usr_state[TAP_RESET+1] = {
    JTAG_STATE_EXIT2DR,JTAG_STATE_EXIT1DR,JTAG_STATE_SHIFTDR,JTAG_STATE_PAUSEDR,
    JTAG_STATE_SELECTIR,JTAG_STATE_UPDATEDR,JTAG_STATE_CAPTUREDR,JTAG_STATE_SELECTDR,
    JTAG_STATE_EXIT2IR,JTAG_STATE_EXIT1IR,JTAG_STATE_SHIFTIR,JTAG_STATE_PAUSEIR,
    JTAG_STATE_IDLE,JTAG_STATE_UPDATEIR,JTAG_STATE_CAPTUREIR,JTAG_STATE_TLRESET
};

static int jtagm_ioctl(struct fs_file_t *zfp, unsigned long cmd, va_list args)
{
    int ret = 0;
    struct jtagm_device *jtag = zfp->filep;
    uint32_t *freq;

    switch (cmd)
    {
    case JTAG_SIOCSTATE:
        struct jtag_tap_state *tapstate = va_arg(args, struct jtag_tap_state *);
        ret = jtag_tap_set(jtag->dev, usr_state_2_kernel_state[tapstate->endstate]);
        break;
    case JTAG_IOCXFER:
        struct jtag_xfer *xfer = va_arg(args, struct jtag_xfer *);
        if (xfer->type == JTAG_SIR_XFER)
        {
            ret = jtag_ir_scan(jtag->dev, xfer->length, (const uint8_t *)(uintptr_t)xfer->tdio, NULL, TAP_IDLE);
        }
        else
        {
            ret = jtag_dr_scan(jtag->dev, xfer->length, NULL, (uint8_t *)(uintptr_t)xfer->tdio, TAP_IDLE);
        }
        break;
    case JTAG_GIOCSTATUS:
        enum jtag_tapstate *state = va_arg(args, enum jtag_tapstate *);
        ret = jtag_tap_get(jtag->dev, (enum tap_state *)state);
        *state = kernel_state_2_usr_state[*(enum tap_state *)state];
        break;
    case JTAG_SIOCMODE:
        {
            struct jtag_mode *mode = va_arg(args, struct jtag_mode *);
            switch (mode->feature) 
            {
            case JTAG_XFER_MODE:
                break;
            case JTAG_CONTROL_MODE:
                break;
            default:
                return -EINVAL;
            }
        }
        break;
    case JTAG_SIOCFREQ:
        freq = va_arg(args, uint32_t *);
        ret = jtag_freq_set(jtag->dev, *freq);
        break;
    case JTAG_GIOCFREQ:
        freq = va_arg(args, uint32_t *);
        ret = jtag_freq_get(jtag->dev, freq);
        break;
    case JTAG_IOCBITBANG:
        break;
    case JTAG_SIOCTRST:
        break;
    case ZFD_IOCTL_SET_LOCK:
        // caller is open, ignore it
        ret = 0;
        break;
    default:
        return -EINVAL;
    }

    return ret;
}

static const struct fs_file_system_t jtagm_fs = {
    .open = jtagm_open,
    .close = jtagm_close,
    .ioctl = jtagm_ioctl,
};

int jtagm_fs_init(void)
{
    char name[32];

    for (int i = 0; i < JTAGM_NUM; i++)
    {
        snprintf(name, sizeof(name) - 1, "/dev/%s", jtag_devices[i].label);
        devfs_register(name, &jtagm_fs);
    }

    return 0;
}

SYS_INIT(jtagm_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
