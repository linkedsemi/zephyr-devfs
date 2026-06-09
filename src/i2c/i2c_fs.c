/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/smbus.h>
#include <zephyr/fs/devfs.h>

#ifdef CONFIG_USERSPACE
#include <zephyr/internal/syscall_handler.h>
#endif

#include "i2c/i2c-dev.h"
#include "i2c/i2c_fs.h"
#include "i2c/i2c.h"

#define DT_DRV_COMPAT linkedsemi_ls_i2c

LOG_MODULE_REGISTER(i2c_fs, CONFIG_I2C_DEV_LOG_LEVEL);

#define I2C_NUM DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT)

#define MAX_I2C_DEVICES 16 /* The maximum number of I2C devices */
#define I2C_MNT_POINT "/dev"

BUILD_ASSERT(I2C_NUM <= MAX_I2C_DEVICES,
             "Exceeded maximum I2C device count");


#define I2C_DEVICE_PROCESS(inst)                            \
    {                                                       \
        .dev = DEVICE_DT_GET(DT_INST(inst, DT_DRV_COMPAT)), \
        .label = DT_INST_PROP(inst, label),                 \
        .addr = 0,                                          \
        .is_open = ATOMIC_INIT(0),                          \
        .flags = 0},

static struct i2c_device i2c_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(I2C_DEVICE_PROCESS)};

static int i2cdev_open(struct fs_file_t *zfp, const char *name, fs_mode_t flags)
{
    if (zfp == NULL)
    {
        LOG_ERR("Invalid file context");
        return -EINVAL;
    }

    struct i2c_device *i2c = NULL;

    for (int i = 0; i < I2C_NUM; i++)
    {
        if (strcmp(name, i2c_devices[i].label) == 0)
        {
            i2c = &i2c_devices[i];
            break;
        }
    }

    if (i2c == NULL)
    {
        LOG_ERR("I2C device not found: %s", name);
        return -ENODEV;
    }

    // Ensure single open
    if (atomic_set(&i2c->is_open, 1) != 0)
    {
        LOG_ERR("I2C device %s is already open", name);
        return -EBUSY;
    }

    // Reset device state on open
    i2c->addr = 0;
    i2c->flags = 0;
    zfp->filep = i2c;

    LOG_DBG("I2C device %s opened successfully", name);
    return 0;
}

static ssize_t i2cdev_read(struct fs_file_t *zfp, void *buf, size_t nbytes)
{
    if (!zfp || !zfp->filep)
    {
        LOG_ERR("Invalid file context");
        return -EINVAL;
    }

    if (!buf)
    {
        LOG_ERR("Invalid buf");
        return -EFAULT;
    }

    if (nbytes == 0)
    {
        LOG_DBG("Read zero bytes, ignore");
        return 0;
    }
    struct i2c_device *i2c = zfp->filep;
    int ret;

    ret = i2c_read(i2c->dev, (uint8_t *)buf, nbytes, i2c->addr);
    if (ret < 0)
    {
        LOG_ERR("I2C read failed: %d (addr=0x%02X)", ret, i2c->addr);
        return ret;
    }

    LOG_DBG("Read %zu bytes from I2C 0x%02X", nbytes, i2c->addr);
    return nbytes;
}

static ssize_t i2cdev_write(struct fs_file_t *zfp, const void *buf, size_t nbytes)
{
    if (!zfp || !zfp->filep)
    {
        LOG_ERR("Invalid file context");
        return -EINVAL;
    }

    if (!buf)
    {
        LOG_ERR("Invalid buf");
        return -EFAULT;
    }

    if (nbytes == 0)
    {
        LOG_DBG("Write zero bytes, ignore");
        return 0;
    }

    struct i2c_device *i2c = zfp->filep;

    int ret = i2c_write(i2c->dev, (const uint8_t *)buf, nbytes, i2c->addr);
    if (ret < 0)
    {
        LOG_ERR("I2C write failed: %d (addr=0x%02X)", ret, i2c->addr);
        return ret;
    }

    LOG_DBG("I2C writing %zu bytes to device at address 0x%x", nbytes, i2c->addr);
    return nbytes;
}

static int i2cdev_close(struct fs_file_t *zfp)
{
    if (zfp == NULL || zfp->filep == NULL)
    {
        LOG_ERR("Invalid file context");
        return -EINVAL;
    }

    struct i2c_device *i2c = zfp->filep;

    // Reset per-open state
    i2c->addr = 0;
    i2c->flags = 0;

    atomic_clear(&i2c->is_open);

    zfp->filep = NULL;

    LOG_DBG("I2C device closed");
    return 0;
}

static int i2cdev_lseek(struct fs_file_t *filp, off_t off, int whence)
{
    LOG_ERR("lseek is not supported for I2C device");
    return -ENOTSUP;
}

// static int i2cdev_opendir(struct fs_dir_t *dp, const char *path)
// {

//     return 0;
// }

// static int i2cdev_readdir(struct fs_dir_t *dp, struct fs_dirent *entry)
// {

//     return 0;
// }

// static int i2cdev_closedir(struct fs_dir_t *dp)
// {

//     return 0;
// }

static int i2cdev_stat(struct fs_mount_t *mountp,
                       const char *path, struct fs_dirent *entry)
{
    if ((!path) || (!entry))
    {
        return -EINVAL;
    }
    memset(entry, 0, sizeof(struct fs_dirent));

    // 去掉路径末尾的 '/'（如果有）
    size_t len = strlen(path);
    while (len > 0 && path[len - 1] == '/')
        len--;

    // 从去掉尾部斜杠的路径中查找最后一个 '/'
    const char *name_start = path;
    for (ssize_t i = len - 1; i >= 0; i--)
    {
        if (path[i] == '/')
        {
            name_start = path + i + 1;
            break;
        }
    }

    // 复制文件名，确保不越界
    size_t name_len = strlen(name_start);
    if (name_len >= MAX_FILE_NAME)
        name_len = MAX_FILE_NAME - 1; // 预留 null 终止符位置
    memcpy(entry->name, name_start, name_len);
    entry->name[name_len] = '\0';

    // 固定为字符设备文件
    entry->type = FS_DIR_ENTRY_FILE;
    entry->size = 0;

    LOG_DBG("I2C device stat successfully\n");
    return 0;
}

// static int i2cdev_statvfs(struct fs_mount_t *mountp, const char *path,
//                           struct fs_statvfs *stat)
// {

//     return 0;
// }

struct smbus_device_info
{
    const char *i2c_name;
    const char *smbus_name;
};

static struct smbus_device_info smbus_devices[] = {
    {"i2c-1", "smbus1"},
    {"i2c-2", "smbus2"},
    {"i2c-3", "smbus3"},
    {"i2c-4", "smbus4"},
    {"i2c-5", "smbus5"},
    {"i2c-6", "smbus6"},
    {"i2c-7", "smbus7"},
    {"i2c-8", "smbus8"},
    {"i2c-9", "smbus9"},
    {"i2c-10", "smbus10"},
    {"i2c-11", "smbus11"},
    {"i2c-12", "smbus12"},
    {"i2c-13", "smbus13"},
    {"i2c-14", "smbus14"},
    {"i2c-15", "smbus15"},
    {"i2c-16", "smbus16"}};

static const char *get_smbus_name(const struct device *dev)
{
    const char *i2c_name = dev->name;
    int count = ARRAY_SIZE(smbus_devices);

    for (int i = 0; i < count; i++)
    {
        if (strcmp(smbus_devices[i].i2c_name, i2c_name) == 0)
        {
            return smbus_devices[i].smbus_name;
        }
    }

    LOG_DBG("No mapping found for device: %s\n", i2c_name);
    return NULL;
}

static int i2cdev_ioctl_smbus(struct i2c_device *client, uint8_t read_write,
                              uint8_t command, uint32_t size,
                              union i2c_smbus_data *data)
{
    int res = 0;
    if ((size != I2C_SMBUS_BYTE) && (size != I2C_SMBUS_QUICK) &&
        (size != I2C_SMBUS_BYTE_DATA) && (size != I2C_SMBUS_WORD_DATA) &&
        (size != I2C_SMBUS_PROC_CALL) && (size != I2C_SMBUS_BLOCK_DATA) &&
        (size != I2C_SMBUS_I2C_BLOCK_BROKEN) &&
        (size != I2C_SMBUS_I2C_BLOCK_DATA) &&
        (size != I2C_SMBUS_BLOCK_PROC_CALL))
    {
        LOG_ERR("size out of range (%x) in ioctl I2C_SMBUS.", size);
        return -EINVAL;
    }
    if ((read_write != I2C_SMBUS_READ) && (read_write != I2C_SMBUS_WRITE))
    {
        LOG_ERR("read_write out of range (%x) in ioctl I2C_SMBUS.", read_write);
        return -EINVAL;
    }

    // 获取smbus dev信息
    const char *smbusName = get_smbus_name(client->dev);
    if (smbusName == NULL)
    {
        return -EINVAL;
    }
    const struct device *smbus_dev = device_get_binding(smbusName);
    if (!smbus_dev)
    {
        LOG_ERR("SMBus: Device %s not found", smbusName);
        return -ENODEV;
    }

    switch (size)
    {
    case I2C_SMBUS_QUICK:
        res = smbus_quick(smbus_dev, client->addr, read_write);
        break;
    case I2C_SMBUS_BYTE:
        if (read_write == I2C_SMBUS_WRITE)
        {
            res = smbus_byte_write(smbus_dev, client->addr, command);
        }
        else
        {
            res = smbus_byte_read(smbus_dev, client->addr, &data->byte);
        }
        break;
    case I2C_SMBUS_BYTE_DATA:
        if (read_write == I2C_SMBUS_WRITE)
        {
            res = smbus_byte_data_write(smbus_dev, client->addr, command, data->byte);
        }
        else
        {
            res = smbus_byte_data_read(smbus_dev, client->addr, command, &data->byte);
        }
        break;
    case I2C_SMBUS_WORD_DATA:
        if (read_write == I2C_SMBUS_WRITE)
        {
            res = smbus_word_data_write(smbus_dev, client->addr, command, data->word);
        }
        else
        {
            res = smbus_word_data_read(smbus_dev, client->addr, command, &data->word);
        }
        break;
    // case I2C_SMBUS_PROC_CALL:
    //     res = smbus_pcall(smbus_dev, client->addr, command, data->word, &data->word);
    //     if (res >= 0) {
    //         data->word = (u16)res;
    //     }
    //     break;
    case I2C_SMBUS_BLOCK_DATA:

        if (read_write == I2C_SMBUS_WRITE)
        {
            res = smbus_block_write(smbus_dev, client->addr, command, data->block[0], &data->block[1]);
        }
        else
        {
            res = smbus_block_read(smbus_dev, client->addr, command, &data->block[0], &data->block[1]);
        }
        break;
    case I2C_SMBUS_I2C_BLOCK_DATA:
    case_I2C_SMBUS_I2C_BLOCK_DATA:
        if (read_write == I2C_SMBUS_WRITE)
        {
            res = i2c_burst_write(client->dev, client->addr, command, &data->block[1], data->block[0]);
        }
        else
        {
            res = i2c_write_read(client->dev, client->addr, &command, 1, &data->block[1], data->block[0]);
        }
        break;
    // case I2C_SMBUS_BLOCK_PROC_CALL:
    //     res = smbus_block_pcall(smbus_dev, client->addr, command, data->block[0], &data->block[1]);
    //     if (res >= 0) {
    //         data->block[0] = (u8)res;
    //     }
    //     break;
    case I2C_SMBUS_I2C_BLOCK_BROKEN:
        // Convert old I2C block commands to the new convention.
        size = I2C_SMBUS_I2C_BLOCK_DATA;
        if (read_write == I2C_SMBUS_READ)
        {
            data->block[0] = I2C_SMBUS_BLOCK_MAX;
        }
        goto case_I2C_SMBUS_I2C_BLOCK_DATA;
    default:
        LOG_ERR("Unsupported SMBus size (%x) in ioctl I2C_SMBUS.\n", size);
        return -EINVAL;
    }

    return res;
}

static int i2cdev_ioctl(struct fs_file_t *zfp, unsigned long cmd, va_list args)
{
    int ret = 0;

    if (!zfp || !zfp->filep)
    {
        LOG_ERR("Invalid file context");
        return -EINVAL;
    }

    struct i2c_device *i2c = zfp->filep;

    switch (cmd)
    {
    case I2C_SLAVE:
    case I2C_SLAVE_FORCE:
    {
        unsigned long addr = va_arg(args, unsigned long);
        // 10位最大0x3FF；7位最大0x7F
        if ((addr > 0x3FF) ||
            (((i2c->flags & I2C_M_TEN) == 0) && (addr > 0x7F)))
        {
            LOG_ERR("Invalid I2C slave address: 0x%lx", addr);
            return -EINVAL;
        }
        i2c->addr = (uint16_t)addr;
        LOG_DBG("Set I2C slave address to 0x%02x", i2c->addr);
        break;
    }
    case I2C_TENBIT:
    {
        unsigned long tenbit = va_arg(args, unsigned long);
        if (tenbit)
        {
            i2c->flags |= I2C_M_TEN;
        }
        else
        {
            i2c->flags &= ~I2C_M_TEN;
        }
        LOG_DBG("I2C 10-bit addressing: %s", tenbit ? "enabled" : "disabled");
        return 0;
    }
    case I2C_FUNCS:
    {
        unsigned long *funcs = va_arg(args, unsigned long *);
        if (!funcs)
        {
            LOG_ERR("I2C_FUNCS: funcs pointer is NULL");
            return -EINVAL;
        }

        *funcs = (unsigned long)(I2C_FUNC_I2C | I2C_FUNC_10BIT_ADDR | I2C_FUNC_SMBUS_EMUL | I2C_FUNC_SMBUS_BLOCK_DATA);
        LOG_DBG("I2C_FUNCS returned: 0x%lx", *funcs);
        break;
    }
    case I2C_RDWR:
    {
        struct i2c_rdwr_ioctl_data *rdwr_arg = va_arg(args, struct i2c_rdwr_ioctl_data *);

        int nmsgs_sent = 0;

        if (!rdwr_arg)
        {
            LOG_ERR("I2C_RDWR: rdwr_arg is NULL");
            return -EINVAL;
        }

        if (rdwr_arg->nmsgs == 0)
        {
            LOG_DBG("I2C_RDWR: zero messages, nothing to do");
            return 0;
        }

        if (rdwr_arg->nmsgs > I2C_RDWR_IOCTL_MAX_MSGS)
        {
            LOG_ERR("I2C_RDWR: too many messages (%d), max is %d",
                    rdwr_arg->nmsgs, I2C_RDWR_IOCTL_MAX_MSGS);
            return -EINVAL;
        }

        struct i2c_msg *msgs = (struct i2c_msg *)k_malloc(rdwr_arg->nmsgs * sizeof(struct i2c_msg));
        if (!msgs)
        {
            return -ENOMEM;
        }

        nmsgs_sent = rdwr_arg->nmsgs;

        // 转换消息数组
        for (int i = 0; i < nmsgs_sent; i++)
        {
            if (!rdwr_arg->msgs[i].buf && rdwr_arg->msgs[i].len > 0)
            {
                LOG_ERR("Invalid buffer pointer for message %d", i);
                k_free(msgs);
                return -EINVAL;
            }
            msgs[i].buf = (uint8_t *)rdwr_arg->msgs[i].buf;
            msgs[i].len = rdwr_arg->msgs[i].len;
            uint16_t lf = rdwr_arg->msgs[i].flags;

            msgs[i].flags = (uint8_t)((lf & I2C_M_RD ? I2C_MSG_READ : I2C_MSG_WRITE) |
                                      (lf & I2C_M_TEN ? I2C_MSG_ADDR_10_BITS : 0) |
                                      (lf & I2C_M_NOSTART ? 0 : I2C_MSG_RESTART) |
                                      (lf & I2C_M_STOP ? I2C_MSG_STOP : 0));
            if (i == 0)
            {
                msgs[i].flags |= I2C_MSG_RESTART;
            }
        }
        // msgs[rdwr_arg->nmsgs - 1].flags |= I2C_MSG_STOP;

        int result = i2c_transfer(i2c->dev, msgs, nmsgs_sent, rdwr_arg->msgs[0].addr);

        k_free(msgs);
        if (result < 0)
        {
            LOG_ERR("i2c_transfer failed with error: %d", result);
            return result;
        }

        LOG_DBG("I2C_RDWR completed: %d messages processed", nmsgs_sent);

        return nmsgs_sent;
    }
    case I2C_PEC:
    {
        /*
         * Setting the PEC flag here won't affect kernel drivers,
         * which will be using the i2c_client node registered with
         * the driver model core.  Likewise, when that client has
         * the PEC flag already set, the i2c-dev driver won't see
         * (or use) this setting.
         */
        int arg = va_arg(args, int);
        if (arg)
            i2c->flags |= I2C_CLIENT_PEC;
        else
            i2c->flags &= ~I2C_CLIENT_PEC;
        return 0;
    }
    case I2C_SMBUS:
    {
        struct i2c_smbus_ioctl_data *data_arg_ptr = va_arg(args, struct i2c_smbus_ioctl_data *);
        struct i2c_smbus_ioctl_data data_arg;

        if (!data_arg_ptr)
        {
            LOG_ERR("I2C_SMBUS: data_arg_ptr is NULL");
            return -EINVAL;
        }

        data_arg = *data_arg_ptr;
        return i2cdev_ioctl_smbus(i2c, data_arg.read_write,
                                  data_arg.command, data_arg.size,
                                  data_arg.data);
    }
    case ZFD_IOCTL_SET_LOCK:
        // Single open mode, no file lock required
        LOG_ERR("ioctl ZFD_IOCTL_SET_LOCK----\n");
        return 0;
    default:
        LOG_ERR("Unsupported I2C ioctl request: 0x%lx", cmd);
        return -ENOTTY;
    }

    return ret;
}

static const struct fs_file_system_t i2cdev_fs = {
    .open = i2cdev_open,
    .close = i2cdev_close,
    .read = i2cdev_read,
    .write = i2cdev_write,
    .lseek = i2cdev_lseek,
    // .opendir = i2cdev_opendir,
    // .readdir = i2cdev_readdir,
    // .closedir = i2cdev_closedir,
    .stat = i2cdev_stat,
    // .statvfs = i2cdev_statvfs,
    .ioctl = i2cdev_ioctl,
};

int i2cdev_fs_init(void)
{
    char name[32];
    LOG_DBG("i2c /dev entries driver\n");

    LOG_DBG("i2c_num = %d\n", I2C_NUM);

    for (int i = 0; i < I2C_NUM; i++)
    {
        if (!device_is_ready(i2c_devices[i].dev))
        {
            LOG_ERR("I2C device %s is not ready", i2c_devices[i].label);
            continue;
        }

        if (i2c_devices[i].dev->name != NULL)
        {
            LOG_DBG("i2c_devices[%d].dev->name = %s\n", i, i2c_devices[i].dev->name);
        }
        if (i2c_devices[i].label != NULL)
        {
            LOG_DBG("i2c_devices[%d].label = %s\n", i, i2c_devices[i].label);
        }
        snprintf(name, sizeof(name) - 1, "/dev/%s", i2c_devices[i].label);
        devfs_register(name, &i2cdev_fs);
    }

    return 0;
}

SYS_INIT(i2cdev_fs_init, POST_KERNEL, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
