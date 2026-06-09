#ifndef __I2C_FS_H__
#define __I2C_FS_H__

#include <zephyr/device.h>
#include <zephyr/sys/atomic.h>

struct i2c_device
{
    const struct device *dev;
    const char *label;
    uint16_t addr;
    atomic_t is_open;
    uint8_t flags;
};

#endif /* __I2C_FS_H__ */
