#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/sysfs.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_i2c, LOG_LEVEL_INF);

#define _HWMON_CLASS_5 1
#define HWMON_CLASS_TO_FLAG_RAW(x) _HWMON_CLASS_##x
#define HWMON_CLASS_TO_FLAG(x) HWMON_CLASS_TO_FLAG_RAW(x)

struct hwmon_i2c
{
    const struct device *dev;
    uint32_t addr;
    struct sysfs_attribute attribute;
    int ppos;
};

struct hwmon_dev
{
    const char *label;
    struct hwmon_i2c *i2c_dev;
    size_t i2c_dev_num;
    struct sysfs_node *node;
    struct sysfs_attribute name;
    int ppos;
};

#define HWMON_I2C_ENTRY(node_id, prop, idx)                \
    {                                                     \
        .dev     = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, hwmon_ins, idx)), \
        .addr = DT_PHA_BY_IDX(node_id,hwmon_ins,idx,channel)\
    },

#define HWMON_I2C_LIST_GEN(inst)                                    \
    static struct hwmon_i2c hwmon_i2c_list_##inst[] = {             \
        DT_INST_FOREACH_PROP_ELEM(inst, hwmon_ins, HWMON_I2C_ENTRY) \
    };

#define HWMON_I2C_LIST(inst)                                        \
    COND_CODE_1(                                                    \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)),             \
        (HWMON_I2C_LIST_GEN(inst)),                                 \
        ()                                                          \
    )

DT_INST_FOREACH_STATUS_OKAY(HWMON_I2C_LIST);

#define HWMON_DEVICE_ENTRY(inst)                                         \
    COND_CODE_1(                                                         \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)),                  \
        ({                                                               \
            .label       = DT_INST_PROP(inst, label),                    \
            .i2c_dev     = hwmon_i2c_list_##inst,                        \
            .i2c_dev_num = ARRAY_SIZE(hwmon_i2c_list_##inst)             \
        },),                                                             \
        ()                                                               \
    )

static struct hwmon_dev hwmon_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(HWMON_DEVICE_ENTRY)
};

static int i2c_attr_open(sysfs_attr_t attr)
{
    struct hwmon_i2c *i2c = attr->user_data;
    LOG_DBG("%s addr (%d) opend.", i2c->dev->name, i2c->addr);
    return 0;
}

static ssize_t i2c_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_i2c *i2c = attr->user_data;
    LOG_DBG("%s addr (%d) read.", i2c->dev->name, i2c->addr);
    return 0;
}

static int i2c_attr_close(sysfs_attr_t attr)
{
    struct hwmon_i2c *i2c = attr->user_data;
    LOG_DBG("%s addr (%d) closed.", i2c->dev->name, i2c->addr);
    return 0;
}

static struct sysfs_attribute_ops i2c_attr_ops = {
    .open = i2c_attr_open,
    .read = i2c_attr_read,
    .close = i2c_attr_close
};

static ssize_t name_open(sysfs_attr_t attr)
{
    struct hwmon_dev *dev = attr->user_data;
    LOG_DBG("%s attribute (%s) opend.", dev->label, attr->name);
    dev->ppos = 0;
    return 0;
}

static ssize_t name_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_dev *dev = attr->user_data;
    const char *name = "i2c-sensor";
    ssize_t len;

    if (dev->ppos > 0)
        return 0;

    len = snprintf(buf, size, "%s\n", name);
    if (len < 0)
    {
        LOG_ERR("%s read name fail.", dev->label);
        return len;
    }

    if (len > size)
        len = size;

    dev->ppos = len;
    return len;
}

static ssize_t name_close(sysfs_attr_t attr)
{
    struct hwmon_dev *dev = attr->user_data;
    LOG_DBG("%s attribute (%s) closed.", dev->label, attr->name);
    dev->ppos = 0;
    return 0;
}

static struct sysfs_attribute_ops name_attr_ops = {
    .open = name_open,
    .read = name_read,
    .close = name_close
};

static int hwmon_fs_init(void)
{
    char path[64];
    struct hwmon_dev *dev;
    struct sysfs_attribute *attr;

    for (int i = 0; i < ARRAY_SIZE(hwmon_devices); i++)
    {
        dev = &hwmon_devices[i];

        if (snprintf(path, 64, "/sys/class/hwmon/%s", dev->label) >= sizeof(path))
        {
            LOG_ERR("hwmon i2c path too long");
            break;
        }

        dev->node = sysfs_mkdir(path);
        if (!dev->node)
        {
            LOG_ERR("mkdir failed: %s", path);
            break;
        }

        for (int j = 0; j < dev->i2c_dev_num; j++)
        {
            attr = &dev->i2c_dev[j].attribute;

            snprintf(attr->name, 
                     SYSFS_ATTRIBUTE_NAME_MAX,
                     "i2c_%d", j);

            attr->user_data = &dev->i2c_dev[j];
            attr->ops = &i2c_attr_ops;

            sysfs_add_attribute(dev->node, attr);
        }

        strncpy(dev->name.name, "name", SYSFS_ATTRIBUTE_NAME_MAX);
        dev->name.ops = &name_attr_ops;
        dev->name.user_data = dev;
        sysfs_add_attribute(dev->node, &dev->name);
    }

    return 0;
}
SYS_INIT(hwmon_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);