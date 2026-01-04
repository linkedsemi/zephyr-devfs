#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/fs/sysfs.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <ls_hal_adcv2.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_adc, LOG_LEVEL_INF);

#define _HWMON_CLASS_2                  1
#define HWMON_CLASS_TO_FLAG_RAW(x)      _HWMON_CLASS_##x
#define HWMON_CLASS_TO_FLAG(x)          HWMON_CLASS_TO_FLAG_RAW(x)

struct hwmon_adc
{
    const struct device *dev;
    uint8_t channel;
    struct sysfs_attribute attribute;
    int ppos;
};

struct hwmon_dev
{
    const char *label;
    struct hwmon_adc *adc_dev;
    size_t adc_dev_num;
    struct sysfs_node *node;
    struct sysfs_attribute name_attr;
    int ppos;
};

#define HWMON_ADC_ENTRY(node_id, prop, idx)                \
    {                                                     \
        .dev     = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, hwmon_ins, idx)), \
        .channel = DT_PHA_BY_IDX(node_id, hwmon_ins, idx, channel),          \
    },

#define HWMON_ADC_LIST_GEN(inst)                                    \
    static struct hwmon_adc hwmon_adc_list_##inst[] = {             \
        DT_INST_FOREACH_PROP_ELEM(inst, hwmon_ins, HWMON_ADC_ENTRY) \
    };

#define HWMON_ADC_LIST(inst)                                        \
    COND_CODE_1(                                                    \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)),             \
        (HWMON_ADC_LIST_GEN(inst)),                                 \
        ()                                                          \
    )

DT_INST_FOREACH_STATUS_OKAY(HWMON_ADC_LIST);

#define HWMON_DEVICE_ENTRY(inst)                                         \
    COND_CODE_1(                                                         \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)),                  \
        ({                                                               \
            .label       = DT_INST_PROP(inst, label),                    \
            .adc_dev     = hwmon_adc_list_##inst,                        \
            .adc_dev_num = ARRAY_SIZE(hwmon_adc_list_##inst)             \
        },),                                                             \
        ()                                                               \
    )

static struct hwmon_dev hwmon_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(HWMON_DEVICE_ENTRY)
};

static int adc_attr_open(sysfs_attr_t attr)
{
    struct hwmon_adc *adc = attr->user_data;
    LOG_DBG("%s channel (%d) opend.", adc->dev->name, adc->channel);
    return 0;
}

static ssize_t adc_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_adc *adc = attr->user_data;
    uint16_t value;
    int ret;

    if (adc->ppos > 0)
        return 0;

    struct adc_channel_cfg channel_config = {
        .channel_id = adc->channel,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_SAMPLETIME_15CYCLES,
    };

    ret = adc_channel_setup(adc->dev, &channel_config);
    if (ret)
    {
        LOG_ERR("%s channel (%d) setup fail.", adc->dev->name, adc->channel);
        return ret;
    }

    struct adc_sequence sequence = {
        .buffer = &value,
        .buffer_size = sizeof(value),
    };

    ret = adc_read(adc->dev, &sequence);
    if (ret)
    {
        LOG_ERR("%s channel (%d) read fail.", adc->dev->name, adc->channel);
        return ret;
    }

    LOG_DBG("%s channel (%d) value: %u.", adc->dev->name, adc->channel, value);

    int len = snprintf(buf, size, "%u\n", value);
    if (len > 0)
    {
        adc->ppos = len;
    }

    return len;
}

static int adc_attr_close(sysfs_attr_t attr)
{
    struct hwmon_adc *adc = attr->user_data;
    adc->ppos = 0;
    LOG_DBG("%s channel (%d) closed.", adc->dev->name, adc->channel);

    return 0;
}

static struct sysfs_attribute_ops adc_attr_ops = {
    .open = adc_attr_open,
    .read = adc_attr_read,
    .close = adc_attr_close
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
    const char *name = "iio_hwmon";
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

        if (snprintf(path, sizeof(path),
                     "/sys/class/hwmon/%s",
                     dev->label) >= sizeof(path))
        {
            LOG_ERR("hwmon path too long");
            break;
        }

        dev->node = sysfs_mkdir(path);
        if (!dev->node)
        {
            LOG_ERR("mkdir failed: %s", path);
            break;
        }

        for (int j = 0; j < dev->adc_dev_num; j++)
        {
            attr = &dev->adc_dev[j].attribute;

            snprintf(attr->name,
                     SYSFS_ATTRIBUTE_NAME_MAX,
                     "in%d_input", j);

            attr->user_data = &dev->adc_dev[j];
            attr->ops = &adc_attr_ops;

            sysfs_add_attribute(dev->node, attr);
        }

        snprintf(dev->name_attr.name,
                 SYSFS_ATTRIBUTE_NAME_MAX,
                 "name");

        dev->name_attr.ops = &name_attr_ops;
        dev->name_attr.user_data = dev;

        sysfs_add_attribute(dev->node, &dev->name_attr);
    }

    return 0;
}
SYS_INIT(hwmon_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);