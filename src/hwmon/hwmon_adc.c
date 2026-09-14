#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/fs/sysfs.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/drivers/adc.h>
#include <ls_hal_adcv2.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_adc, LOG_LEVEL_INF);

/*
 * Number of ADC channels converted by one regular sequence (LSQSH ADC fills
 * REG_DAT00..REG_DAT12, 13 channels). The sequence buffer holds one result per
 * configured channel, in configuration order, so the size of the read buffer
 * must be the number of channels converted by one sequence. Keep this in sync
 * with `nbr_of_conversion` of the ADC node.
 */
#define HWMON_ADC_CHANNEL_NUM       16

/* The ADC driver numbers the channels of every device separately, so keep one
 * rank counter per ADC device (LSQSH has adc1/adc2). */
#define HWMON_ADC_DEV_MAX           2

#define _HWMON_CLASS_2                  1
#define HWMON_CLASS_TO_FLAG_RAW(x)      _HWMON_CLASS_##x
#define HWMON_CLASS_TO_FLAG(x)          HWMON_CLASS_TO_FLAG_RAW(x)

struct hwmon_adc
{
    const struct device *dev;
    struct sysfs_attribute attribute;
    int ppos;
    struct k_poll_signal poll_signal;
    uint8_t channel;
    uint8_t rank;       /* position of the channel in the ADC sequence */
    uint8_t inited;
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

#define HWMON_ADC_ENTRY(node_id, prop, idx)                               \
    {                                                                     \
        .dev = DEVICE_DT_GET(DT_PHANDLE_BY_IDX(node_id, hwmon_ins, idx)), \
        .channel = DT_PHA_BY_IDX(node_id, hwmon_ins, idx, channel),       \
        .inited = 0,                                                      \
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

/*
 * The ADC regular sequence stores one result per configured channel, in the
 * order the channels are set up on that device, so the rank of a channel is
 * the number of channels already set up on the same ADC device.
 */
static const struct device *hwmon_adc_rank_dev[HWMON_ADC_DEV_MAX];
static uint8_t hwmon_adc_rank_cnt[HWMON_ADC_DEV_MAX];
static size_t hwmon_adc_rank_num = 0;

static uint8_t hwmon_adc_next_rank(const struct device *adc_dev)
{
    size_t i = 0;

    for (i = 0; i < hwmon_adc_rank_num; i++)
    {
        if (hwmon_adc_rank_dev[i] == adc_dev)
            break;
    }

    if (i == hwmon_adc_rank_num)
    {
        __ASSERT(i < HWMON_ADC_DEV_MAX, "too many ADC devices");

        hwmon_adc_rank_dev[i] = adc_dev;
        hwmon_adc_rank_cnt[i] = 0;
        hwmon_adc_rank_num++;
    }

    return hwmon_adc_rank_cnt[i]++;
}

static int adc_attr_open(sysfs_attr_t attr)
{
    struct hwmon_adc *adc = attr->user_data;
    adc->ppos = 0;

    if (!adc->inited)
    {
        k_poll_signal_init(&adc->poll_signal);

        adc->inited = 1;
    }
    LOG_DBG("%s channel (%d) opend.", adc->dev->name, adc->channel);
    return 0;
}

static ssize_t adc_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_adc *adc = attr->user_data;
    uint16_t values[HWMON_ADC_CHANNEL_NUM] = { 0 };
    uint16_t value;
    int ret;

    if (adc->ppos > 0)
        return 0;

    if (adc->rank >= HWMON_ADC_CHANNEL_NUM)
    {
        LOG_ERR("%s channel (%d) rank (%d) out of range.",
                adc->dev->name, adc->channel, adc->rank);
        return -EINVAL;
    }

    /* The channels were configured at driver load time: one read converts the
     * whole sequence, the wanted channel is picked by the position it was set
     * up in. */
    struct adc_sequence sequence = {
        .buffer = values,
        .buffer_size = sizeof(values),
        .channels = adc->channel,
        // .resolution = 12,
    };

    ret = adc_read(adc->dev, &sequence);
    if (ret)
    {
        LOG_ERR("%s channel (%d) read fail: %d.", adc->dev->name, adc->channel, ret);
        return ret;
    }

    value = values[adc->rank];

    LOG_DBG("%s channel (%d) value: %u.", adc->dev->name, adc->channel, value);

    int len = snprintf(buf, size, "%u\n", value);
    if (len > 0)
    {
        adc->ppos = len;
        k_poll_signal_reset(&adc->poll_signal);
    }

    return len;
}

static int adc_attr_close(sysfs_attr_t attr)
{
    struct hwmon_adc *adc = attr->user_data;
    adc->ppos = 0;
    k_poll_signal_reset(&adc->poll_signal);
    LOG_DBG("%s channel (%d) closed.", adc->dev->name, adc->channel);

    return 0;
}


static int hwmon_poll_prepare_ctx(struct hwmon_adc *adc,
                                  struct zvfs_pollfd *pfd,
                                  struct k_poll_event **pev,
                                  struct k_poll_event *pev_end)
{
    if (pfd->events & ZVFS_POLLIN)
    {
        if (*pev >= pev_end)
        {
            return -ENOMEM;
        }

        (*pev)->obj = &adc->poll_signal;
        (*pev)->type = K_POLL_TYPE_SIGNAL;
        (*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
        (*pev)->state = K_POLL_STATE_NOT_READY;
        (*pev)++;

        if (adc->ppos == 0)
        {
            k_poll_signal_raise(&adc->poll_signal, 0);
        }
    }

    return 0;
}

static int hwmon_poll_update_ctx(struct hwmon_adc *adc,
                                 struct zvfs_pollfd *pfd,
                                 struct k_poll_event **pev)
{
    if (pfd->events & ZVFS_POLLIN)
    {
        if ((*pev)->state != K_POLL_STATE_NOT_READY && (adc->ppos == 0))
        {
            pfd->revents |= ZVFS_POLLIN;
        }
        (*pev)++;
    }

    return 0;
}

static int adc_attr_ioctl(sysfs_attr_t attr, unsigned long request, va_list args)
{
    LOG_DBG("adc ioctl: 0x%lx", request);
    struct hwmon_adc *adc = attr->user_data;

    if (!adc)
    {
        LOG_ERR("No adc context");
        return -EINVAL;
    }

    switch (request)
    {
    case F_GETFL: // TODO:
        return O_RDWR;

    case F_SETFL: // TODO:
    {
        return 0;
    }

    case ZFD_IOCTL_POLL_PREPARE:
    {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
        struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);
        return hwmon_poll_prepare_ctx(adc, pfd, pev, pev_end);
    }

    case ZFD_IOCTL_POLL_UPDATE:
    {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
        return hwmon_poll_update_ctx(adc, pfd, pev);
    }

    case ZFD_IOCTL_POLL_OFFLOAD:
    {
        k_poll_signal_reset(&adc->poll_signal);
        return 0;
    }

    case ZFD_IOCTL_SET_LOCK:
        return 0;

    default:
        LOG_ERR("Unsupported ioctl request: 0x%lx", request);
        return -EOPNOTSUPP;
    }

    return 0;
}

static struct sysfs_attribute_ops adc_attr_ops = {
    .open = adc_attr_open,
    .read = adc_attr_read,
    .write = NULL,
    .close = adc_attr_close,
    .ioctl = adc_attr_ioctl
};

static ssize_t name_open(sysfs_attr_t attr)
{
    struct hwmon_dev *dev = attr->user_data;
    LOG_DBG("%s attribute (%s) opened.", dev->label, attr->name);
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
    .write = NULL,
    .close = name_close,
    .ioctl = NULL
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

            // Generate hwmon input file names for OpenBMC sensor compatibility
            // https://github.com/openbmc/linux/blob/master/drivers/hwmon/iio_hwmon.c#L131-L133
            snprintf(attr->name,
                     SYSFS_ATTRIBUTE_NAME_MAX,
#ifdef CONFIG_OPENBMC_ZEPHYR
                     "in%d_input", j + 1);
#else
                     "in%d_input", j);
#endif /* CONFIG_OPENBMC_ZEPHYR */

            attr->user_data = &dev->adc_dev[j];
            attr->ops = &adc_attr_ops;

            sysfs_add_attribute(dev->node, attr);

            /* The ADC driver is loaded at this point, configure the channel
             * here once so that a read only has to trigger the sequence. */
            struct adc_channel_cfg channel_config = {
                .channel_id = dev->adc_dev[j].channel,
                .reference = ADC_REF_VDD_1,
                .acquisition_time = ADC_SAMPLETIME_15CYCLES,
            };

            if (adc_channel_setup(dev->adc_dev[j].dev, &channel_config) != 0)
            {
                LOG_ERR("%s channel (%d) setup fail.",
                        dev->adc_dev[j].dev->name, dev->adc_dev[j].channel);
                continue;
            }

            /* Remember where this channel's value lands in the sequence
             * buffer: the results are stored in setup order per device. */
            dev->adc_dev[j].rank = hwmon_adc_next_rank(dev->adc_dev[j].dev);
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
