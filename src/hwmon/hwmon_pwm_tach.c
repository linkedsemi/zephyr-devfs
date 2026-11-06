#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/fs/sysfs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_pwm_tach, LOG_LEVEL_INF);

#define _HWMON_CLASS_0 1
#define HWMON_CLASS_TO_FLAG_RAW(x) _HWMON_CLASS_##x
#define HWMON_CLASS_TO_FLAG(x) HWMON_CLASS_TO_FLAG_RAW(x)

/* Default PWM period: 25kHz = 40000ns (standard fan PWM frequency) */
#define PWM_FAN_PERIOD_NS PWM_USEC(40)
// #define PWM_FAN_PERIOD_NS PWM_HZ(100)
struct fan_pwm_tach
{
    const struct device *cap_dev;
    const struct device *pwm_dev;
    uint8_t cap_channel;
    uint8_t pwm_channel;
    uint8_t pwm_value;
    uint8_t pwm_enable;
    struct sysfs_attribute fan_attr;
    struct sysfs_attribute pwm_attr;
    struct sysfs_attribute pwm_en_attr;
    struct k_poll_signal poll_signal;
    uint8_t inited;
    /* Sysfs ops only see the attribute, not the open file, so "value already
     * handed out" has to live on the attribute. Re-armed by open() and, for
     * fanN_input, by poll_prepare(): poll driven readers get one value per
     * poll cycle, plain fs readers (`fs cat`) see value + EOF. */
    uint8_t read_done;      /* fanN_input */
    uint8_t pwm_done;       /* pwmN */
    uint8_t pwm_en_done;    /* pwmN_enable */
};

struct hwmon_fan_dev
{
    const char *label;
    struct fan_pwm_tach *fan_dev;
    struct sysfs_node *node;
    struct sysfs_attribute name_attr;
    struct sysfs_attribute label_attr; /* device-level "label" (DT label) */
    uint8_t name_done;      /* read cursor of the name file */
    uint8_t dev_label_done; /* read cursor of the label file */
};

#define FAN_PWM_TACH_DEFINE(inst)                                              \
    static struct fan_pwm_tach fan_tach_##inst = {                             \
        .pwm_channel = DT_INST_PHA_BY_IDX(inst, hwmon_outs, 0, channel),       \
        .pwm_dev = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, hwmon_outs, 0)), \
        .cap_channel = DT_INST_PHA_BY_IDX(inst, hwmon_ins, 0, channel),        \
        .cap_dev = DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(inst, hwmon_ins, 0)),  \
        .inited = 0,                                                           \
        .pwm_value = 0,                                                        \
        .pwm_enable = 1,                                                       \
    };
#define FAN_TACH_DEFINE(inst)                           \
    COND_CODE_1(                                        \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)), \
        (FAN_PWM_TACH_DEFINE(inst)),                    \
        ())
DT_INST_FOREACH_STATUS_OKAY(FAN_TACH_DEFINE)

#define FAN_SENSOR_ENTRY(inst)              \
    {                                       \
        .label = DT_INST_PROP(inst, label), \
        .fan_dev = &fan_tach_##inst,        \
    },

#define FAN_DEVICE_ENTRY(inst)                          \
    COND_CODE_1(                                        \
        HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)), \
        (FAN_SENSOR_ENTRY(inst)),                       \
        ())

static struct hwmon_fan_dev hwmon_fan_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(FAN_DEVICE_ENTRY)};
/* --- fan1_input (tach) sysfs operations --- */

static int fan_attr_open(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    /* A fresh descriptor starts at offset 0: re-arm the one-shot value. */
    fan->read_done = 0;
    if (!fan->inited)
    {
        k_poll_signal_init(&fan->poll_signal);
        fan->inited = 1;
    }
    LOG_DBG("fan_input opened. cap_ch=%d", fan->cap_channel);
    return 0;
}

static ssize_t fan_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct fan_pwm_tach *fan = attr->user_data;
    int len;
    int ret;

    /* Value already handed out on this open/poll cycle -> EOF. */
    if (fan->read_done)
        return 0;

    /* Read RPM from cap sensor via sensor_sample_fetch_chan/sensor_channel_get */
    enum sensor_channel cap_ch = (enum sensor_channel)(SENSOR_CHAN_CAP_01 + fan->cap_channel);
    ret = sensor_sample_fetch_chan(fan->cap_dev, cap_ch);
    if (ret)
    {
        LOG_ERR("sensor_sample_fetch_chan failed for cap_ch %d: %d", fan->cap_channel, ret);
        return ret;
    }

    struct sensor_value val;
    ret = sensor_channel_get(fan->cap_dev, cap_ch, &val);
    if (ret)
    {
        LOG_ERR("sensor_channel_get failed for cap_ch %d: %d", fan->cap_channel, ret);
        return ret;
    }

    /* val.val1 == frequency in Hz, RPM = val.val1 */
    uint32_t rpm = (uint32_t)((uint64_t)val.val1 * 60 / 2);

    len = snprintf(buf, size, "%u\n", rpm);

    if (len > 0)
    {
        fan->read_done = 1;
    }
    k_poll_signal_reset(&fan->poll_signal);

    return len;
}

static int fan_attr_close(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    k_poll_signal_reset(&fan->poll_signal);
    LOG_DBG("fan_input closed.");
    return 0;
}

/* --- pwm1 sysfs operations --- */

static int pwm_attr_open(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    fan->pwm_done = 0;
    LOG_DBG("pwm opened. pwm_ch=%d", fan->pwm_channel);
    return 0;
}

static ssize_t pwm_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct fan_pwm_tach *fan = attr->user_data;
    int len;

    if (fan->pwm_done)
        return 0;

    /* Read back the stored PWM value (0-255) */
    len = snprintf(buf, size, "%u\n", fan->pwm_value);
    if (len > 0)
    {
        fan->pwm_done = 1;
    }
    return len;
}

static ssize_t pwm_attr_write(sysfs_attr_t attr, const void *buf, size_t size)
{
    struct fan_pwm_tach *fan = attr->user_data;
    unsigned long val;
    uint32_t pulse_ns;
    int ret;

    char tmp[64];
    size_t cp_sz = MIN(size, sizeof(tmp) - 1);
    memcpy(tmp, buf, cp_sz);
    tmp[cp_sz] = '\0';
    /* Parse the input value (0-255) */
    val = strtoul(buf, NULL, 0);
    if (val > 255)
        val = 255;
    /* Convert 0-255 to pulse width in nanoseconds */
    pulse_ns = (PWM_FAN_PERIOD_NS * (uint32_t)val) / 255U;

    /* Set PWM hardware with period and duty */
    ret = pwm_set(fan->pwm_dev, fan->pwm_channel,
                  PWM_FAN_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
    if (ret < 0)
    {
        LOG_ERR("pwm_set failed: %d", ret);
        return ret;
    }

    /* Store the value for read-back */
    fan->pwm_value = (uint8_t)val;

    LOG_DBG("pwm write: ch=%d val=%lu pulse_ns=%u",
            fan->pwm_channel, val, pulse_ns);

    return size;
}

static int pwm_attr_close(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    LOG_DBG("pwm closed.");
    return 0;
}

/* --- ioctl / poll operations --- */

/* fanN_input is generated on demand in fan_attr_read(), so a freshly opened
 * descriptor is always readable: report POLLIN unconditionally. The read
 * cursor is attribute wide and must not gate POLLIN here (one reader that had
 * already consumed the value used to silence the others). Instead every poll
 * re-arms it: a poll driven reader (boost::asio keeps its fd open forever)
 * gets one value per poll cycle, while `fs cat` still ends on EOF. */
static int fan_poll_prepare(struct fan_pwm_tach *fan,
                            struct zvfs_pollfd *pfd,
                            struct k_poll_event **pev,
                            struct k_poll_event *pev_end)
{
    if (pfd->events & ZVFS_POLLIN)
    {
        if (*pev >= pev_end)
            return -ENOMEM;

        (*pev)->obj = &fan->poll_signal;
        (*pev)->type = K_POLL_TYPE_SIGNAL;
        (*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
        (*pev)->state = K_POLL_STATE_NOT_READY;
        (*pev)++;

        fan->read_done = 0;
        k_poll_signal_raise(&fan->poll_signal, 0);
    }
    return 0;
}

static int fan_poll_update(struct fan_pwm_tach *fan,
                           struct zvfs_pollfd *pfd,
                           struct k_poll_event **pev)
{
    (void)fan;

    if (pfd->events & ZVFS_POLLIN)
    {
        if ((*pev)->state != K_POLL_STATE_NOT_READY)
            pfd->revents |= ZVFS_POLLIN;
        (*pev)++;
    }
    return 0;
}

static int fan_attr_ioctl(sysfs_attr_t attr, unsigned long request, va_list args)
{
    struct fan_pwm_tach *fan = attr->user_data;

    if (!fan)
    {
        LOG_ERR("No fan context");
        return -EINVAL;
    }

    switch (request)
    {
    case F_GETFL:
        return O_RDWR;
    case F_SETFL:
        return 0;
    case ZFD_IOCTL_POLL_PREPARE:
    {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
        struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);
        return fan_poll_prepare(fan, pfd, pev, pev_end);
    }
    case ZFD_IOCTL_POLL_UPDATE:
    {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
        return fan_poll_update(fan, pfd, pev);
    }
    case ZFD_IOCTL_POLL_OFFLOAD:
        k_poll_signal_reset(&fan->poll_signal);
        return 0;
    case ZFD_IOCTL_SET_LOCK:
        return 0;
    default:
        LOG_ERR("Unsupported ioctl: 0x%lx", request);
        return -EOPNOTSUPP;
    }
}

static int pwm_attr_ioctl(sysfs_attr_t attr, unsigned long request, va_list args)
{
    struct fan_pwm_tach *fan = attr->user_data;

    if (!fan)
    {
        LOG_ERR("No fan context");
        return -EINVAL;
    }

    switch (request)
    {
    case F_GETFL:
        return O_RDWR;
    case F_SETFL:
        return 0;
    case ZFD_IOCTL_SET_LOCK:
        return 0;
    default:
        LOG_ERR("Unsupported pwm ioctl: 0x%lx", request);
        return -EOPNOTSUPP;
    }
}

static struct sysfs_attribute_ops fan_attr_ops = {
    .open = fan_attr_open,
    .read = fan_attr_read,
    .write = NULL,
    .close = fan_attr_close,
    .ioctl = fan_attr_ioctl,
};

static struct sysfs_attribute_ops pwm_attr_ops = {
    .open = pwm_attr_open,
    .read = pwm_attr_read,
    .write = pwm_attr_write,
    .close = pwm_attr_close,
    .ioctl = pwm_attr_ioctl,
};

/* --- name attribute --- */

static ssize_t name_open(sysfs_attr_t attr)
{
    struct hwmon_fan_dev *dev = attr->user_data;
    dev->name_done = 0;
    return 0;
}

static ssize_t name_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_fan_dev *dev = attr->user_data;
    const char *name = "aspeed,ast2400-pwm-tacho";
    ssize_t len;

    if (dev->name_done)
        return 0;

    len = snprintf(buf, size, "%s\n", name);
    if (len < 0)
    {
        LOG_ERR("%s read name fail.", dev->label);
        return len;
    }

    if (len > size)
        len = size;

    dev->name_done = 1;
    return len;
}

static ssize_t name_close(sysfs_attr_t attr)
{
    ARG_UNUSED(attr);
    return 0;
}

static struct sysfs_attribute_ops name_attr_ops = {
    .open = name_open,
    .read = name_read,
    .write = NULL,
    .close = name_close,
    .ioctl = NULL,
};

/* --- device label attribute ---
 * Device-level "label" file holding the DT label (e.g. "hwmon1"). Not a Linux
 * hwmon file, but OpenBMC consumers look it up to identify a hwmon node by
 * label (dbus-sensors' ChassisIntrusion opens label on every hwmon node);
 * without it each such open fails with -EINVAL and logs an error. */
static int dev_label_open(sysfs_attr_t attr)
{
    struct hwmon_fan_dev *dev = attr->user_data;
    dev->dev_label_done = 0;
    return 0;
}

static ssize_t dev_label_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct hwmon_fan_dev *dev = attr->user_data;
    ssize_t len;

    if (dev->dev_label_done)
        return 0;

    len = snprintf(buf, size, "%s\n", dev->label);
    if (len < 0)
    {
        LOG_ERR("%s read label fail.", dev->label);
        return len;
    }

    if (len > size)
        len = size;

    dev->dev_label_done = 1;
    return len;
}

static ssize_t dev_label_close(sysfs_attr_t attr)
{
    ARG_UNUSED(attr);
    return 0;
}

static struct sysfs_attribute_ops dev_label_attr_ops = {
    .open = dev_label_open,
    .read = dev_label_read,
    .write = NULL,
    .close = dev_label_close,
    .ioctl = NULL,
};

/* --- pwm_enable attribute --- */

static int pwm_en_attr_open(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    fan->pwm_en_done = 0;
    LOG_DBG("pwm enable opened. pwm_ch=%d", fan->pwm_channel);
    return 0;
}

static int pwm_en_attr_close(sysfs_attr_t attr)
{
    struct fan_pwm_tach *fan = attr->user_data;
    LOG_DBG("pwm enable closed.");
    return 0;
}

static ssize_t pwm_en_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
    struct fan_pwm_tach *fan = attr->user_data;
    int len;

    if (fan->pwm_en_done)
        return 0;

    len = snprintf(buf, size, "%u\n", fan->pwm_enable);
    if (len > 0)
    {
        fan->pwm_en_done = 1;
    }
    return len;
}

static ssize_t pwm_en_attr_write(sysfs_attr_t attr, const void *buf, size_t size)
{
    struct fan_pwm_tach *fan = attr->user_data;
    unsigned long val;
    int ret;

    char tmp[64];
    size_t cp_sz = MIN(size, sizeof(tmp) - 1);
    memcpy(tmp, buf, cp_sz);
    tmp[cp_sz] = '\0';
    val = strtoul(buf, NULL, 0);
    if (val > 1) // only 0 / 1
        val = 1;

    fan->pwm_enable = (uint8_t)val;
    uint32_t pulse_ns = 0;
    if (fan->pwm_enable)
    {
        pulse_ns = (PWM_FAN_PERIOD_NS * (uint32_t)fan->pwm_value) / 255U;
    }

    ret = pwm_set(fan->pwm_dev, fan->pwm_channel,
                  PWM_FAN_PERIOD_NS, pulse_ns, PWM_POLARITY_NORMAL);
    if (ret < 0)
    {
        LOG_ERR("pwm enable set failed: %d", ret);
        return ret;
    }

    LOG_DBG("pwm enable write: ch=%d en=%lu", fan->pwm_channel, val);
    return size;
}

static struct sysfs_attribute_ops pwm_en_attr_ops = {
    .open = pwm_en_attr_open,
    .read = pwm_en_attr_read,
    .write = pwm_en_attr_write,
    .close = pwm_en_attr_close,
    .ioctl = NULL,
};

/* --- Init function: create sysfs nodes --- */

static int hwmon_fan_pwm_fs_init(void)
{
    char path[64];
    struct hwmon_fan_dev *dev;

    for (int i = 0; i < ARRAY_SIZE(hwmon_fan_devices); i++)
    {
        dev = &hwmon_fan_devices[i];

        if (snprintf(path, sizeof(path),
                     "/sys/class/hwmon/%s",
                     dev->label) >= sizeof(path))
        {
            LOG_ERR("hwmon fan path too long");
            break;
        }

        dev->node = sysfs_mkdir(path);
        if (!dev->node)
        {
            LOG_ERR("mkdir failed: %s", path);
            continue;
        }

        // for (int j = 0; j < dev->fan_dev_num; j++)
        // {
            struct fan_pwm_tach *fan = &dev->fan_dev[0];

            if (!device_is_ready(fan->cap_dev) || !device_is_ready(fan->pwm_dev))
            {
                LOG_ERR("CAP/PWM device not ready");
                continue;
            }

            /* fan1_input (tach reading from cap sensor) */
            snprintf(fan->fan_attr.name,
                     SYSFS_ATTRIBUTE_NAME_MAX,
#ifdef CONFIG_OPENBMC_ZEPHYR
                     "fan%d_input", i + 1);
#else
                     "fan%d_input", i);
#endif
            fan->fan_attr.user_data = fan;
            fan->fan_attr.ops = &fan_attr_ops;
            sysfs_add_attribute(dev->node, &fan->fan_attr);

            /* pwm1 (PWM output control) */
            snprintf(fan->pwm_attr.name,
                     SYSFS_ATTRIBUTE_NAME_MAX,
#ifdef CONFIG_OPENBMC_ZEPHYR
                     "pwm%d", i + 1);
#else
                     "pwm%d", i);
#endif
            fan->pwm_attr.user_data = fan;
            fan->pwm_attr.ops = &pwm_attr_ops;
            sysfs_add_attribute(dev->node, &fan->pwm_attr);

            snprintf(fan->pwm_en_attr.name,
                     SYSFS_ATTRIBUTE_NAME_MAX,
#ifdef CONFIG_OPENBMC_ZEPHYR
                     "pwm%d_enable", i + 1);
#else
                     "pwm%d_enable", i);
#endif
            fan->pwm_en_attr.user_data = fan;
            fan->pwm_en_attr.ops = &pwm_en_attr_ops;
            sysfs_add_attribute(dev->node, &fan->pwm_en_attr);
            // Initialize fan with 20% duty cycle output on startup, normal polarity,
            // full period defined by PWM_FAN_PERIOD_NS
            int ret = pwm_set(fan->pwm_dev, fan->pwm_channel,
                              PWM_FAN_PERIOD_NS, PWM_FAN_PERIOD_NS / 5, PWM_POLARITY_NORMAL);
            fan->pwm_value = 255 / 5;
        // }

        /* name attribute */
        snprintf(dev->name_attr.name,
                 SYSFS_ATTRIBUTE_NAME_MAX,
                 "name");
        dev->name_attr.ops = &name_attr_ops;
        dev->name_attr.user_data = dev;
        sysfs_add_attribute(dev->node, &dev->name_attr);

        /* device label attribute */
        snprintf(dev->label_attr.name,
                 SYSFS_ATTRIBUTE_NAME_MAX,
                 "label");
        dev->label_attr.ops = &dev_label_attr_ops;
        dev->label_attr.user_data = dev;
        sysfs_add_attribute(dev->node, &dev->label_attr);
    }

    return 0;
}
SYS_INIT(hwmon_fan_pwm_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
