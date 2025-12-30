#include <stdarg.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/fs/devfs.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <stdio.h>
#include <linux/ioctl.h>
#include <gpio/gpio_fs.h>

LOG_MODULE_REGISTER(gpio_fs, LOG_LEVEL_ERR);

#define DT_DRV_COMPAT               linkedsemi_lsqsh_gpio
#define GPIO_LINE_HANDLE_MAX                    (64)
#define GPIO_LINE_EVENT_MAX                     (64)

#define GPIO_LINE_EVENT_DATA_MAX                (16)

#define GPIOHANDLE_REQUEST_VALID_FLAGS \
                                    (GPIOHANDLE_REQUEST_INPUT | \
                                    GPIOHANDLE_REQUEST_OUTPUT | \
                                    GPIOHANDLE_REQUEST_ACTIVE_LOW | \
                                    GPIOHANDLE_REQUEST_OPEN_DRAIN | \
                                    GPIOHANDLE_REQUEST_OPEN_SOURCE)

#define GPIOEVENT_REQUEST_VALID_FLAGS \
                                    (GPIOEVENT_REQUEST_RISING_EDGE | \
                                    GPIOEVENT_REQUEST_FALLING_EDGE)

struct gpio_chip
{
    const struct device* dev;
    const char *label;
    const char **line_names;
    const uint32_t pin_count;
    uint32_t pin_mask;
    atomic_t ref_cnt;
};

struct gpio_line
{
    struct gpio_chip *chip;
    struct gpiohandle_request req;
    char name[8];
};

struct gpio_event
{
    struct gpio_chip *chip;
    struct gpioevent_request req;
    struct gpio_callback gpio_cb;
    struct k_poll_signal signal;
    struct ring_buf rb;
    struct gpioevent_data rb_buf[GPIO_LINE_EVENT_DATA_MAX];
    char name[8];
};

#define INIT_GPIO_LINE_NAMES_ARRAY(inst)                                       \
    static const char* gpio_line_names_##inst[] =                              \
        DT_INST_PROP_OR(inst, gpio_line_names, {NULL});

DT_INST_FOREACH_STATUS_OKAY(INIT_GPIO_LINE_NAMES_ARRAY)

#define GPIO_DEVICE_PROCESS(inst)                                               \
    {                                                                           \
        .dev =  DEVICE_DT_GET(DT_INST(inst, DT_DRV_COMPAT)),                    \
        .label = DT_INST_PROP(inst, label),                                     \
        .line_names = gpio_line_names_##inst,                                   \
        .pin_count = DT_INST_PROP(inst, ngpios),                                \
        .pin_mask = 0,                                                          \
        .ref_cnt = ATOMIC_INIT(0)                                               \
    },

static struct gpio_chip gpio_devices[] = {
    DT_INST_FOREACH_STATUS_OKAY(GPIO_DEVICE_PROCESS)};

static struct gpio_line gpio_line_handle[GPIO_LINE_HANDLE_MAX];
static struct gpio_event gpio_line_event[GPIO_LINE_EVENT_MAX];
static struct k_mutex lock;

static inline void gpio_lock()
{
    k_mutex_lock(&lock, K_FOREVER);
}

static inline void gpio_unlock()
{
    k_mutex_unlock(&lock);
}

static struct gpio_chip *gpio_chip_find(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_devices); i++)
    {
        if (strcmp(name, gpio_devices[i].label) == 0)
            return &gpio_devices[i];
    }

    return NULL;
}

static struct gpio_line *gpio_line_handle_find(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_handle); i++)
    {
        if (strcmp(name, gpio_line_handle[i].name) == 0)
            return &gpio_line_handle[i];
    }

    return NULL;
}

static struct gpio_line *gpio_line_handle_alloc(void)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_handle); i++)
    {
        if (gpio_line_handle[i].chip == NULL)
            return &gpio_line_handle[i];
    }

    return NULL;
}

static void gpio_line_handle_init(void)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_handle); i++)
    {
        snprintf(gpio_line_handle[i].name, 8, "line%d", i);
    }
}

static void gpio_line_handle_free(struct gpio_line *line_handle)
{
    line_handle->chip = NULL;
}

static struct gpio_event *gpio_line_event_find(const char *name)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_event); i++)
    {
        if (strcmp(name, gpio_line_event[i].name) == 0)
            return &gpio_line_event[i];
    }

    return NULL;
}

static struct gpio_event *gpio_line_event_alloc(void)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_event); i++)
    {
        if (gpio_line_event[i].chip == NULL)
            return &gpio_line_event[i];
    }

    return NULL;
}

static void gpio_line_event_init(void)
{
    for (int i = 0; i < ARRAY_SIZE(gpio_line_event); i++)
    {
        snprintf(gpio_line_event[i].name, 8, "event%d", i);
    }
}

static void gpio_line_event_free(struct gpio_event *line_event)
{
    line_event->chip = NULL;
}

static gpio_flags_t req_flag_2_gpio_flag(uint32_t flags)
{
    gpio_flags_t gpio_flag = 0;

    if (flags & GPIOHANDLE_REQUEST_INPUT)
        gpio_flag |= GPIO_INPUT;
    if (flags & GPIOHANDLE_REQUEST_OUTPUT)
        gpio_flag |= GPIO_OUTPUT;
    if (flags & GPIOHANDLE_REQUEST_ACTIVE_LOW)
        gpio_flag |= GPIO_OUTPUT_INIT_LOW;
    if (flags & GPIOHANDLE_REQUEST_OPEN_DRAIN)
        gpio_flag |= GPIO_OPEN_DRAIN;
    if (flags & GPIOHANDLE_REQUEST_OPEN_SOURCE)
        gpio_flag |= GPIO_OPEN_SOURCE;
    if (flags & GPIOHANDLE_REQUEST_BIAS_PULL_UP)
        gpio_flag |= GPIO_PULL_UP;
    if (flags & GPIOHANDLE_REQUEST_BIAS_PULL_DOWN)
        gpio_flag |= GPIO_PULL_DOWN;
    if (flags & GPIOHANDLE_REQUEST_BIAS_DISABLE)
        gpio_flag = GPIO_DISCONNECTED;

    return gpio_flag;
}

static int gpio_line_open(struct fs_file_t* zfp, const char* file_name, fs_mode_t mode)
{
    struct gpio_line *line_handle = gpio_line_handle_find(file_name);
    if (line_handle == NULL || line_handle->chip == NULL)
        return -EINVAL;

    struct gpio_chip *chip = line_handle->chip;
    uint32_t lflags = line_handle->req.flags;
    gpio_flags_t flags = req_flag_2_gpio_flag(lflags);
    uint32_t *lineoffsets = line_handle->req.lineoffsets;
    uint8_t *default_values = line_handle->req.default_values;

    gpio_lock();
    for (int i = 0; i < line_handle->req.lines; i++)
    {
        chip->pin_mask |= BIT(lineoffsets[i]);
        gpio_pin_configure(chip->dev, lineoffsets[i], flags);
        if (flags & GPIO_OUTPUT)
            gpio_pin_set(chip->dev, lineoffsets[i], default_values[i]);
    }

    gpio_unlock();
    zfp->filep = line_handle;

    return 0;
}

static int gpio_line_close(struct fs_file_t* zfp)
{
    struct gpio_line *line_handle = zfp->filep;
    uint32_t *lineoffsets = line_handle->req.lineoffsets;
    char path[32];
    snprintf(path, sizeof(path) - 1, "/dev/%s", line_handle->name);

    gpio_lock();
    /* disable pin and clear pin mask */
    for (int i = 0; i < line_handle->req.lines; i++)
    {
        line_handle->chip->pin_mask &= ~BIT(line_handle->req.lineoffsets[i]);
        gpio_pin_configure(line_handle->chip->dev, lineoffsets[i], GPIO_DISCONNECTED);
    }
    gpio_unlock();

    /* unregister path */
    devfs_unregister_anon(path);
    /* free line object */
    gpio_line_handle_free(line_handle);

    return 0;
}

static int gpio_line_ioctl(struct fs_file_t* zfp, unsigned long request, va_list args)
{
    struct gpio_line *line_handle = zfp->filep;

    switch (request)
    {
    case GPIOHANDLE_SET_LINE_VALUES_IOCTL:
        {
            struct gpiohandle_data *data = va_arg(args, struct gpiohandle_data *);
            uint32_t *lineoffsets = line_handle->req.lineoffsets;
            gpio_lock();
            for (int i = 0; i < line_handle->req.lines; i++)
            {
                gpio_pin_set(line_handle->chip->dev, lineoffsets[i], data->values[i]);
            }
            gpio_unlock();
        }
        break;
    case GPIOHANDLE_GET_LINE_VALUES_IOCTL:
        {
            uint32_t value = 0;
            struct gpiohandle_data *data = va_arg(args, struct gpiohandle_data *);
            uint32_t *lineoffsets = line_handle->req.lineoffsets;

            gpio_lock();
            gpio_port_get(line_handle->chip->dev, &value);
            gpio_unlock();

            for (int i = 0; i < line_handle->req.lines; i++)
            {
                uint32_t pin = lineoffsets[i];
                data->values[i] = (value & (gpio_port_pins_t)BIT(pin)) != 0 ? 1 : 0;
            }
        }
        break;
    case GPIOHANDLE_SET_CONFIG_IOCTL:
        {
            struct gpiohandle_config *config = va_arg(args, struct gpiohandle_config *);
            uint32_t lflags = config->flags;
            uint32_t *lineoffsets = line_handle->req.lineoffsets;
            gpio_flags_t flags = req_flag_2_gpio_flag(lflags);

            gpio_lock();
            for (int i = 0; i < line_handle->req.lines; i++)
            {
                gpio_pin_configure(line_handle->chip->dev, lineoffsets[i], flags);
                if (flags & GPIO_OUTPUT)
                    gpio_pin_set(line_handle->chip->dev, lineoffsets[i], config->default_values[i]);
            }
            gpio_unlock();
        }
        break;
    default:
        LOG_ERR("Unsupported line ioctl request: %lu", request);
        return -ENOTTY;
    }

    return 0;
}

static const struct fs_file_system_t gpio_line = {
    .open = gpio_line_open,
    .close = gpio_line_close,
    .ioctl = gpio_line_ioctl
};

static int gpio_line_handle_create(struct gpiohandle_request *req, struct gpio_chip *chip)
{
    if (req->lines == 0 || req->lines > chip->pin_count)
    {
        LOG_ERR("GPIO_GET_LINEHANDLE_IOCTL invalid lines=[%u] (must 1~%d)\n", req->lines, chip->pin_count);
        return -EINVAL;
    }

    for (int i = 0; i < req->lines; i++)
    {
        uint32_t offset = req->lineoffsets[i];
        if (offset >= chip->pin_count)
        {
            LOG_ERR( "GPIO_GET_LINEHANDLE_IOCTL i=[%d] offset=[%u] exceed max=[%d]\n", i, offset, chip->pin_count - 1);
            return -EINVAL;
        }
    }

    for (int i = 0; i < req->lines; i++)
    {
        uint32_t pin = req->lineoffsets[i];
        if (BIT(pin) & chip->pin_mask)
        {
            LOG_ERR( "GPIO_GET_LINEHANDLE_IOCTL pin [%d] is opened\n", pin);
            return -EINVAL;
        }
    }

    uint32_t lflags = req->flags;

    if (lflags & ~GPIOHANDLE_REQUEST_VALID_FLAGS)
        return -EINVAL;

    /*
    * Do not allow both INPUT & OUTPUT flags to be set as they are
    * contradictory.
    */
    if ((lflags & GPIOHANDLE_REQUEST_INPUT) &&
        (lflags & GPIOHANDLE_REQUEST_OUTPUT))
        return -EINVAL;

    /*
    * Do not allow OPEN_SOURCE & OPEN_DRAIN flags in a single request. If
    * the hardware actually supports enabling both at the same time the
    * electrical result would be disastrous.
    */
    if ((lflags & GPIOHANDLE_REQUEST_OPEN_DRAIN) &&
        (lflags & GPIOHANDLE_REQUEST_OPEN_SOURCE))
        return -EINVAL;

    /* OPEN_DRAIN and OPEN_SOURCE flags only make sense for output mode. */
    if (!(lflags & GPIOHANDLE_REQUEST_OUTPUT) &&
        ((lflags & GPIOHANDLE_REQUEST_OPEN_DRAIN) ||
        (lflags & GPIOHANDLE_REQUEST_OPEN_SOURCE)))
        return -EINVAL;

    gpio_lock();

    struct gpio_line *line_handle = gpio_line_handle_alloc();
    if (line_handle == NULL)
    {
        gpio_unlock();
        return -ENOMEM;
    }

    int res = 0;
    char path[32];

    snprintf(path, sizeof(path) - 1, "/dev/%s", line_handle->name);
    if ((res = !devfs_register_anon(path, &gpio_line)))
    {
        line_handle->chip = chip;
        line_handle->req = *req;

        req->fd = open(path, O_RDWR);
        if (req->fd < 0)
        {
            devfs_unregister_anon(path);
            gpio_line_handle_free(line_handle);
            res = -errno;
        }
        line_handle->req.fd = req->fd;
    }
    else
    {
        gpio_line_handle_free(line_handle);
    }

    gpio_unlock();

    return res;
}

static void gpio_irq(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct gpio_event *event = CONTAINER_OF(cb, struct gpio_event, gpio_cb);
    struct gpioevent_data event_data;
    uint32_t eflags = event->req.eventflags;
    uint32_t pin = event->req.lineoffset;

    if (pins & BIT(pin))
    {
        event_data.timestamp = k_uptime_get() * NSEC_PER_MSEC;

        if ((eflags & GPIOEVENT_REQUEST_RISING_EDGE) && (eflags & GPIOEVENT_REQUEST_FALLING_EDGE))
        {
            bool pin_val = gpio_pin_get(event->chip->dev, pin);
            if (pin_val)
                event_data.id = GPIOEVENT_EVENT_RISING_EDGE;
            else
                event_data.id = GPIOEVENT_EVENT_FALLING_EDGE;
        }
        else if (eflags & GPIOEVENT_REQUEST_RISING_EDGE)
        {
            event_data.id = GPIOEVENT_EVENT_RISING_EDGE;
        }
        else if (eflags & GPIOEVENT_REQUEST_FALLING_EDGE)
        {
            event_data.id = GPIOEVENT_EVENT_FALLING_EDGE;
        }

        uint32_t w_size = ring_buf_put(&event->rb, (void *)&event_data, sizeof(event_data));
        if (w_size != sizeof(event_data))
            __ASSERT(0, "chip: %s, pin: %d, buffer full!!", event->chip->label, pin);

        k_poll_signal_raise(&event->signal, 1);
    }
}

static int gpio_event_open(struct fs_file_t* zfp, const char* file_name, fs_mode_t mode)
{
    struct gpio_event *event = gpio_line_event_find(file_name);
    if (event == NULL || event->chip == NULL)
        return -EINVAL;

    int ret = 0;
    uint32_t pin = event->req.lineoffset;
    uint32_t eflags = event->req.eventflags;
    gpio_flags_t int_flag = 0;

    if (eflags == GPIOEVENT_REQUEST_BOTH_EDGES)
        int_flag = GPIO_INT_EDGE_BOTH;
    else if (eflags & GPIOEVENT_REQUEST_RISING_EDGE)
        int_flag = GPIO_INT_EDGE_RISING;
    else if (eflags & GPIOEVENT_REQUEST_FALLING_EDGE)
        int_flag = GPIO_INT_EDGE_FALLING;

    ring_buf_init(&event->rb, sizeof(event->rb_buf), (uint8_t *)&event->rb_buf);
    k_poll_signal_init(&event->signal);
    gpio_init_callback(&event->gpio_cb, gpio_irq, BIT(pin));
    gpio_lock();

    ret = gpio_pin_configure(event->chip->dev, pin, GPIO_INPUT);
    if (ret != 0)
    {
        gpio_unlock();
        LOG_ERR("Error %d: failed to configure pin %d\n", ret, pin);
        return ret;
    }

    ret = gpio_pin_interrupt_configure(event->chip->dev, pin, int_flag);
    if (ret != 0)
    {
        gpio_unlock();
        LOG_ERR("Error %d: failed to configure interrupt pin %d\n", ret, pin);
        return ret;
    }

    gpio_add_callback(event->chip->dev, &event->gpio_cb);
    event->chip->pin_mask |= BIT(pin);

    gpio_unlock();
    zfp->filep = event;

    return 0;
}

static int gpio_event_close(struct fs_file_t* zfp)
{
    char path[32];
    struct gpio_event *event = zfp->filep;

    gpio_lock();
    event->chip->pin_mask &= ~BIT(event->req.lineoffset);
    /* irq disable */
    gpio_pin_interrupt_configure(event->chip->dev, event->req.lineoffset, GPIO_INT_DISABLE);
    gpio_remove_callback(event->chip->dev, &event->gpio_cb);
    gpio_unlock();

    /* internal struct reset */
    k_poll_signal_reset(&event->signal);
    ring_buf_reset(&event->rb);
    /* free event object */
    gpio_line_event_free(event);
    /* unregister path */
    snprintf(path, sizeof(path) - 1, "/dev/%s", event->name);
    devfs_unregister_anon(path);

    return 0;
}

static ssize_t gpio_event_read(struct fs_file_t *filp, void *dest, size_t nbytes)
{
    struct gpio_event *event = filp->filep;
    return ring_buf_get(&event->rb, dest, nbytes);
}

static int gpio_event_ioctl(struct fs_file_t* zfp, unsigned long request, va_list args)
{
    switch (request)
    {
    case GPIOHANDLE_GET_LINE_VALUES_IOCTL:
    {
        struct gpiohandle_data* data = va_arg(args, struct gpiohandle_data*);
        struct gpio_event *event = zfp->filep;
        gpio_lock();
        data->values[0] = gpio_pin_get(event->chip->dev, event->req.lineoffset);
        gpio_unlock();
    }
    break;
    case ZFD_IOCTL_POLL_PREPARE:
    {
        struct zvfs_pollfd* pfd = va_arg(args, struct zvfs_pollfd*);
        struct k_poll_event** pev = va_arg(args, struct k_poll_event**);
        struct k_poll_event* pev_end = va_arg(args, struct k_poll_event*);
        struct gpio_event *event = zfp->filep;
        if (pfd->events & ZVFS_POLLIN)
        {
            if (*pev == pev_end)
            {
                LOG_ERR("ZFD_IOCTL_POLL_PREPARE: no free poll event slot");
                return -ENOMEM;
            }

            (*pev)->obj = &event->signal;
            (*pev)->type = K_POLL_TYPE_SIGNAL;
            (*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
            (*pev)->state = K_POLL_STATE_NOT_READY;
            (*pev)++;
        }
    }
    break;
    case ZFD_IOCTL_POLL_UPDATE:
    {
        struct zvfs_pollfd* pfd = va_arg(args, struct zvfs_pollfd*);
        struct k_poll_event** pev = va_arg(args, struct k_poll_event**);
        struct gpio_event *event = zfp->filep;

        if (pfd->events & ZVFS_POLLIN)
        {
            size_t buf_used = ring_buf_size_get(&event->rb);
            if (buf_used >= sizeof(struct gpioevent_data))
            {
                pfd->revents = ZVFS_POLLIN;
                k_poll_signal_reset(&event->signal);
                (*pev)->state = K_POLL_STATE_NOT_READY;
            }
            (*pev)++;
        }
    }
    break;
    default:
        LOG_ERR("Unsupported event ioctl request: %lu", request);
        return -ENOTTY;
    }

    return 0;
}

static const struct fs_file_system_t gpio_event_ops = {
    .open = gpio_event_open,
    .close = gpio_event_close,
    .read = gpio_event_read,
    .ioctl = gpio_event_ioctl
};

static int gpio_line_event_create(struct gpioevent_request *req, struct gpio_chip *chip)
{
    uint32_t lflags = req->handleflags;
    uint32_t eflags = req->eventflags;
    uint32_t pin = req->lineoffset;

    /* check arg */
    if (pin >= chip->pin_count)
    {
        LOG_ERR( "GPIO_GET_LINEEVENT_IOCTL offset=[%u] exceed max=[%d]\n", pin, chip->pin_count - 1);
        return -EINVAL;
    }

    if (BIT(pin) & chip->pin_mask)
    {
        LOG_ERR( "GPIO_GET_LINEEVENT_IOCTL pin [%d] is opened\n", pin);
        return -EINVAL;
    }

    /* Return an error if a unknown flag is set */
    if ((lflags & ~GPIOHANDLE_REQUEST_VALID_FLAGS) ||
        (eflags & ~GPIOEVENT_REQUEST_VALID_FLAGS))
        return -EINVAL;

    /* This is just wrong: we don't look for events on output lines */
    if ((lflags & GPIOHANDLE_REQUEST_OUTPUT) ||
        (lflags & GPIOHANDLE_REQUEST_OPEN_DRAIN) ||
        (lflags & GPIOHANDLE_REQUEST_OPEN_SOURCE))
        return -EINVAL;

    /* Only one bias flag can be set. */
    if (((lflags & GPIOHANDLE_REQUEST_BIAS_DISABLE) &&
            (lflags & (GPIOHANDLE_REQUEST_BIAS_PULL_DOWN |
            GPIOHANDLE_REQUEST_BIAS_PULL_UP))) ||
        ((lflags & GPIOHANDLE_REQUEST_BIAS_PULL_DOWN) &&
            (lflags & GPIOHANDLE_REQUEST_BIAS_PULL_UP)))
        return -EINVAL;

    gpio_lock();
    struct gpio_event *line_event = gpio_line_event_alloc();

    if (line_event == NULL)
    {
        gpio_unlock();
        return -ENOMEM;
    }

    int res = 0;
    char path[32];

    snprintf(path, sizeof(path) - 1, "/dev/%s", line_event->name);
    if ((res = !devfs_register_anon(path, &gpio_event_ops)))
    {
        line_event->chip = chip;
        line_event->req = *req;

        req->fd = open(path, O_RDWR);
        if (req->fd < 0)
        {
            devfs_unregister_anon(path);
            gpio_line_event_free(line_event);
            res = -errno;
        }
        line_event->req.fd = req->fd;
    }
    else
    {
        gpio_line_event_free(line_event);
    }

    gpio_unlock();

    return res;
}

static int gpio_chip_open(struct fs_file_t* zfp, const char* file_name, fs_mode_t mode)
{
    struct gpio_chip *chip = gpio_chip_find(file_name);

    if (chip == NULL)
        return -EINVAL;

    if (!device_is_ready(chip->dev))
    {
        LOG_ERR("GPIO device not ready!\n");
        return -ENODEV;
    }

    zfp->filep = chip;
    atomic_inc(&chip->ref_cnt);

    return 0;
}

static int gpio_chip_close(struct fs_file_t* zfp)
{
    struct gpio_chip *chip = zfp->filep;
    atomic_dec(&chip->ref_cnt);

    if (chip->ref_cnt == 0)
    {
        zfp->filep = NULL;
    }

    return 0;
}

static int gpio_chip_ioctl(struct fs_file_t* zfp, unsigned long request, va_list args)
{
    switch (request)
    {
    case GPIO_GET_CHIPINFO_IOCTL:
    {
        struct gpiochip_info *chip_info = va_arg(args, struct gpiochip_info *);
        struct gpio_chip *chip = zfp->filep;
        chip_info->lines = chip->pin_count;
        strncpy(chip_info->name, chip->dev->name, sizeof(chip_info->name) - 1);
        strncpy(chip_info->label, chip->label, sizeof(chip_info->label) - 1);
    }
    break;
    case GPIO_GET_LINEINFO_IOCTL:
    {
        struct gpioline_info *line_info = va_arg(args, struct gpioline_info *);
        struct gpio_chip *chip = zfp->filep;
        uint32_t offset = line_info->line_offset;
        if (offset >= chip->pin_count)
        {
            LOG_ERR("GPIO_GET_LINEINFO_IOCTL Offset %d exceeds pin count %d", offset, chip->pin_count);
            return -EINVAL;
        }

        if (chip->line_names[offset])
        {
            strncpy(line_info->name, chip->line_names[offset], sizeof(line_info->name) - 1);
        }
        else
        {
            line_info->name[0] = '\0';
        }
        strncpy(line_info->consumer, "zephyr-gpio", sizeof(line_info->consumer) - 1);
        line_info->flags = GPIOLINE_FLAG_KERNEL;
    }
    break;
    case GPIO_GET_LINEHANDLE_IOCTL:
        return gpio_line_handle_create(va_arg(args, struct gpiohandle_request *), zfp->filep);
    case GPIO_GET_LINEEVENT_IOCTL:
        return gpio_line_event_create(va_arg(args, struct gpioevent_request *), zfp->filep);
    case F_GETFL:
        return O_RDWR;
    case F_SETFL:
        return 0;
    default:
        LOG_ERR("Unsupported chip ioctl request: %lu", request);
        return -ENOTTY;
    }
    return 0;
}

static const struct fs_file_system_t gpio_chip = {
    .open = gpio_chip_open,
    .close = gpio_chip_close,
    .ioctl = gpio_chip_ioctl
};

int gpio_fs_init(void)
{
    char name[32];

    for (int i = 0; i < ARRAY_SIZE(gpio_devices); i++)
    {
        snprintf(name, sizeof(name) - 1, "/dev/%s", gpio_devices[i].label);
        devfs_register(name, &gpio_chip);
    }
    gpio_line_handle_init();
    gpio_line_event_init();

    return 0;
}
SYS_INIT(gpio_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);