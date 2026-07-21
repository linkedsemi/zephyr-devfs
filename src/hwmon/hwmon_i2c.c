/*
 * linkedsemi hwmon class 5 (i2c / sensor client) driver.
 *
 * DESIGN PRINCIPLE: this driver is GENERIC and has ZERO knowledge of any
 * sensor driver's internals. There is no per-driver registration and no
 * shared header: hwmon never sees a channel table, a rail selector, or a bus
 * register.
 *
 * Which hwmon ABI files to expose is declared in devicetree, on each class-5
 * node, via the `meas` string-array property, e.g.:
 *
 *     meas = "in1_input:vin1", "in3_input:vout1",
 *            "curr2_input:iout1", "power2_input:pout1",
 *            "temp1_input", "fan1_input";
 *
 * Each entry is "<chan><n>_input" with an OPTIONAL ":<label>" suffix. The part
 * before ':' is the value file; the OPTIONAL part after ':' is the content of a
 * companion "<chan><n>_label" file (input->label). The label lets dbus-sensors
 * distinguish vin/vout, iin/iout, pin/pout; if omitted, no label file is made
 * and dbus-sensors falls back to the generic "output" property for that rail.
 *
 * At read time hwmon parses the filename itself: the prefix selects the
 * sensor_channel (in->VOLTAGE, curr->CURRENT, power->POWER, temp->AMBIENT_TEMP,
 * fan->RPM) and the number (minus 1) is passed as val2 -- the rail /
 * sub-channel selector that the underlying driver's channel_get already
 * understands. hwmon then calls the standard sensor API directly:
 *
 *     sensor_sample_fetch(dev);             // cache, one transaction / window
 *     sensor_channel_get(dev, chan, &val);  // val.val2 = number - 1
 *
 * This is the ONLY coupling between hwmon and the sensor driver, and it is the
 * standard Zephyr sensor API + the hwmon ABI naming convention -- no custom
 * contract, no header to include, nothing to register. Adding a new chip =
 * write its sensor driver (which already implements channel_get with val2 as
 * its rail selector) + list its filenames in DT. hwmon_i2c.c is never touched.
 *
 * The actual bus protocol (PMBus / raw i2c / SMBus), the multi-rail mapping,
 * and all value scaling are owned entirely by the sensor driver.
 */

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <zephyr/kernel.h>
#include <zephyr/fs/sysfs.h>
#include <zephyr/sys/fdtable.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_i2c, LOG_LEVEL_INF);

/* ---- sensor sub-type (class 5 only) ---- */
enum hwmon_sensor_type {
	HWMON_TYPE_NONE = 0,   /* legacy: no sensor-type property (deprecated) */
	HWMON_TYPE_TEMP,
	HWMON_TYPE_PSU,
	/* NVMe is intentionally NOT a hwmon type: in native OpenBMC the NVMe
	 * drive temperature is read via NVMe-MI by the dedicated NVMeSensor
	 * daemon, not through hwmon tempX_input files. */
};

/* ---- hwmon ABI file lists come from devicetree ----
 * Each class-5 node lists its files via the `meas` string-array property,
 * e.g. "in3_input:vout1". The whole property is captured directly as a brace
 * initializer via DT_INST_PROP(); this preserves the ':' separator (Zephyr's
 * per-element string-array tokens mangle ':' to '_', so per-element access is
 * unsuitable here). See HWMON_DEVICE_ENTRY / HWMON_FILES below. */

#define HWMON_MAX_CHANNELS 16

/* ---- i2c client (controller + address) for bus_addr ---- */
struct hwmon_i2c_client {
	const struct device *bus;
	uint32_t addr;
};

/* ---- per-instance channel (runtime) ---- */
struct hwmon_channel {
	const char *filename;/* the "<chan><n>_input" file (label stripped) */
	struct sysfs_attribute attr;
	int ppos;
	struct k_poll_signal poll_signal;/* for poll/async-read support (dbus-sensors) */
	struct hwmon_dev *dev;/* back pointer */

	/* optional "<chan><n>_label" file: created only when the DT `meas`
	 * entry carried a ":label" suffix (e.g. "in3_input:vout1"). The label
	 * lets dbus-sensors tell vin/vout, iin/iout, pin/pout apart; without it
	 * every rail falls back to the generic "output" property. */
	const char *label;   /* label file content (e.g. "vout1"), or NULL */
	struct sysfs_attribute label_attr;
	int label_ppos;
	bool has_label;
};

struct hwmon_dev {
	const char *label;
	const char *sensor_type_str;/* from DT `sensor-type`, may be NULL */
	enum hwmon_sensor_type type;/* resolved at init */
	const char *name_str;       /* value for the `name` file (from sensor-type) */
	const struct device *sensor_dev;/* sensor API device (sensor-type only) */
	struct hwmon_i2c_client client;/* for bus_addr */
	const char * const *files;  /* hwmon ABI filenames (from DT `meas`) */
	size_t file_count;
	size_t channel_num;         /* MIN(file_count, HWMON_MAX_CHANNELS) */
	struct hwmon_channel channels[HWMON_MAX_CHANNELS];
	int64_t last_fetch_ms;
	struct sysfs_node *node;
	struct sysfs_attribute name_attr;
	struct sysfs_attribute bus_addr_attr;
	int name_ppos;
	int bus_ppos;
};

/* ---- device table ----
 * Sensor nodes: `hwmon-ins = <&sensor_dev N>` -> sensor device; 
 * The hwmon ABI file list comes from the DT `meas` property. Legacy nodes
 * (no sensor-type) are deprecated and skipped at runtime. */
/* `meas` file list as a compound literal. COND_CODE_1 keeps it a plain
 * compound literal `(const char * const[]){ ... }`; do NOT use
 * DT_INST_PROP_OR here -- its `({ })` wrapping would create a nested
 * statement-expression, which is illegal inside an initializer at file
 * scope. */
#define HWMON_FILES(inst)                                                  \
	COND_CODE_1(                                                       \
		DT_INST_NODE_HAS_PROP(inst, meas),                         \
		((const char * const[])DT_INST_PROP(inst, meas)),          \
		((const char * const[]){NULL})                             \
	)

/* ---- class filter (mirrors hwmon_adc.c) ----
 * hwmon_i2c.c owns only class-5 (i2c / sensor-client) instances. Other
 * classes have their own hwmon_xxx.c handlers and are skipped here: for them
 * `_HWMON_CLASS_<n>` is left undefined, so Zephyr's COND_CODE_1 selects the
 * empty `()` branch and emits nothing -- exactly how hwmon_adc.c skips the
 * class-5 node (it only defines `_HWMON_CLASS_2`). */
#define _HWMON_CLASS_5                  1
#define HWMON_CLASS_TO_FLAG_RAW(x)      _HWMON_CLASS_##x
#define HWMON_CLASS_TO_FLAG(x)          HWMON_CLASS_TO_FLAG_RAW(x)

/* class-5 sensor entry: a plain `{ ... }` initializer (legal at file scope --
 * NOT a GNU `({ })` statement-expression). Per-field values that depend on
 * `sensor-type` are selected with COND_CODE_1, which wraps the chosen value in
 * `(...)`; that is fine for a value initializer (e.g. `.sensor_dev = (NULL)`).
 * The trailing comma lets it compose with DT_FOREACH_OKAY_INST's `fn(0) fn(1)`
 * expansion (which inserts no comma of its own). */
#define HWMON_SENSOR_ENTRY(inst)                                         \
	{                                                                  \
		.label = DT_INST_PROP(inst, label),                         \
		.sensor_type_str = COND_CODE_1(                            \
			DT_INST_NODE_HAS_PROP(inst, sensor_type),         \
			(DT_INST_PROP(inst, sensor_type)),                \
			(NULL)),                                            \
		.sensor_dev = COND_CODE_1(                               \
			DT_INST_NODE_HAS_PROP(inst, sensor_type),         \
			(DEVICE_DT_GET(DT_INST_PHANDLE_BY_IDX(            \
				inst, hwmon_ins, 0))),                    \
			(NULL)),                                            \
		.client = {                                              \
			.bus = COND_CODE_1(                             \
				DT_INST_NODE_HAS_PROP(inst, sensor_type), \
				(DEVICE_DT_GET(DT_BUS(DT_INST_PHANDLE_BY_IDX( \
					inst, hwmon_ins, 0)))),          \
				(NULL)),                                \
			.addr = COND_CODE_1(                            \
				DT_INST_NODE_HAS_PROP(inst, sensor_type), \
				(DT_REG_ADDR(DT_INST_PHANDLE_BY_IDX(     \
					inst, hwmon_ins, 0))),            \
				(0)),                                  \
		},                                                         \
		.files = HWMON_FILES(inst),                               \
		.file_count = DT_INST_PROP_LEN_OR(inst, meas, 0),          \
	},

/* class-5 -> entry; any other class -> nothing (its `_HWMON_CLASS_<n>` is
 * undefined -> COND_CODE_1's `()` branch). Mirrors hwmon_adc.c: the entry
 * body is wrapped as a GNU statement-expression `({ ... },)` via
 * (HWMON_SENSOR_ENTRY(inst)) -- legal as an array element in this toolchain,
 * same as hwmon_adc.c. The trailing comma composes with
 * DT_INST_FOREACH_STATUS_OKAY's no-comma `fn(0) fn(1)` expansion. */
#define HWMON_DEVICE_ENTRY(inst)                                          \
	COND_CODE_1(                                                     \
		HWMON_CLASS_TO_FLAG(DT_INST_PROP(inst, class)),          \
		(HWMON_SENSOR_ENTRY(inst)),                              \
		()                                                       \
	)

static struct hwmon_dev hwmon_devices[] = {
	DT_INST_FOREACH_STATUS_OKAY(HWMON_DEVICE_ENTRY)
};

/* ---- resolve sensor-type string -> enum (runtime) ---- */
static enum hwmon_sensor_type hwmon_type_from_str(const char *s)
{
	if (!s) {
		return HWMON_TYPE_NONE;
	}
	if (!strcmp(s, "temp")) {
		return HWMON_TYPE_TEMP;
	}
	if (!strcmp(s, "psu")) {
		return HWMON_TYPE_PSU;
	}
	return HWMON_TYPE_NONE;
}

/* name file content exposed at /sys/class/hwmon/hwmonX/name. */
static const char *hwmon_name_for_type(enum hwmon_sensor_type t)
{
	switch (t) {
	case HWMON_TYPE_TEMP:
		return "temp";
	case HWMON_TYPE_PSU:
		return "pmbus";
	default:
		return "hwmon";
	}
}

/* derive i2c bus number from controller device name (e.g. "i2c-1" -> 1). */
static int hwmon_i2c_bus_num(const struct device *bus)
{
	const char *n = bus ? bus->name : "";
	int len = (int)strlen(n);
	int end = len;

	/* skip trailing non-digits */
	while (end > 0 && !isdigit((unsigned char)n[end - 1])) {
		end--;
	}
	if (end == 0) {
		return 0;
	}
	/* walk back to the start of this digit run */
	int start = end;

	while (start > 0 && isdigit((unsigned char)n[start - 1])) {
		start--;
	}
	return atoi(&n[start]);
}

/* ---- sensor API fetch with short-lived cache ----
 * One sample_fetch per read window avoids N i2c/SMBus transactions when
 * dbus-sensors polls all channels of one device. */
#define HWMON_CACHE_MS 5000

static int hwmon_ensure_fresh(struct hwmon_dev *dev)
{
	int64_t now = k_uptime_get();

	if (dev->last_fetch_ms != 0 &&
	    (now - dev->last_fetch_ms) < HWMON_CACHE_MS) {
		return 0;
	}
	if (!device_is_ready(dev->sensor_dev)) {
		LOG_ERR("%s: sensor device not ready", dev->label);
		return -ENODEV;
	}

	int ret = sensor_sample_fetch(dev->sensor_dev);

	if (ret < 0) {
		LOG_ERR("%s: sensor_sample_fetch failed (%d)", dev->label, ret);
		return ret;
	}
	dev->last_fetch_ms = now;
	return 0;
}

/* Parse a hwmon ABI filename into the sensor_channel (from the prefix) and
 * the 1-based index (the number). hwmon passes (index - 1) as val2 -- the
 * rail/sub-channel selector the driver's channel_get expects. Returns -EINVAL
 * for names hwmon does not understand. */
static int hwmon_chan_from_name(const char *name, enum sensor_channel *chan,
				int *num)
{
	unsigned int n;
	char prefix[8];
	enum sensor_channel c;

	/* e.g. "in3_input", "curr2_input", "power1_input", "temp1_input",
	 * "fan1_input": a non-digit prefix, then a number, then "_input". */
	if (sscanf(name, "%7[^0-9]%u_input", prefix, &n) < 2) {
		return -EINVAL;
	}

	if (!strcmp(prefix, "in")) {
		c = SENSOR_CHAN_VOLTAGE;
	} else if (!strcmp(prefix, "curr")) {
		c = SENSOR_CHAN_CURRENT;
	} else if (!strcmp(prefix, "power")) {
		c = SENSOR_CHAN_POWER;
	} else if (!strcmp(prefix, "temp")) {
		c = SENSOR_CHAN_AMBIENT_TEMP;
	} else if (!strcmp(prefix, "fan")) {
		c = SENSOR_CHAN_RPM;
	} else {
		return -EINVAL;
	}

	*chan = c;
	*num = (int)n;
	return 0;
}

/* ---- per-device channel setup from the DT `meas` file list ---- */
/* forward declarations of the sysfs ops structs (defined below) */
static struct sysfs_attribute_ops chan_attr_ops;
static struct sysfs_attribute_ops label_attr_ops;
static struct sysfs_attribute_ops name_attr_ops;
static struct sysfs_attribute_ops bus_addr_ops;

static void hwmon_dev_setup(struct hwmon_dev *dev)
{
	/* The file list comes straight from DT (`meas`); hwmon has no per-driver
	 * table to look up. `name` is derived from sensor-type.
	 *
	 * Each `meas` entry is "<chan><n>_input" with an OPTIONAL ":<label>"
	 * suffix, e.g. "in3_input:vout1". The part before ':' is the value file
	 * (the driver read path); the part after ':' becomes the content of a
	 * companion "<chan><n>_label" file. */
	dev->name_str = hwmon_name_for_type(dev->type);
	dev->channel_num = MIN(dev->file_count, HWMON_MAX_CHANNELS);
	dev->last_fetch_ms = 0;

	for (size_t i = 0; i < dev->channel_num; i++) {
		struct hwmon_channel *ch = &dev->channels[i];
		const char *entry = dev->files[i];
		const char *colon = strchr(entry, ':');
		int fn_len = colon ? (int)(colon - entry) : (int)strlen(entry);

		ch->dev = dev;
		ch->ppos = 0;
		k_poll_signal_init(&ch->poll_signal);

		/* value file: copy just the "<chan><n>_input" part (before ':') */
		snprintf(ch->attr.name, SYSFS_ATTRIBUTE_NAME_MAX, "%.*s",
			 fn_len, entry);
		ch->filename = ch->attr.name;
		ch->attr.user_data = ch;
		ch->attr.ops = &chan_attr_ops;

		/* optional label file: name = value name with "input"->"label",
		 * content = the DT-supplied text after ':'. */
		ch->has_label = false;
		ch->label = NULL;
		ch->label_ppos = 0;
		if (colon && colon[1] != '\0') {
			snprintf(ch->label_attr.name, SYSFS_ATTRIBUTE_NAME_MAX,
				 "%s", ch->filename);

			char *p = strstr(ch->label_attr.name, "input");

			if (p) {
				memcpy(p, "label", 5);/* same length */
				ch->label = colon + 1;/* points into DT string */
				ch->label_attr.user_data = ch;
				ch->label_attr.ops = &label_attr_ops;
				ch->has_label = true;
			}
		}
	}
}

/* ---- channel attribute ops ---- */
static int chan_attr_open(sysfs_attr_t attr)
{
	struct hwmon_channel *ch = attr->user_data;

	ch->ppos = 0;
	return 0;
}

static ssize_t chan_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_channel *ch = attr->user_data;

	if (ch->ppos > 0) {
		return 0;
	}

	struct hwmon_dev *dev = ch->dev;

	if (hwmon_ensure_fresh(dev) < 0) {
		return -EIO;
	}

	/* Parse the filename (e.g. "in3_input") into the sensor_channel (from
	 * the prefix) and the 1-based number. val2 = number - 1 is the rail /
	 * sub-channel selector the driver's channel_get already understands, so
	 * hwmon needs no knowledge of rails. */
	enum sensor_channel chan;
	int num;

	if (hwmon_chan_from_name(ch->filename, &chan, &num) < 0) {
		LOG_DBG("%s: unrecognised hwmon file %s", dev->label,
			ch->filename);
		return -EIO;
	}

	struct sensor_value val = { .val2 = num - 1 };
	int ret = sensor_channel_get(dev->sensor_dev, chan, &val);

	if (ret < 0) {
		LOG_DBG("%s: %s read failed (%d)", dev->label, ch->filename, ret);
		return -EIO;
	}

	/* driver returns the value already in the hwmon ABI unit (val1) */
	int len = snprintf(buf, size, "%d\n", val.val1);

	if (len > 0) {
		ch->ppos = len;
	}
	return len;
}

static int chan_attr_close(sysfs_attr_t attr)
{
	struct hwmon_channel *ch = attr->user_data;

	ch->ppos = 0;
	return 0;
}

/* ---- poll/async-read support ----
 * dbus-sensors (HwmonTempSensor) reads tempN_input via boost::asio
 * async_read_some, which needs fcntl (F_GETFL/F_SETFL) + poll
 * (ZFD_IOCTL_POLL_*). Without an ioctl handler the async read fails ->
 * Value=nan. The value is produced on demand in chan_attr_read, so the fd
 * is readable whenever ppos == 0. Mirror hwmon_adc.c: raise a k_poll_signal
 * in POLL_PREPARE when ppos == 0. With HwmonTempSensor reopening the fd
 * each poll, ppos is 0 every cycle -> always readable. */
static int chan_poll_prepare(struct hwmon_channel *ch,
			     struct zvfs_pollfd *pfd,
			     struct k_poll_event **pev,
			     struct k_poll_event *pev_end)
{
	if (pfd->events & ZVFS_POLLIN) {
		if (*pev >= pev_end) {
			return -ENOMEM;
		}
		(*pev)->obj = &ch->poll_signal;
		(*pev)->type = K_POLL_TYPE_SIGNAL;
		(*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
		(*pev)->state = K_POLL_STATE_NOT_READY;
		(*pev)++;
		if (ch->ppos == 0) {
			k_poll_signal_raise(&ch->poll_signal, 0);
		}
	}
	return 0;
}

static int chan_poll_update(struct hwmon_channel *ch,
			    struct zvfs_pollfd *pfd,
			    struct k_poll_event **pev)
{
	if (pfd->events & ZVFS_POLLIN) {
		if ((*pev)->state != K_POLL_STATE_NOT_READY && (ch->ppos == 0)) {
			pfd->revents |= ZVFS_POLLIN;
		}
		(*pev)++;
	}
	return 0;
}

static int chan_attr_ioctl(sysfs_attr_t attr, unsigned long request,
			   va_list args)
{
	struct hwmon_channel *ch = attr->user_data;

	if (!ch) {
		return -EINVAL;
	}

	switch (request) {
	case F_GETFL:
		return O_RDWR;

	case F_SETFL:
		return 0;

	case ZFD_IOCTL_POLL_PREPARE: {
		struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);
		return chan_poll_prepare(ch, pfd, pev, pev_end);
	}

	case ZFD_IOCTL_POLL_UPDATE: {
		struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
		struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
		return chan_poll_update(ch, pfd, pev);
	}

	case ZFD_IOCTL_POLL_OFFLOAD:
		k_poll_signal_reset(&ch->poll_signal);
		return 0;

	case ZFD_IOCTL_SET_LOCK:
		return 0;

	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static struct sysfs_attribute_ops chan_attr_ops = {
	.open = chan_attr_open,
	.read = chan_attr_read,
	.write = NULL,
	.close = chan_attr_close,
	.ioctl = chan_attr_ioctl,
};

/* ---- label attribute ops ----
 * A static string file (e.g. in3_label -> "vout1") so dbus-sensors can map
 * the rail to its proper PSU property (Input/Output Voltage, etc). */
static int label_attr_open(sysfs_attr_t attr)
{
	struct hwmon_channel *ch = attr->user_data;

	ch->label_ppos = 0;
	return 0;
}

static ssize_t label_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_channel *ch = attr->user_data;

	if (ch->label_ppos > 0) {
		return 0;
	}

	int len = snprintf(buf, size, "%s\n", ch->label);

	if (len > 0) {
		ch->label_ppos = len;
	}
	return len;
}

static int label_attr_close(sysfs_attr_t attr)
{
	struct hwmon_channel *ch = attr->user_data;

	ch->label_ppos = 0;
	return 0;
}

static struct sysfs_attribute_ops label_attr_ops = {
	.open = label_attr_open,
	.read = label_attr_read,
	.write = NULL,
	.close = label_attr_close,
};

/* ---- name attribute ops ---- */
static int name_attr_open(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->name_ppos = 0;
	return 0;
}

static ssize_t name_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_dev *dev = attr->user_data;

	if (dev->name_ppos > 0) {
		return 0;
	}

	int len = snprintf(buf, size, "%s\n", dev->name_str);

	if (len > 0) {
		dev->name_ppos = len;
	}
	return len;
}

static int name_attr_close(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->name_ppos = 0;
	return 0;
}

static struct sysfs_attribute_ops name_attr_ops = {
	.open = name_attr_open,
	.read = name_attr_read,
	.write = NULL,
	.close = name_attr_close,
};

/* ---- bus_addr attribute ops (Zephyr pairing key for dbus-sensors) ---- */
static int bus_addr_open(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->bus_ppos = 0;
	return 0;
}

static ssize_t bus_addr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_dev *dev = attr->user_data;

	if (dev->bus_ppos > 0) {
		return 0;
	}

	int bus = hwmon_i2c_bus_num(dev->client.bus);
	int len = snprintf(buf, size, "%u-%04x\n", bus, dev->client.addr);

	if (len > 0) {
		dev->bus_ppos = len;
	}
	return len;
}

static int bus_addr_close(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->bus_ppos = 0;
	return 0;
}

static struct sysfs_attribute_ops bus_addr_ops = {
	.open = bus_addr_open,
	.read = bus_addr_read,
	.write = NULL,
	.close = bus_addr_close,
};

/* ---- fs init ---- */
static int hwmon_fs_init(void)
{
	char path[64];

	for (size_t i = 0; i < ARRAY_SIZE(hwmon_devices); i++) {
		struct hwmon_dev *dev = &hwmon_devices[i];

		dev->type = hwmon_type_from_str(dev->sensor_type_str);

		/* Nodes with a missing or unrecognized `sensor-type` are not
		 * supported: the generic driver only talks to sensor-API devices
		 * and only knows the `temp` / `psu` sub-types. Convert legacy
		 * nodes to `sensor-type` + `hwmon-ins = <&sensor_dev 0>` and list
		 * the ABI files in `meas`. */
		if (dev->type == HWMON_TYPE_NONE) {
			LOG_WRN("%s: class-5 node has missing or unknown "
				"`sensor-type`, skipped", dev->label);
			continue;
		}

		if ((size_t)snprintf(path, sizeof(path), "/sys/class/hwmon/%s",
			     dev->label) >= sizeof(path)) {
			LOG_ERR("hwmon i2c path too long");
			break;
		}

		dev->node = sysfs_mkdir(path);
		if (!dev->node) {
			LOG_ERR("mkdir failed: %s", path);
			break;
		}

		hwmon_dev_setup(dev);
		for (size_t j = 0; j < dev->channel_num; j++) {
			sysfs_add_attribute(dev->node, &dev->channels[j].attr);
			if (dev->channels[j].has_label) {
				sysfs_add_attribute(dev->node,
						    &dev->channels[j].label_attr);
			}
		}

		snprintf(dev->name_attr.name, SYSFS_ATTRIBUTE_NAME_MAX, "name");
		dev->name_attr.user_data = dev;
		dev->name_attr.ops = &name_attr_ops;
		sysfs_add_attribute(dev->node, &dev->name_attr);

		snprintf(dev->bus_addr_attr.name, SYSFS_ATTRIBUTE_NAME_MAX,
			 "bus_addr");
		dev->bus_addr_attr.user_data = dev;
		dev->bus_addr_attr.ops = &bus_addr_ops;
		sysfs_add_attribute(dev->node, &dev->bus_addr_attr);
	}

	return 0;
}

SYS_INIT(hwmon_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
