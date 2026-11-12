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
 * ALTERNATIVE, for chips that expose dozens of files (PMBus PSU, MOC FPGA...):
 * name an `attr-set` instead of listing every file, e.g.
 *
 *     attr-set = "pmbus";     // group table lives in this file
 *     channels = <2>;         // optional: drop files numbered > 2
 *
 * The set table is code, not DT, so adding a chip or a rail does not grow the
 * board overlay. Precedence: `meas` (explicit list) wins over `attr-set`.
 * Entry order is irrelevant -- the number inside each file name is the sensor
 * driver's sub-channel selector (val2 = number - 1) and is parsed at read
 * time. DT `channels = <n>` is an optional cut-off that drops files whose
 * number is > n; omitting it keeps the whole set. Layouts whose numbering
 * differs from the set still need `meas`.
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
#include <zephyr/drivers/sensor/caseopen_ls.h>

#define DT_DRV_COMPAT linkedsemi_hwmon

LOG_MODULE_REGISTER(hwmon_genernal, LOG_LEVEL_INF);

/* ---- sensor sub-type (class 5 only) ---- */
enum hwmon_sensor_type {
	HWMON_TYPE_NONE = 0,   /* legacy: no sensor-type property (deprecated) */
	HWMON_TYPE_TEMP,
	HWMON_TYPE_PMBUS,
	HWMON_TYPE_INTRUSION,
	/* PECI CPU temperature + power (IntelCPUSensor). The Intel CPU is
	 * on a PECI bus, not I2C, but the same hwmon sysfs interface is
	 * used as a transport: DTS / Margin / DIMM A1 / DIMM B1 come
	 * through tempN_input + *_label, and Package / DRAM power through
	 * powerN_input + *_label. The `name` file reports "peci_cputemp"
	 * (the same string the upstream Linux xeon-pci-sensors driver
	 * emits) so IntelCPUSensorMain can pick this node up unchanged. */
	HWMON_TYPE_PECI_CPUTEMP,
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

/* Sized for the biggest `attr-set` plus some headroom: a full PMBus PSU set
 * is ~8 files, the legacy `meas` lists top out around 16. */
#define HWMON_MAX_CHANNELS 24

/* ---- i2c client (controller + address) for bus_addr ---- */
struct hwmon_i2c_client {
	const struct device *bus;
	uint32_t addr;
};

/* The hwmon node can reference a sensor device via hwmon-ins. If that sensor
 * is on an I2C bus (temp/psu) we derive the bus/addr for the legacy bus_addr
 * file; if it is an on-SOC peripheral (intrusion/caseopen) there is no I2C
 * controller and DT_BUS would not expand, so leave bus NULL / addr 0.
 */
#define HWMON_SENSOR_IS_I2C(inst)                                         \
	UTIL_AND(DT_INST_NODE_HAS_PROP(inst, sensor_type),               \
		 DT_ON_BUS(DT_INST_PHANDLE_BY_IDX(inst, hwmon_ins, 0), i2c))

/* ---- per-instance channel (runtime) ----
 * Sysfs back-end ops only get the attribute, never the open file, so a
 * "already read" flag can only live on the attribute itself. It is re-armed by
 * open() and by poll_prepare() (see chan_poll_prepare): a reader of the fs API
 * (shell `fs cat`) opens once and sees value + EOF, while a poll driven reader
 * (dbus-sensors, boost::asio) re-arms on every poll and gets a fresh value. */
struct hwmon_channel {
	const char *filename;/* the "<chan><n>_input" file (label stripped) */
	struct sysfs_attribute attr;
	struct k_poll_signal poll_signal;/* for poll/async-read support (dbus-sensors) */
	bool read_done;      /* value already handed out, next read is EOF */
	struct hwmon_dev *dev;/* back pointer */

	/* optional "<chan><n>_label" file: created only when the DT `meas`
	 * entry carried a ":label" suffix (e.g. "in3_input:vout1"). The label
	 * lets dbus-sensors tell vin/vout, iin/iout, pin/pout apart; without it
	 * every rail falls back to the generic "output" property. */
	const char *label;   /* label file content (e.g. "vout1"), or NULL */
	struct sysfs_attribute label_attr;
	bool has_label;
	bool label_done;     /* per-file read cursor (see struct comment) */
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
	const char *attr_set_str;   /* DT `attr-set`, used when `meas` is absent */
	uint8_t channels_req;       /* DT `channels`, 0 = keep the whole set */
	size_t channel_num;         /* created by hwmon_dev_setup() */
	struct hwmon_channel channels[HWMON_MAX_CHANNELS];
	int64_t last_fetch_ms;
	struct sysfs_node *node;
	struct sysfs_attribute name_attr;
	struct sysfs_attribute bus_addr_attr;
	struct sysfs_attribute label_attr;
	struct sysfs_attribute rearm_attr;
	bool name_done;      /* per-file read cursors (see struct hwmon_channel) */
	bool bus_done;
	bool dev_label_done;
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
			HWMON_SENSOR_IS_I2C(inst),                \
			(DEVICE_DT_GET(DT_BUS(DT_INST_PHANDLE_BY_IDX( \
				inst, hwmon_ins, 0)))),          \
			(NULL)),                                \
		.addr = COND_CODE_1(                            \
			HWMON_SENSOR_IS_I2C(inst),                \
			(DT_REG_ADDR(DT_INST_PHANDLE_BY_IDX(     \
				inst, hwmon_ins, 0))),            \
			(0)),                                  \
	},                                                         \
		.files = HWMON_FILES(inst),                               \
		.file_count = DT_INST_PROP_LEN_OR(inst, meas, 0),          \
		.attr_set_str = COND_CODE_1(                              \
			DT_INST_NODE_HAS_PROP(inst, attr_set),            \
			(DT_INST_PROP(inst, attr_set)),                   \
			(NULL)),                                          \
		.channels_req = DT_INST_PROP_OR(inst, channels, 0),        \
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
	if (!strcmp(s, "pmbus")) {
		return HWMON_TYPE_PMBUS;
	}
	if (!strcmp(s, "intrusion")) {
		return HWMON_TYPE_INTRUSION;
	}
	if (!strcmp(s, "peci_cputemp")) {
		return HWMON_TYPE_PECI_CPUTEMP;
	}
	return HWMON_TYPE_NONE;
}

/* name file content exposed at /sys/class/hwmon/hwmonX/name. */
static const char *hwmon_name_for_type(enum hwmon_sensor_type t)
{
	switch (t) {
	case HWMON_TYPE_TEMP:
		return "temp";
	case HWMON_TYPE_PMBUS:
		return "pmbus";
	case HWMON_TYPE_INTRUSION:
		return "intrusion";
	case HWMON_TYPE_PECI_CPUTEMP:
		return "peci_cputemp";
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
	char prefix[16];
	enum sensor_channel c;

	/* e.g. "in3_input", "curr2_input", "power1_input", "temp1_input",
	 * "fan1_input", "intrusion1_input": a non-digit prefix, then a
	 * number, then "_input". */
	if (sscanf(name, "%15[^0-9]%u_input", prefix, &n) < 2) {
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
	} else if (!strcmp(prefix, "intrusion")) {
		c = SENSOR_CHAN_INTRUSION;
	} else {
		return -EINVAL;
	}

	*chan = c;
	*num = (int)n;
	return 0;
}

/* ---- per-device channel setup: DT `meas` list, or a code-side `attr-set` ---- */
/* forward declarations of the sysfs ops structs (defined below) */
static struct sysfs_attribute_ops chan_attr_ops;
static struct sysfs_attribute_ops label_attr_ops;
static struct sysfs_attribute_ops dev_label_attr_ops;
static struct sysfs_attribute_ops name_attr_ops;
static struct sysfs_attribute_ops bus_addr_ops;
static struct sysfs_attribute_ops rearm_attr_ops;

/* ---- attribute sets ----
 * A set is a code-side table of hwmon ABI files for a conventional layout, so
 * a chip with dozens of files needs one DT line instead of dozens. The table
 * holds FILE NAMES only -- it knows nothing about the chip's registers, scaling
 * or rails; the sensor driver still owns all of that.
 *
 * Entry order is irrelevant: each file name carries its sub-channel number,
 * which hwmon_chan_from_name() turns into the driver's rail selector
 * (val2 = number - 1) at read time. DT `channels = <n>` therefore just drops
 * entries whose number is > n; omitting it keeps the whole set.
 */
struct hwmon_set_entry {
	const char *file;   /* "<chan><n>_input" */
	const char *label;  /* "<chan><n>_label" content; NULL = no label file */
};

struct hwmon_attr_set {
	const char *name;
	const struct hwmon_set_entry *entries;
	size_t count;
};

#define HWMON_SET_ENTRY(_file, _label) \
	{ .file = (_file), .label = (_label) }

/* Conventional single-output PMBus PSU layout, matching the example in the
 * file header and in lsqsh_evb_cpu1.overlay: in1=vin, in3=vout1,
 * curr1=iin1, curr2=iout1, power1=pin, power2=pout1, temp1, fan1. */
static const struct hwmon_set_entry pmbus_set[] = {
	HWMON_SET_ENTRY("in1_input", "vin1"),
	HWMON_SET_ENTRY("curr1_input", "iin1"),
	HWMON_SET_ENTRY("power1_input", "pin"),
	HWMON_SET_ENTRY("in3_input", "vout1"),
	HWMON_SET_ENTRY("curr2_input", "iout1"),
	HWMON_SET_ENTRY("power2_input", "pout1"),
	HWMON_SET_ENTRY("temp1_input", NULL),
	HWMON_SET_ENTRY("fan1_input", NULL),
};

/* Test-stub layout: mirrors src/fake_psu.c's rail numbering (in1=vin,
 * in2=vin1, in3=vout1, power2=pin1, power3=pout1, curr1=iout1, temp1, fan1),
 * i.e. exactly what lsqsh_evb_cpu1_fake_psu.overlay used to list in `meas`.
 * Differs from `pmbus` only in which sub-channel each rail maps to, so the
 * values land on the right dbus properties with the fake driver.
 * The labels must stay in PSUSensorMain's labelMatch table. */
static const struct hwmon_set_entry fake_psu_set[] = {
	HWMON_SET_ENTRY("in1_input", "vin"),
	HWMON_SET_ENTRY("in2_input", "vin1"),
	HWMON_SET_ENTRY("in3_input", "vout1"),
	HWMON_SET_ENTRY("power2_input", "pin1"),
	HWMON_SET_ENTRY("power3_input", "pout1"),
	HWMON_SET_ENTRY("curr1_input", "iout1"),
	HWMON_SET_ENTRY("temp1_input", "temp1"),
	HWMON_SET_ENTRY("fan1_input", "fan1"),
};

static const struct hwmon_attr_set hwmon_attr_sets[] = {
	{
		.name = "pmbus",
		.entries = pmbus_set,
		.count = ARRAY_SIZE(pmbus_set),
	},
	{
		.name = "fake-psu",
		.entries = fake_psu_set,
		.count = ARRAY_SIZE(fake_psu_set),
	},
};

static const struct hwmon_attr_set *hwmon_find_set(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(hwmon_attr_sets); i++) {
		if (!strcmp(hwmon_attr_sets[i].name, name)) {
			return &hwmon_attr_sets[i];
		}
	}
	return NULL;
}

/* Append one channel (value file + optional label file) to `dev`. `label` must
 * stay valid for the lifetime of the node: either a string literal from a set
 * table or the tail of a DT string. */
static void hwmon_chan_add(struct hwmon_dev *dev, const char *file,
			   const char *label)
{
	if (dev->channel_num >= HWMON_MAX_CHANNELS) {
		LOG_WRN("%s: too many hwmon files, %s dropped", dev->label, file);
		return;
	}

	struct hwmon_channel *ch = &dev->channels[dev->channel_num++];

	ch->dev = dev;
	ch->read_done = false;
	ch->label_done = false;
	k_poll_signal_init(&ch->poll_signal);

	snprintf(ch->attr.name, sizeof(ch->attr.name), "%s", file);
	ch->filename = ch->attr.name;
	ch->attr.user_data = ch;
	ch->attr.ops = &chan_attr_ops;

	/* optional label file: name = value name with "input"->"label". */
	ch->has_label = false;
	ch->label = NULL;
	if (label != NULL && label[0] != '\0') {
		snprintf(ch->label_attr.name, sizeof(ch->label_attr.name), "%s",
			 file);

		char *p = strstr(ch->label_attr.name, "input");

		if (p) {
			memcpy(p, "label", 5);/* same length */
			ch->label = label;
			ch->label_attr.user_data = ch;
			ch->label_attr.ops = &label_attr_ops;
			ch->has_label = true;
		}
	}
}

static void hwmon_dev_setup(struct hwmon_dev *dev)
{
	dev->name_str = hwmon_name_for_type(dev->type);
	dev->last_fetch_ms = 0;
	dev->channel_num = 0;

	/* 1) Explicit `meas` list wins (backward compatible: every existing board
	 * DT keeps working unchanged). */
	if (dev->file_count > 0) {
		for (size_t i = 0; i < dev->file_count; i++) {
			const char *entry = dev->files[i];
			const char *colon = strchr(entry, ':');
			size_t fn_len = colon ? (size_t)(colon - entry)
					      : strlen(entry);
			char file[SYSFS_ATTRIBUTE_NAME_MAX + 1];

			if (fn_len > SYSFS_ATTRIBUTE_NAME_MAX) {
				LOG_ERR("%s: meas entry too long: %s",
					dev->label, entry);
				continue;
			}
			memcpy(file, entry, fn_len);
			file[fn_len] = '\0';
			hwmon_chan_add(dev, file,
				       (colon != NULL) ? colon + 1 : NULL);
		}
		return;
	}

	/* 2) `attr-set`: expand the code-side table. Entry order is irrelevant;
	 * the sub-channel number inside each file name is what the read path
	 * turns into val2. DT `channels = <n>` (optional) merely drops files
	 * whose number is > n; without it the whole set is created. */
	if (dev->attr_set_str != NULL) {
		const struct hwmon_attr_set *set =
			hwmon_find_set(dev->attr_set_str);

		if (set == NULL) {
			LOG_ERR("%s: unknown attr-set '%s'", dev->label,
				dev->attr_set_str);
			return;
		}

		for (size_t i = 0; i < set->count; i++) {
			enum sensor_channel chan;
			int num = 0;

			/* A name hwmon cannot parse is still added (the read
			 * path reports it per file) -- but it never matches the
			 * `channels` cut-off either. */
			if (dev->channels_req != 0 &&
			    hwmon_chan_from_name(set->entries[i].file, &chan,
						 &num) == 0 &&
			    num > (int)dev->channels_req) {
				continue;
			}
			hwmon_chan_add(dev, set->entries[i].file,
				       set->entries[i].label);
		}

		LOG_INF("%s: attr-set '%s' -> %u file(s), channels=%u",
			dev->label, set->name, (unsigned int)dev->channel_num,
			(unsigned int)dev->channels_req);
		return;
	}

	LOG_WRN("%s: neither `meas` nor `attr-set` given; no channel files",
		dev->label);
}

/* ---- channel attribute ops ---- */
static int chan_attr_open(sysfs_attr_t attr)
{
	struct hwmon_channel *ch = attr->user_data;

	/* A fresh descriptor starts at offset 0: re-arm the one-shot value. */
	ch->read_done = false;
	return 0;
}

static ssize_t chan_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_channel *ch = attr->user_data;
	struct hwmon_dev *dev = ch->dev;

	/* Value already handed out on this open/poll cycle -> EOF. */
	if (ch->read_done) {
		return 0;
	}

	int ret = hwmon_ensure_fresh(dev);
	if (ret < 0) {
		LOG_ERR("%s: %s ensure_fresh failed (%d)", dev->label,
			ch->filename, ret);
		return -EIO;
	}

	/* Parse the filename (e.g. "in3_input") into the sensor_channel (from
	 * the prefix) and the 1-based number. val2 = number - 1 is the rail /
	 * sub-channel selector the driver's channel_get already understands, so
	 * hwmon needs no knowledge of rails. */
	enum sensor_channel chan;
	int num;

	if (hwmon_chan_from_name(ch->filename, &chan, &num) < 0) {
		LOG_ERR("%s: unrecognised hwmon file %s", dev->label,
			ch->filename);
		return -EIO;
	}

	struct sensor_value val = { .val2 = num - 1 };
	ret = sensor_channel_get(dev->sensor_dev, chan, &val);

	if (ret < 0) {
		LOG_ERR("%s: %s channel_get failed (%d)", dev->label,
			ch->filename, ret);
		return -EIO;
	}

	/* driver returns the value already in the hwmon ABI unit (val1). */
	int len = snprintf(buf, size, "%d\n", val.val1);

	if (len > 0) {
		ch->read_done = true;
	}

	return len;
}

static int chan_attr_close(sysfs_attr_t attr)
{
	ARG_UNUSED(attr);

	return 0;
}

/* ---- poll/async-read support ----
 * dbus-sensors (HwmonTempSensor) used to read tempN_input via boost::asio
 * async_read_some, which needs fcntl (F_GETFL/F_SETFL) + poll
 * (ZFD_IOCTL_POLL_*). Without an ioctl handler the async read fails ->
 * Value=nan.
 *
 * Attribute contents are generated on demand in chan_attr_read(), so a freshly
 * opened descriptor is always readable: report POLLIN unconditionally. The
 * read cursor is deliberately NOT consulted here -- it is attribute wide, so
 * gating POLLIN on it used to silence one reader because another had already
 * consumed the value.
 *
 * Every poll() call re-arms the cursor instead: a poll driven reader
 * (boost::asio keeps its fd open forever) gets one value per poll cycle, while
 * a plain fs reader that never polls (`fs cat`) still terminates on EOF. */
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
		ch->read_done = false;
		k_poll_signal_raise(&ch->poll_signal, 0);
	}
	return 0;
}

static int chan_poll_update(struct hwmon_channel *ch,
			    struct zvfs_pollfd *pfd,
			    struct k_poll_event **pev)
{
	(void)ch;

	if (pfd->events & ZVFS_POLLIN) {
		if ((*pev)->state != K_POLL_STATE_NOT_READY) {
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

	ch->label_done = false;
	return 0;
}

static ssize_t label_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_channel *ch = attr->user_data;

	if (ch->label_done) {
		return 0;
	}

	int len = snprintf(buf, size, "%s\n", ch->label);

	if (len > 0) {
		ch->label_done = true;
	}
	return len;
}

static int label_attr_close(sysfs_attr_t attr)
{
	ARG_UNUSED(attr);

	return 0;
}

static struct sysfs_attribute_ops label_attr_ops = {
	.open = label_attr_open,
	.read = label_attr_read,
	.write = NULL,
	.close = label_attr_close,
};

/* ---- device label attribute ops ----
 * A device-level "label" file with the DT `label` property value so
 * dbus-sensors can identify the hwmon node by its HwmonLabel. */
static int dev_label_attr_open(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->dev_label_done = false;
	return 0;
}

static ssize_t dev_label_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_dev *dev = attr->user_data;

	if (dev->dev_label_done) {
		return 0;
	}

	int len = snprintf(buf, size, "%s\n", dev->label);

	if (len > 0) {
		dev->dev_label_done = true;
	}
	return len;
}

static int dev_label_attr_close(sysfs_attr_t attr)
{
	ARG_UNUSED(attr);

	return 0;
}

static struct sysfs_attribute_ops dev_label_attr_ops = {
	.open = dev_label_attr_open,
	.read = dev_label_attr_read,
	.write = NULL,
	.close = dev_label_attr_close,
};

/* ---- name attribute ops ---- */
static int name_attr_open(sysfs_attr_t attr)
{
	struct hwmon_dev *dev = attr->user_data;

	dev->name_done = false;
	return 0;
}

static ssize_t name_attr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_dev *dev = attr->user_data;

	if (dev->name_done) {
		return 0;
	}

	int len = snprintf(buf, size, "%s\n", dev->name_str);

	if (len > 0) {
		dev->name_done = true;
	}
	return len;
}

static int name_attr_close(sysfs_attr_t attr)
{
	ARG_UNUSED(attr);

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

	dev->bus_done = false;
	return 0;
}

static ssize_t bus_addr_read(sysfs_attr_t attr, void *buf, size_t size)
{
	struct hwmon_dev *dev = attr->user_data;
	int bus = hwmon_i2c_bus_num(dev->client.bus);
	int len;

	if (dev->bus_done) {
		return 0;
	}

	len = snprintf(buf, size, "%u-%04x\n", bus, dev->client.addr);

	if (len > 0) {
		dev->bus_done = true;
	}
	return len;
}

static int bus_addr_close(sysfs_attr_t attr)
{
	ARG_UNUSED(attr);

	return 0;
}

static struct sysfs_attribute_ops bus_addr_ops = {
	.open = bus_addr_open,
	.read = bus_addr_read,
	.write = NULL,
	.close = bus_addr_close,
};

/* ---- rearm attribute ops (intrusion-only) ----
 * Writing anything to this file triggers a hardware latch clear on the
 * underlying caseopen sensor. */
static int rearm_attr_open(sysfs_attr_t attr)
{
	return 0;
}

static ssize_t rearm_attr_write(sysfs_attr_t attr, const void *buf,
				size_t size)
{
	struct hwmon_dev *dev = attr->user_data;
	struct sensor_value val;

	if (!dev->sensor_dev) {
		return -ENODEV;
	}

	val.val1 = 0;
	val.val2 = 0;
	int ret = sensor_attr_set(dev->sensor_dev, SENSOR_CHAN_INTRUSION,
				  SENSOR_ATTR_CASEOPEN_REARM, &val);

	if (ret < 0) {
		LOG_ERR("%s: rearm failed (%d)", dev->label, ret);
		return ret;
	}

	LOG_INF("%s: rearmed", dev->label);
	return (ssize_t)size;
}

static int rearm_attr_close(sysfs_attr_t attr)
{
	return 0;
}

static struct sysfs_attribute_ops rearm_attr_ops = {
	.open = rearm_attr_open,
	.read = NULL,
	.write = rearm_attr_write,
	.close = rearm_attr_close,
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

		snprintf(dev->label_attr.name, SYSFS_ATTRIBUTE_NAME_MAX, "label");
		dev->label_attr.user_data = dev;
		dev->label_attr.ops = &dev_label_attr_ops;
		sysfs_add_attribute(dev->node, &dev->label_attr);

		if (dev->type == HWMON_TYPE_INTRUSION) {
			snprintf(dev->rearm_attr.name,
				 SYSFS_ATTRIBUTE_NAME_MAX, "rearm");
			dev->rearm_attr.user_data = dev;
			dev->rearm_attr.ops = &rearm_attr_ops;
			sysfs_add_attribute(dev->node, &dev->rearm_attr);
		}

		snprintf(dev->bus_addr_attr.name, SYSFS_ATTRIBUTE_NAME_MAX,
			 "bus_addr");
		dev->bus_addr_attr.user_data = dev;
		dev->bus_addr_attr.ops = &bus_addr_ops;
		sysfs_add_attribute(dev->node, &dev->bus_addr_attr);
	}

	return 0;
}

SYS_INIT(hwmon_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
