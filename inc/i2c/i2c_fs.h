#ifndef __I2C_FS_H__
#define __I2C_FS_H__

#include <zephyr/device.h>
#include <zephyr/spinlock.h>

struct fs_file_t;

struct i2c_device
{
    const struct device *dev;
    const char *label;
    uint16_t addr;
    /* Exclusive-open bookkeeping.
     *
     * This filesystem allows ONE open per bus (mirroring the old Linux i2c-dev
     * single-client rule, and needed because addr/flags below are per-open, not
     * per-bus state). That is what makes a leaked fd fatal: the leaked fd holds
     * the bus and every later open fails with -EBUSY, for the rest of the
     * session.
     *
     * owner      makes a release attributable: only the holder may give the bus
     *            back, so a stale or foreign file object cannot release a bus
     *            somebody else is still using.
     * open_seq /
     * close_seq  make a leak distinguishable from real contention. Contention
     *            keeps opens-minus-closes at 0 or 1; a tool that leaks makes it
     *            climb, and the -EBUSY message can say so instead of leaving the
     *            caller to guess.
     *
     * lock guards owner, open_seq, close_seq, addr and flags together, so those
     * can never be observed half-updated. */
    struct fs_file_t *owner;
    uint32_t open_seq;
    uint32_t close_seq;
    uint8_t flags;
    struct k_spinlock lock;
};

#endif /* __I2C_FS_H__ */
