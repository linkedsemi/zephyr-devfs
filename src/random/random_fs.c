#include <stdarg.h>

#include <zephyr/fs/devfs.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/random/random.h>
#include <zephyr/sys/fdtable.h>

LOG_MODULE_REGISTER(random_fs, LOG_LEVEL_ERR);

#define RANDOM_DEV_PATH "/dev/urandom"

struct random_file {
    int flags;
    struct k_poll_signal poll_signal;
};

K_MEM_SLAB_DEFINE_STATIC(random_file_pool, sizeof(struct random_file),
             CONFIG_DEVFS_FILE_NUM_MAX, __alignof__(struct random_file));

static int random_open(struct fs_file_t *zfp, const char *file_name,
               fs_mode_t mode)
{
    struct random_file *file;
    int ret;

    ARG_UNUSED(file_name);

    if (mode & FS_O_WRITE) {
        return -EACCES;
    }

    ret = k_mem_slab_alloc(&random_file_pool, (void **)&file, K_NO_WAIT);
    if (ret < 0) {
        return ret;
    }

    file->flags = O_RDONLY;
    k_poll_signal_init(&file->poll_signal);
    zfp->filep = file;

    return 0;
}

static int random_close(struct fs_file_t *zfp)
{
    struct random_file *file = zfp->filep;

    if (file == NULL) {
        return -EBADF;
    }

    k_poll_signal_reset(&file->poll_signal);
    k_mem_slab_free(&random_file_pool, file);
    zfp->filep = NULL;

    return 0;
}

static ssize_t random_read(struct fs_file_t *zfp, void *ptr, size_t size)
{
    int ret;

    ARG_UNUSED(zfp);

    ret = sys_csrand_get(ptr, size);
    if (ret < 0) {
        return ret;
    }

    return size;
}

static int random_poll_prepare(struct random_file *file,
                   struct zvfs_pollfd *pfd,
                   struct k_poll_event **pev,
                   struct k_poll_event *pev_end)
{
    if (!(pfd->events & ZVFS_POLLIN)) {
        return 0;
    }

    if (*pev >= pev_end) {
        return -ENOMEM;
    }

    (*pev)->obj = &file->poll_signal;
    (*pev)->type = K_POLL_TYPE_SIGNAL;
    (*pev)->mode = K_POLL_MODE_NOTIFY_ONLY;
    (*pev)->state = K_POLL_STATE_NOT_READY;
    (*pev)++;

    /* Random bytes are generated on demand, so POLLIN is always ready. */
    k_poll_signal_raise(&file->poll_signal, 0);

    return 0;
}

static int random_poll_update(struct random_file *file,
                  struct zvfs_pollfd *pfd,
                  struct k_poll_event **pev)
{
    if (!(pfd->events & ZVFS_POLLIN)) {
        return 0;
    }

    if ((*pev)->state != K_POLL_STATE_NOT_READY) {
        pfd->revents |= ZVFS_POLLIN;
    }
    (*pev)++;

    k_poll_signal_reset(&file->poll_signal);

    return 0;
}

static int random_ioctl(struct fs_file_t *zfp, unsigned long request,
            va_list args)
{
    struct random_file *file = zfp->filep;

    if (file == NULL) {
        return -EBADF;
    }

    switch (request) {
    case F_GETFL:
        return file->flags;

    case F_SETFL: {
        int flags = va_arg(args, int);

        file->flags = (file->flags & O_ACCMODE) | (flags & O_NONBLOCK);
        return 0;
    }

    case ZFD_IOCTL_POLL_PREPARE: {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);
        struct k_poll_event *pev_end = va_arg(args, struct k_poll_event *);

        return random_poll_prepare(file, pfd, pev, pev_end);
    }

    case ZFD_IOCTL_POLL_UPDATE: {
        struct zvfs_pollfd *pfd = va_arg(args, struct zvfs_pollfd *);
        struct k_poll_event **pev = va_arg(args, struct k_poll_event **);

        return random_poll_update(file, pfd, pev);
    }

    default:
        return -ENOTTY;
    }
}

static const struct fs_file_system_t random_fs = {
    .open = random_open,
    .close = random_close,
    .read = random_read,
    .ioctl = random_ioctl,
};

static int random_fs_init(void)
{
    return devfs_register(RANDOM_DEV_PATH, &random_fs);
}

SYS_INIT(random_fs_init, APPLICATION, CONFIG_FILE_SYSTEM_INIT_PRIORITY);
