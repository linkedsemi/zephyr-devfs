#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/dirent.h>
#include <zephyr/sys/printk.h>
#include <poll.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdio.h>

static int hwmon_i2c_name_read(void)
{
    char name[16];
    int fd = open("/sys/class/hwmon/hwmon1/name", O_RDONLY);

    int len = read(fd, name, sizeof(name));
    name[len] = '\0';
    printf("len = %d, name: %s", len, name);

    close(fd);
    return 0;
}

static int hwmon_i2c_test(void)
{
    hwmon_i2c_name_read();

    return 0;
}
SHELL_CMD_REGISTER(hwmon_i2c_test, NULL, "hwmon_i2c_test", hwmon_i2c_test);