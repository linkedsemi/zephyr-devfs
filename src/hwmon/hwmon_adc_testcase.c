#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/dirent.h>
#include <zephyr/sys/printk.h>
#include <poll.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <stdio.h>

static int hwmon_adc_name_read(void)
{
    char name[16];
    int fd = open("/sys/class/hwmon/hwmon0/name", O_RDONLY);

    int len = read(fd, name, sizeof(name));
    name[len] = '\0';
    printf("len = %d, name: %s", len, name);

    close(fd);
    return 0;
}

static int hwmon_adc_read(void)
{
    uint8_t value[8];
    int fd_0 = open("/sys/class/hwmon/hwmon0/in0_input", O_RDONLY);
    int fd_1 = open("/sys/class/hwmon/hwmon0/in1_input", O_RDONLY);
    int fd_2 = open("/sys/class/hwmon/hwmon0/in2_input", O_RDONLY);

    int len = read(fd_0, &value, sizeof(value));
    value[len] = '\0';
    printf("len = %d, in0_input value = %s", len, value);

    len = read(fd_1, &value, sizeof(value));
    value[len] = '\0';
    printf("len = %d, in0_input value = %s", len, value);

    len = read(fd_2, &value, sizeof(value));
    value[len] = '\0';
    printf("len = %d, in0_input value = %s", len, value);

    close(fd_0);
    close(fd_1);
    close(fd_2);

    return 0;
}

static int hwmon_adc_testcase(void)
{
    hwmon_adc_name_read();
    hwmon_adc_read();

    return 0;
}
SHELL_CMD_REGISTER(hwmon_adc_testcase, NULL, "hwmon_adc_testcase", hwmon_adc_testcase);