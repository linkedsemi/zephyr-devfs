#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/sys/ioctl.h>
#include <string.h>
#include <linux/ioctl.h>
#include <gpio/gpio_fs.h>
#include <zephyr/shell/shell.h>
#include <stdio.h>

static int gpio_chip_info(void)
{
    char path[64];
    for (int i = 0; i < 15; i++)
    {
        snprintf(path, 64, "/dev/gpiochip%d", i);

        int fd = open(path, O_RDWR);
        struct gpiochip_info chipinfo;
        memset(&chipinfo, 0, sizeof(chipinfo));

        if (ioctl(fd, GPIO_GET_CHIPINFO_IOCTL, &chipinfo) < 0)
        {
            printf("[TEST] Failed to GPIO_GET_CHIPINFO_IOCTL failed! errno=%d, path: %s \n", errno, path);
        }
        else
        {
            printf("[TEST] Chip name: %s\n", chipinfo.name);
            printf("[TEST] Chip label: %s\n", chipinfo.label);
            printf("[TEST] Number of lines: %u\n", chipinfo.lines);
        }
        close(fd);
    }

    return 0;
}

static int gpio_line_info(void)
{
    char path[64];
    for (int i = 0; i < 15; i++)
    {
        snprintf(path, 64, "/dev/gpiochip%d", i);

        int fd = open(path, O_RDWR);
        struct gpioline_info lineinfo;
        memset(&lineinfo, 0, sizeof(lineinfo));

        lineinfo.line_offset = 11;

        if (ioctl(fd, GPIO_GET_LINEINFO_IOCTL, &lineinfo) < 0)
        {
            printf("[TEST] Failed to GPIO_GET_LINEINFO_IOCTL failed! errno=%d \n",errno);
        }
        else
        {
            printf("[TEST] Line Offset: %u\n", lineinfo.line_offset);
            printf("[TEST] Flags: 0x%x\n", lineinfo.flags);
            printf("[TEST] Name: %s\n", lineinfo.name);
            printf("[TEST] Consumer: %s\n", lineinfo.consumer);
        }
        close(fd);
    }

    return 0;
}

static int gpio_linehandle_output_test(void)
{
    int fd = open("/dev/gpiochip7", O_RDWR);
    struct gpiohandle_request req;

    req.lineoffsets[0] = 8;
    req.lineoffsets[1] = 7;
    req.lineoffsets[2] = 0;

    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 1;
    req.default_values[1] = 0;
    req.default_values[2] = 1;
    req.lines = 3;

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        printf("[TEST] Failed to GPIO_GET_LINEHANDLE_IOCTL failed!\n");
        close(fd);
    }

    k_msleep(1000);

    struct gpiohandle_data data;
    data.values[0] = 0;
    data.values[1] = 1;
    data.values[2] = 0;

    if (ioctl(req.fd, GPIOHANDLE_SET_LINE_VALUES_IOCTL, &data) < 0)
    {
        printf("[TEST]---Failed to GPIO_SET_LINEINFO_IOCTL failed-------\n");
    }

    k_msleep(1000);

    close(req.fd);
    close(fd);

    return 0;
}

static int gpio_linehandle_input_test(void)
{
    int fd = open("/dev/gpiochip6", O_RDWR);
    struct gpiohandle_request req;
    req.flags = GPIOHANDLE_REQUEST_INPUT;
    req.lineoffsets[0] = 4;
    req.lines = 1;

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        printf("[TEST] Failed to GPIO_GET_LINEHANDLE_IOCTL failed!\n");
        close(fd);
    }

    struct gpiohandle_data data;
    if (ioctl(req.fd, GPIOHANDLE_GET_LINE_VALUES_IOCTL, &data) < 0)
    {
        printf("[TEST]---Failed to GPIO_SET_LINEINFO_IOCTL failed-------\n");
    }
    else
    {
        printf("[TEST]---GPIOHANDLE_GET_LINE_VALUES_IOCTL value = %d\n", data.values[0]);
    }

    close(req.fd);
    close(fd);

    return 0;
}

static int gpio_linehandle_open_multi(void)
{
    int fd = open("/dev/gpiochip7", O_RDWR);
    struct gpiohandle_request req, req1;

    req.lineoffsets[0] = 8;
    req.lineoffsets[1] = 7;
    req.lineoffsets[2] = 0;

    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 1;
    req.default_values[1] = 1;
    req.default_values[2] = 1;
    req.lines = 3;

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        printf("[TEST] Failed to GPIO_GET_LINEHANDLE_IOCTL failed!\n");
        close(fd);
        return -1;
    }

    req1.lineoffsets[0] = 9;
    req1.lineoffsets[1] = 6;
    req1.lineoffsets[2] = 0;

    req1.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req1.default_values[0] = 1;
    req1.default_values[1] = 0;
    req1.default_values[2] = 1;
    req1.lines = 3;

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req1) < 0) {
        printf("[TEST] success to GPIO_GET_LINEHANDLE_IOCTL multi !\n");
        close(fd);
        close(req.fd);
        return -1;
    }

    return 0;
}

static int gpio_linehandle_config(void)
{
    int fd = open("/dev/gpiochip7", O_RDWR);
    struct gpiohandle_request req;

    req.lineoffsets[0] = 8;
    req.lineoffsets[1] = 7;
    req.lineoffsets[2] = 0;

    req.flags = GPIOHANDLE_REQUEST_OUTPUT;
    req.default_values[0] = 1;
    req.default_values[1] = 1;
    req.default_values[2] = 1;
    req.lines = 3;

    if (ioctl(fd, GPIO_GET_LINEHANDLE_IOCTL, &req) < 0) {
        printf("[TEST] Failed to GPIO_GET_LINEHANDLE_IOCTL failed!\n");
        close(fd);
    }

    k_msleep(1000);

    struct gpiohandle_config config;
    config.flags = GPIOHANDLE_REQUEST_OUTPUT;
    config.default_values[0] = 0;
    config.default_values[1] = 0;
    config.default_values[2] = 0;

    if (ioctl(req.fd, GPIOHANDLE_SET_CONFIG_IOCTL, &config) < 0)
    {
        printf("[TEST]---Failed to GPIOHANDLE_SET_CONFIG_IOCTL failed-------\n");
    }

    close(req.fd);
    close(fd);

    return 0;
}

#include <poll.h>

static int gpio_lineevent_test(void)
{
    int fd = open("/dev/gpiochip6", O_RDWR);

    struct gpioevent_request req;

    req.lineoffset = 4;
    req.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;

    if(ioctl(fd, GPIO_GET_LINEEVENT_IOCTL, &req) < 0)
    {
        printf("[TEST] Failed to GPIO_GET_LINEEVENT_IOCTL failed!\n");
        close(fd);
    }

    struct pollfd pfd = {
        .fd = req.fd,
        .events = POLLIN,
        .revents = 0
    };

    while (1)
    {
        int ret = poll(&pfd, 1, 3000);
        if (ret < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }

        if (ret == 0)
        {
            printf("timeout!!\n");
            break;
        }

        if (pfd.revents & POLLIN)
        {
            struct gpioevent_data evt_data;
            ssize_t read_len = read(pfd.fd, &evt_data, sizeof(evt_data));

            if (read_len == sizeof(evt_data))
            {
                printf("read_len = %zd, timestamp: %lld, id: %d\n", read_len, evt_data.timestamp, evt_data.id);
                break;
            }
            else
            {
                printf("[TEST] event read fail!!\n");
            }
        }
    }

    close(req.fd);
    close(fd);

    return 0;
}

static int gpio_lineevent_multi_test(void)
{
    int fd_0 = open("/dev/gpiochip6", O_RDWR); // g4
    int fd_1 = open("/dev/gpiochip7", O_RDWR); // h1 h4
    int fd_2 = open("/dev/gpiochip7", O_RDWR); // h1 h4

    if (fd_0 < 0 || fd_1 < 0 || fd_2 < 0)
    {
        printf( "[ERROR] fail open chip \n");
        return -1;
    }

    struct gpioevent_request req_0;
    struct gpioevent_request req_1;
    struct gpioevent_request req_2;

    req_0.lineoffset = 4;
    req_0.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req_0.eventflags = GPIOEVENT_REQUEST_BOTH_EDGES;

    req_1.lineoffset = 1;
    req_1.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req_1.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;

    req_2.lineoffset = 4;
    req_2.handleflags = GPIOHANDLE_REQUEST_INPUT;
    req_2.eventflags = GPIOEVENT_REQUEST_FALLING_EDGE;

    if(ioctl(fd_0, GPIO_GET_LINEEVENT_IOCTL, &req_0) < 0 ||
       ioctl(fd_1, GPIO_GET_LINEEVENT_IOCTL, &req_1) < 0 ||
       ioctl(fd_2, GPIO_GET_LINEEVENT_IOCTL, &req_2) < 0
    )
    {
        printf("[TEST] Failed to GPIO_GET_LINEEVENT_IOCTL failed!\n");
        if (req_0.fd >= 0)
            close(req_0.fd);
        if (req_1.fd >= 0)
            close(req_1.fd);
        close(fd_0);
        close(fd_1);
    }

    struct pollfd pfd[3] = {
        {
            .fd = req_0.fd,
            .events = POLLIN,
            .revents = 0
        },
        {
            .fd = req_1.fd,
            .events = POLLIN,
            .revents = 0
        },
        {
            .fd = req_2.fd,
            .events = POLLIN,
            .revents = 0
        }
    };

    printf("poll start!!\n");
    while (1)
    {
        int ret = poll(pfd, sizeof(pfd) / sizeof(*pfd), 5000);
        if (ret < 0)
        {
            if (errno == EINTR)
                continue;
            break;
        }

        if (ret == 0)
        {
            printf("poll timeout!!\n");
            break;
        }

        for (int i = 0; i < sizeof(pfd) / sizeof(*pfd); i++)
        {
            if (pfd[i].revents & POLLIN)
            {
                while (1)
                {
                    struct gpioevent_data evt_data;
                    ssize_t read_len = read(pfd[i].fd, &evt_data, sizeof(evt_data));
                    if (read_len == sizeof (evt_data))
                    {
                        printf("read_len = %zd, timestamp: %lld, id: %d, fd = %d\n", read_len, evt_data.timestamp, evt_data.id, pfd[i].fd);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }

    close(req_0.fd);
    close(req_1.fd);
    close(req_2.fd);
    close(fd_0);
    close(fd_1);
    close(fd_2);

    return 0;
}

static int gpio_fs_test(void)
{
    gpio_chip_info();
    gpio_line_info();
    gpio_linehandle_output_test();
    gpio_linehandle_input_test();
    gpio_linehandle_open_multi();
    gpio_linehandle_config();
    gpio_lineevent_test();
    gpio_lineevent_multi_test();

    return 0;
}
SHELL_CMD_REGISTER(gpio_fs_test, NULL, "gpio_fs_test", gpio_fs_test);
