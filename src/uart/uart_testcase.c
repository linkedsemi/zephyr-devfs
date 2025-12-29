#include <stdio.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/posix/dirent.h>
#include <zephyr/sys/printk.h>
#include <poll.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>

static void uart_nonblock_write(void)
{
    int fd = open("/dev/ttyS0", O_RDWR);

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    printk("fd = %d.\n", fd);

    write(fd, "hello /dev/ttyS0\r\n", strlen("hello /dev/ttyS0\r\n"));
    write(fd, "hello /dev/ttyS0\r\n", strlen("hello /dev/ttyS0\r\n"));
    write(fd, "hello /dev/ttyS0\r\n", strlen("hello /dev/ttyS0\r\n"));

    k_msleep(10);

    close(fd);
}

static void uart_nonblock_read(void)
{
    int fd = open("/dev/ttyS0", O_RDWR);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    uint8_t read_buf[9] = {0};
    /* default is block recv */
    int size = read(fd, read_buf, 8);
    printf("size = %d, read_buf = %s.\n", size, read_buf);

    close(fd);
}

static void uart_block_read(void)
{
    int fd = open("/dev/ttyS0", O_RDWR);
    uint8_t read_buf[9] = {0};

    /* default is block recv */
    int size = read(fd, read_buf, 8);
    printf("size = %d, read_buf = %s.\n", size, read_buf);

    close(fd);
}

static void uart_block_write(void)
{
    int fd = open("/dev/ttyS0", O_RDWR);
    const char *write_str = "hello /dev/ttyS 0hello /dev/ttyS0 hello /dev/ttyS0 hello /dev/ttyS0 hello /dev/ttyS 0hello /dev/ttyS0 hello /dev/ttyS0 hello /dev/ttyS0\r\n";
    /* Adjust the tx_rb buffer size to 32 to test this function */
    int w_size = write(fd, write_str, strlen(write_str));
    __ASSERT(w_size == strlen(write_str), "write fail");

    close(fd);
}

int uart_testcase(void)
{
    printf("nonblock read and write test\n");
    uart_nonblock_write();
    uart_nonblock_read();

    printf("block read and write test\n");
    uart_block_read();
    uart_block_write();

    // printf("poll test\n");

    return 0;
}
SHELL_CMD_REGISTER(uart_testcase, NULL, "uart_testcase", uart_testcase);
