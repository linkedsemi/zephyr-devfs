/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <zephyr/posix/sys/ioctl.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/shell/shell.h>
#include <zephyr/fs/fs_sys.h>
#include <zephyr/logging/log_ctrl.h>
#include "i2c/i2c-dev.h"
#include "i2c/i2c.h"
#include <zephyr/posix/fcntl.h>

#include <linux/ioctl.h>

#define TEST_I2C_DEV "/dev/i2c-5"
#define TEST_ADDR 0x50
#define TEST_RETRIES 3
#define TEST_TIMEOUT 100

/* Helper macro to print test result */
#define TEST_PASS(fmt, ...) printf("  [PASS] " fmt "\n", ##__VA_ARGS__)
#define TEST_FAIL(fmt, ...) printf("  [FAIL] " fmt "\n", ##__VA_ARGS__)

/* Helper: test a condition and report pass/fail */
static int check_result(int ret, int expected, const char *test_name)
{
    if (ret == expected)
    {
        TEST_PASS("%s: got %d (expected %d)", test_name, ret, expected);
        return 0;
    }
    else
    {
        TEST_FAIL("%s: got %d (expected %d)", test_name, ret, expected);
        return -1;
    }
}

static int check_result_ne(int ret, int not_expected, const char *test_name)
{
    if (ret != not_expected)
    {
        TEST_PASS("%s: got %d (not %d)", test_name, ret, not_expected);
        return 0;
    }
    else
    {
        TEST_FAIL("%s: got %d (should not be %d)", test_name, ret, not_expected);
        return -1;
    }
}

/*
 * Test 1: Open and close operations
 */
static int test_open_close(void)
{
    int fd, fd2;
    int failures = 0;

    printf("\n========== Test 1: Open/Close ==========\n");

    /* 1.1: Normal open */
    printf("--- 1.1: Normal open ---\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    failures += check_result_ne(fd, -1, "open(O_RDWR)");

    /* 1.2: Exclusive open - second open should fail */
    printf("--- 1.2: Exclusive open (should fail with EBUSY) ---\n");
    fd2 = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd2 < 0)
    {
        TEST_PASS("Exclusive open: second open failed as expected (err=%d)", fd2);
    }
    else
    {
        TEST_FAIL("Exclusive open: second open succeeded unexpectedly");
        close(fd2);
        failures++;
    }

    /* 1.3: Close and reopen should succeed */
    printf("--- 1.3: Reopen after close ---\n");
    close(fd);
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    failures += check_result_ne(fd, -1, "reopen after close");

    /* 1.4: Open with O_RDONLY */
    printf("--- 1.4: Open with O_RDONLY ---\n");
    close(fd);
    fd = open(TEST_I2C_DEV, O_RDONLY);
    failures += check_result_ne(fd, -1, "open(O_RDONLY)");

    /* 1.5: Open with O_WRONLY */
    printf("--- 1.5: Open with O_WRONLY ---\n");
    close(fd);
    fd = open(TEST_I2C_DEV, O_WRONLY);
    failures += check_result_ne(fd, -1, "open(O_WRONLY)");

    close(fd);
    printf("--- Test 1: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 2: I2C_SLAVE and I2C_SLAVE_FORCE ioctl
 */
static int test_slave_address(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 2: Slave Address ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 2.1: Set valid 7-bit address via I2C_SLAVE */
    printf("--- 2.1: I2C_SLAVE with valid address (0x50) ---\n");
    ret = ioctl(fd, I2C_SLAVE, TEST_ADDR);
    failures += check_result(ret, 0, "I2C_SLAVE 0x50");

    /* 2.2: I2C_SLAVE_FORCE with valid address */
    printf("--- 2.2: I2C_SLAVE_FORCE with valid address (0x51) ---\n");
    ret = ioctl(fd, I2C_SLAVE_FORCE, 0x51);
    failures += check_result(ret, 0, "I2C_SLAVE_FORCE 0x51");

    /* 2.3: Set another valid address */
    printf("--- 2.3: I2C_SLAVE with 0x68 ---\n");
    ret = ioctl(fd, I2C_SLAVE, 0x68);
    failures += check_result(ret, 0, "I2C_SLAVE 0x68");

    /* 2.4: Set 10-bit address */
    printf("--- 2.4: I2C_SLAVE with 10-bit address (0x200) ---\n");
    ioctl(fd, I2C_TENBIT, 1);
    ret = ioctl(fd, I2C_SLAVE, 0x200);
    failures += check_result(ret, 0, "I2C_SLAVE 0x200 (10-bit)");

    /* 2.5: Set invalid address (out of range) */
    printf("--- 2.5: I2C_SLAVE with invalid address (0x400) ---\n");
    ret = ioctl(fd, I2C_SLAVE, 0x400);
    if (ret < 0)
    {
        TEST_PASS("I2C_SLAVE 0x400: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("I2C_SLAVE 0x400: should have been rejected");
        failures++;
    }

    /* 2.6: Set negative address */
    printf("--- 2.6: I2C_SLAVE with negative address (-1) ---\n");
    ret = ioctl(fd, I2C_SLAVE, -1);
    if (ret < 0)
    {
        TEST_PASS("I2C_SLAVE -1: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("I2C_SLAVE -1: should have been rejected");
        failures++;
    }

    /* Reset to valid address for subsequent tests */
    ioctl(fd, I2C_SLAVE, TEST_ADDR);

    close(fd);
    printf("--- Test 2: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 3: I2C_TENBIT ioctl
 */
static int test_tenbit(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 3: 10-bit Addressing (I2C_TENBIT) ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 3.1: Enable 10-bit addressing */
    printf("--- 3.1: Enable 10-bit addressing ---\n");
    ret = ioctl(fd, I2C_TENBIT, 1);
    failures += check_result(ret, 0, "I2C_TENBIT enable");

    /* 3.2: Disable 10-bit addressing */
    printf("--- 3.2: Disable 10-bit addressing ---\n");
    ret = ioctl(fd, I2C_TENBIT, 0);
    failures += check_result(ret, 0, "I2C_TENBIT disable");

    /* 3.3: Enable with non-zero value */
    printf("--- 3.3: Enable 10-bit with value 5 ---\n");
    ret = ioctl(fd, I2C_TENBIT, 5);
    failures += check_result(ret, 0, "I2C_TENBIT 5");

    /* 3.4: Disable again */
    printf("--- 3.4: Disable 10-bit again ---\n");
    ret = ioctl(fd, I2C_TENBIT, 0);
    failures += check_result(ret, 0, "I2C_TENBIT disable again");

    close(fd);
    printf("--- Test 3: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 4: I2C_PEC ioctl
 */
static int test_pec(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 4: PEC (Packet Error Checking) ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 4.1: Enable PEC */
    printf("--- 4.1: Enable PEC ---\n");
    ret = ioctl(fd, I2C_PEC, 1);
    failures += check_result(ret, 0, "I2C_PEC enable");

    /* 4.2: Disable PEC */
    printf("--- 4.2: Disable PEC ---\n");
    ret = ioctl(fd, I2C_PEC, 0);
    failures += check_result(ret, 0, "I2C_PEC disable");

    close(fd);
    printf("--- Test 4: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 5: I2C_FUNCS ioctl
 */
static int test_funcs(void)
{
    int fd, ret;
    unsigned long funcs = 0;
    int failures = 0;

    printf("\n========== Test 5: I2C_FUNCS ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 5.1: Get functions */
    printf("--- 5.1: Get I2C_FUNCS ---\n");
    ret = ioctl(fd, I2C_FUNCS, &funcs);
    failures += check_result(ret, 0, "I2C_FUNCS");
    printf("  funcs = 0x%lx\n", funcs);

    /* 5.2: Verify expected function bits are set */
    printf("--- 5.2: Check function bits ---\n");
    if (funcs & I2C_FUNC_I2C)
    {
        TEST_PASS("I2C_FUNC_I2C is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_I2C is NOT set");
        failures++;
    }

    if (funcs & I2C_FUNC_SMBUS_BYTE)
    {
        TEST_PASS("I2C_FUNC_SMBUS_BYTE is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_SMBUS_BYTE is NOT set");
        failures++;
    }

    if (funcs & I2C_FUNC_SMBUS_BYTE_DATA)
    {
        TEST_PASS("I2C_FUNC_SMBUS_BYTE_DATA is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_SMBUS_BYTE_DATA is NOT set");
        failures++;
    }

    if (funcs & I2C_FUNC_SMBUS_WORD_DATA)
    {
        TEST_PASS("I2C_FUNC_SMBUS_WORD_DATA is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_SMBUS_WORD_DATA is NOT set");
        failures++;
    }

    if (funcs & I2C_FUNC_SMBUS_BLOCK_DATA)
    {
        TEST_PASS("I2C_FUNC_SMBUS_BLOCK_DATA is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_SMBUS_BLOCK_DATA is NOT set");
        failures++;
    }

    if (funcs & I2C_FUNC_10BIT_ADDR)
    {
        TEST_PASS("I2C_FUNC_10BIT_ADDR is set");
    }
    else
    {
        TEST_FAIL("I2C_FUNC_10BIT_ADDR is NOT set");
        failures++;
    }

    /* 5.3: I2C_FUNCS with NULL pointer (should fail) */
    printf("--- 5.3: I2C_FUNCS with NULL pointer (should fail) ---\n");
    ret = ioctl(fd, I2C_FUNCS, NULL);
    if (ret < 0)
    {
        TEST_PASS("I2C_FUNCS NULL: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("I2C_FUNCS NULL: should have been rejected");
        failures++;
    }

    close(fd);
    printf("--- Test 5: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 6: I2C_RETRIES ioctl
 */
static int test_retries(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 6: I2C_RETRIES ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 6.1: Set retries to 3 */
    printf("--- 6.1: Set I2C_RETRIES to 3 ---\n");
    ret = ioctl(fd, I2C_RETRIES, 3UL);
    failures += check_result(ret, 0, "I2C_RETRIES 3");

    /* 6.2: Set retries to 0 */
    printf("--- 6.2: Set I2C_RETRIES to 0 ---\n");
    ret = ioctl(fd, I2C_RETRIES, 0UL);
    failures += check_result(ret, 0, "I2C_RETRIES 0");

    /* 6.3: Set retries to large value */
    printf("--- 6.3: Set I2C_RETRIES to 100 ---\n");
    ret = ioctl(fd, I2C_RETRIES, 100UL);
    failures += check_result(ret, 0, "I2C_RETRIES 100");

    close(fd);
    printf("--- Test 6: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 7: I2C_TIMEOUT ioctl
 */
static int test_timeout(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 7: I2C_TIMEOUT ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 7.1: Set timeout */
    printf("--- 7.1: Set I2C_TIMEOUT to 100 ---\n");
    ret = ioctl(fd, I2C_TIMEOUT, TEST_TIMEOUT);
    failures += check_result(ret, 0, "I2C_TIMEOUT 100");

    /* 7.2: Set timeout to 0 */
    printf("--- 7.2: Set I2C_TIMEOUT to 0 ---\n");
    ret = ioctl(fd, I2C_TIMEOUT, 0UL);
    failures += check_result(ret, 0, "I2C_TIMEOUT 0");

    /* 7.3: Set timeout to large value */
    printf("--- 7.3: Set I2C_TIMEOUT to 1000 ---\n");
    ret = ioctl(fd, I2C_TIMEOUT, 1000UL);
    failures += check_result(ret, 0, "I2C_TIMEOUT 1000");

    close(fd);
    printf("--- Test 7: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 8: Read/Write operations
 */
static int test_read_write(void)
{
    int fd;
    int failures = 0;
    uint8_t wr_buf[8] = {0x00, 0x00, 'H', 'e', 'l', 'l', 'o', '!'};
    uint8_t rd_buf[16] = {0};
    ssize_t rw_ret;

    printf("\n========== Test 8: Read/Write ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* Set slave address */
    ioctl(fd, I2C_SLAVE, TEST_ADDR);

    /* 8.1: Write to I2C device */
    printf("--- 8.1: Write to I2C device ---\n");
    rw_ret = write(fd, wr_buf, sizeof(wr_buf));
    if (rw_ret == sizeof(wr_buf))
    {
        TEST_PASS("write: wrote %zd bytes", rw_ret);
    }
    else
    {
        TEST_FAIL("write: returned %zd (expected %zu)", rw_ret, sizeof(wr_buf));
        failures++;
    }

    /* 8.2: Read from I2C device */
    printf("--- 8.2: Read from I2C device ---\n");
    rw_ret = read(fd, rd_buf, sizeof(rd_buf));
    if (rw_ret >= 0)
    {
        TEST_PASS("read: got %zd bytes", rw_ret);
    }
    else
    {
        TEST_FAIL("read: returned %zd (expected >= 0)", rw_ret);
        failures++;
    }

    /* 8.3: Write zero bytes (should succeed) */
    printf("--- 8.3: Write zero bytes ---\n");
    rw_ret = write(fd, wr_buf, 0);
    if (rw_ret == 0)
    {
        TEST_PASS("write zero bytes: returned 0");
    }
    else
    {
        TEST_FAIL("write zero bytes: returned %zd (expected 0)", rw_ret);
        failures++;
    }

    /* 8.4: Read zero bytes (should succeed) */
    printf("--- 8.4: Read zero bytes ---\n");
    rw_ret = read(fd, rd_buf, 0);
    if (rw_ret == 0)
    {
        TEST_PASS("read zero bytes: returned 0");
    }
    else
    {
        TEST_FAIL("read zero bytes: returned %zd (expected 0)", rw_ret);
        failures++;
    }

    /* 8.5: Write with NULL buffer (should fail) */
    printf("--- 8.5: Write with NULL buffer (should fail) ---\n");
    rw_ret = write(fd, NULL, 10);
    if (rw_ret < 0)
    {
        TEST_PASS("write NULL buffer: failed as expected (ret=%zd)", rw_ret);
    }
    else
    {
        TEST_FAIL("write NULL buffer: should have failed");
        failures++;
    }

    /* 8.6: Read with NULL buffer (should fail) */
    printf("--- 8.6: Read with NULL buffer (should fail) ---\n");
    rw_ret = read(fd, NULL, 10);
    if (rw_ret < 0)
    {
        TEST_PASS("read NULL buffer: failed as expected (ret=%zd)", rw_ret);
    }
    else
    {
        TEST_FAIL("read NULL buffer: should have failed");
        failures++;
    }

    close(fd);
    printf("--- Test 8: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 9: lseek operation (should fail)
 */
static int test_lseek(void)
{
    int fd;
    off_t ret;
    int failures = 0;

    printf("\n========== Test 9: lseek (should fail) ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 9.1: lseek SEEK_SET */
    printf("--- 9.1: lseek SEEK_SET ---\n");
    ret = lseek(fd, 0, SEEK_SET);
    if (ret < 0)
    {
        TEST_PASS("lseek SEEK_SET: rejected as expected (ret=%ld)", (long)ret);
    }
    else
    {
        TEST_FAIL("lseek SEEK_SET: should have been rejected");
        failures++;
    }

    /* 9.2: lseek SEEK_CUR */
    printf("--- 9.2: lseek SEEK_CUR ---\n");
    ret = lseek(fd, 10, SEEK_CUR);
    if (ret < 0)
    {
        TEST_PASS("lseek SEEK_CUR: rejected as expected (ret=%ld)", (long)ret);
    }
    else
    {
        TEST_FAIL("lseek SEEK_CUR: should have been rejected");
        failures++;
    }

    /* 9.3: lseek SEEK_END */
    printf("--- 9.3: lseek SEEK_END ---\n");
    ret = lseek(fd, 0, SEEK_END);
    if (ret < 0)
    {
        TEST_PASS("lseek SEEK_END: rejected as expected (ret=%ld)", (long)ret);
    }
    else
    {
        TEST_FAIL("lseek SEEK_END: should have been rejected");
        failures++;
    }

    close(fd);
    printf("--- Test 9: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 10: I2C_RDWR combined read/write ioctl
 */
static int test_rdwr(void)
{
    int fd, ret;
    int failures = 0;
    uint8_t test_buf[8] = {0x00, 0x00};
    uint8_t rd_buf[16] = {0};
    struct i2c_msg_linux msgs[2];
    struct i2c_rdwr_ioctl_data rdwr;

    printf("\n========== Test 10: I2C_RDWR ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }
    ioctl(fd, I2C_SLAVE, TEST_ADDR);

    /* 10.1: Combined write-then-read */
    printf("--- 10.1: Combined write-then-read ---\n");
    msgs[0].addr = TEST_ADDR;
    msgs[0].flags = 0; /* Write */
    msgs[0].len = sizeof(test_buf);
    msgs[0].buf = test_buf;

    msgs[1].addr = TEST_ADDR;
    msgs[1].flags = I2C_M_RD; /* Read */
    msgs[1].len = 8;
    msgs[1].buf = rd_buf;

    rdwr.msgs = msgs;
    rdwr.nmsgs = 2;

    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR: failed with %d", ret);
        failures++;
    }

    /* 10.2: Single write message */
    printf("--- 10.2: Single write message ---\n");
    msgs[0].addr = TEST_ADDR;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = test_buf;
    rdwr.msgs = msgs;
    rdwr.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR single write: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR single write: failed with %d", ret);
        failures++;
    }

    /* 10.3: Single read message */
    printf("--- 10.3: Single read message ---\n");
    msgs[0].addr = TEST_ADDR;
    msgs[0].flags = I2C_M_RD;
    msgs[0].len = 8;
    msgs[0].buf = rd_buf;
    rdwr.msgs = msgs;
    rdwr.nmsgs = 1;

    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR single read: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR single read: failed with %d", ret);
        failures++;
    }

    /* 10.4: Zero messages (should succeed and return 0) */
    printf("--- 10.4: Zero messages ---\n");
    rdwr.nmsgs = 0;
    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret == 0)
    {
        TEST_PASS("I2C_RDWR zero messages: returned 0");
    }
    else
    {
        TEST_FAIL("I2C_RDWR zero messages: returned %d (expected 0)", ret);
        failures++;
    }

    /* 10.5: NULL rdwr_arg (should fail) */
    printf("--- 10.5: NULL rdwr_arg (should fail) ---\n");
    ret = ioctl(fd, I2C_RDWR, NULL);
    if (ret < 0)
    {
        TEST_PASS("I2C_RDWR NULL: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR NULL: should have been rejected");
        failures++;
    }

    /* 10.6: Message with 10-bit address flag */
    printf("--- 10.6: Message with I2C_M_TEN flag ---\n");
    msgs[0].addr = 0x200;
    msgs[0].flags = I2C_M_TEN;
    msgs[0].len = 2;
    msgs[0].buf = test_buf;
    rdwr.nmsgs = 1;
    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR with 10-bit addr: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR with 10-bit addr: failed with %d", ret);
        failures++;
    }

    /* 10.7: Message with I2C_M_NOSTART flag */
    printf("--- 10.7: Message with I2C_M_NOSTART flag ---\n");
    msgs[0].addr = TEST_ADDR;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = test_buf;

    msgs[1].addr = TEST_ADDR;
    msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;
    msgs[1].len = 8;
    msgs[1].buf = rd_buf;
    rdwr.nmsgs = 2;
    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR with NOSTART: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR with NOSTART: failed with %d", ret);
        failures++;
    }

    /* 10.8: Message with I2C_M_STOP flag */
    printf("--- 10.8: Message with I2C_M_STOP flag ---\n");
    msgs[0].addr = TEST_ADDR;
    msgs[0].flags = I2C_M_STOP;
    msgs[0].len = 2;
    msgs[0].buf = test_buf;
    rdwr.nmsgs = 1;
    ret = ioctl(fd, I2C_RDWR, &rdwr);
    if (ret >= 0)
    {
        TEST_PASS("I2C_RDWR with STOP: %d messages processed", ret);
    }
    else
    {
        TEST_FAIL("I2C_RDWR with STOP: failed with %d", ret);
        failures++;
    }

    close(fd);
    printf("--- Test 10: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 11: I2C_SMBUS operations
 */
static int test_smbus(void)
{
    int fd, ret;
    int failures = 0;
    struct i2c_smbus_ioctl_data smbus_args;
    union i2c_smbus_data smbus_data;

    printf("\n========== Test 11: I2C_SMBUS ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }
    ioctl(fd, I2C_SLAVE, TEST_ADDR);

    /* 11.1: SMBus Quick command */
    printf("--- 11.1: SMBus Quick (write) ---\n");
    smbus_args.read_write = I2C_SMBUS_WRITE;
    smbus_args.command = 0;
    smbus_args.size = I2C_SMBUS_QUICK;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Quick write: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Quick write: failed with %d", ret);
        failures++;
    }

    /* 11.2: SMBus Byte Read */
    printf("--- 11.2: SMBus Byte Read ---\n");
    smbus_args.read_write = I2C_SMBUS_READ;
    smbus_args.command = 0;
    smbus_args.size = I2C_SMBUS_BYTE;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Byte Read: success, data=0x%02x (ret=%d)", smbus_data.byte, ret);
    }
    else
    {
        TEST_FAIL("SMBus Byte Read: failed with %d", ret);
        failures++;
    }

    /* 11.3: SMBus Byte Write */
    printf("--- 11.3: SMBus Byte Write ---\n");
    smbus_data.byte = 0xAB;
    smbus_args.read_write = I2C_SMBUS_WRITE;
    smbus_args.command = 0;
    smbus_args.size = I2C_SMBUS_BYTE;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Byte Write: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Byte Write: failed with %d", ret);
        failures++;
    }

    /* 11.4: SMBus Byte Data Read */
    printf("--- 11.4: SMBus Byte Data Read (cmd=0x00) ---\n");
    smbus_args.read_write = I2C_SMBUS_READ;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_BYTE_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Byte Data Read: success, data=0x%02x (ret=%d)", smbus_data.byte, ret);
    }
    else
    {
        TEST_FAIL("SMBus Byte Data Read: failed with %d", ret);
        failures++;
    }

    /* 11.5: SMBus Byte Data Write */
    printf("--- 11.5: SMBus Byte Data Write (cmd=0x00) ---\n");
    smbus_data.byte = 0xCD;
    smbus_args.read_write = I2C_SMBUS_WRITE;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_BYTE_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Byte Data Write: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Byte Data Write: failed with %d", ret);
        failures++;
    }

    /* 11.6: SMBus Word Data Read */
    printf("--- 11.6: SMBus Word Data Read (cmd=0x00) ---\n");
    smbus_args.read_write = I2C_SMBUS_READ;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_WORD_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Word Data Read: success, data=0x%04x (ret=%d)", smbus_data.word, ret);
    }
    else
    {
        TEST_FAIL("SMBus Word Data Read: failed with %d", ret);
        failures++;
    }

    /* 11.7: SMBus Word Data Write */
    printf("--- 11.7: SMBus Word Data Write (cmd=0x00) ---\n");
    smbus_data.word = 0xABCD;
    smbus_args.read_write = I2C_SMBUS_WRITE;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_WORD_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Word Data Write: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Word Data Write: failed with %d", ret);
        failures++;
    }

    /* 11.8: SMBus Block Read */
    printf("--- 11.8: SMBus Block Read (cmd=0x00) ---\n");
    smbus_args.read_write = I2C_SMBUS_READ;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_BLOCK_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Block Read: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Block Read: failed with %d", ret);
        failures++;
    }

    /* 11.9: SMBus Block Write */
    printf("--- 11.9: SMBus Block Write (cmd=0x00) ---\n");
    smbus_data.block[0] = 4; /* Block length */
    smbus_data.block[1] = 'T';
    smbus_data.block[2] = 'E';
    smbus_data.block[3] = 'S';
    smbus_data.block[4] = 'T';
    smbus_args.read_write = I2C_SMBUS_WRITE;
    smbus_args.command = 0x00;
    smbus_args.size = I2C_SMBUS_BLOCK_DATA;
    smbus_args.data = &smbus_data;
    ret = ioctl(fd, I2C_SMBUS, &smbus_args);
    if (ret >= 0)
    {
        TEST_PASS("SMBus Block Write: success (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus Block Write: failed with %d", ret);
        failures++;
    }

    /* 11.10: SMBus with NULL data pointer (should fail) */
    printf("--- 11.10: SMBus with NULL data pointer (should fail) ---\n");
    ret = ioctl(fd, I2C_SMBUS, NULL);
    if (ret < 0)
    {
        TEST_PASS("SMBus NULL: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("SMBus NULL: should have been rejected");
        failures++;
    }

    close(fd);
    printf("--- Test 11: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 12: Invalid ioctl command
 */
static int test_invalid_ioctl(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 12: Invalid ioctl ==========\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("open failed");
        return 1;
    }

    /* 12.1: Call with unsupported ioctl command */
    printf("--- 12.1: Unsupported ioctl command (0x9999) ---\n");
    ret = ioctl(fd, 0x9999, 0);
    if (ret < 0)
    {
        TEST_PASS("Unsupported ioctl: rejected as expected (ret=%d)", ret);
    }
    else
    {
        TEST_FAIL("Unsupported ioctl: should have been rejected");
        failures++;
    }

    /* 12.2: Call with ioctl on a valid ioctl */
    printf("--- 12.2: ioctl on a different valid ioctl ---\n");
    ret = ioctl(fd, I2C_SLAVE, TEST_ADDR);
    failures += check_result(ret, 0, "ioctl on valid fd");

    close(fd);
    printf("--- Test 12: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 13: State reset on close/reopen
 */
static int test_state_reset(void)
{
    int fd, ret;
    int failures = 0;

    printf("\n========== Test 13: State Reset on Close/Reopen ==========\n");

    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("Failed to open device for state reset test");
        return 1;
    }

    /* Set some state */
    ret = ioctl(fd, I2C_SLAVE, TEST_ADDR);
    failures += check_result(ret, 0, "set slave addr before close");

    ret = ioctl(fd, I2C_RETRIES, 5UL);
    failures += check_result(ret, 0, "set retries before close");

    ret = ioctl(fd, I2C_TIMEOUT, 200UL);
    failures += check_result(ret, 0, "set timeout before close");

    ret = ioctl(fd, I2C_TENBIT, 1);
    failures += check_result(ret, 0, "set tenbit before close");

    ret = ioctl(fd, I2C_PEC, 1);
    failures += check_result(ret, 0, "set PEC before close");

    /* Close the device */
    printf("--- 13.1: Closing device ---\n");
    ret = close(fd);
    failures += check_result(ret, 0, "close device");

    /* Reopen - state should be reset */
    printf("--- 13.2: Reopen after close (state should be reset) ---\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_CLOEXEC);
    if (fd < 0)
    {
        TEST_FAIL("Failed to reopen device after close");
        return failures + 1;
    }

    /* I2C_SLAVE should have been reset (addr=0), so read/write without
     * setting addr should fail */
    printf("--- 13.3: Read without setting slave addr (should fail) ---\n");
    ssize_t rw_ret = read(fd, (uint8_t[8]){0}, 8);
    if (rw_ret < 0)
    {
        TEST_PASS("Read without addr: rejected as expected (ret=%zd)", rw_ret);
    }
    else
    {
        TEST_FAIL("Read without addr: should have been rejected");
        failures++;
    }

    printf("--- 13.4: Write without setting slave addr (should fail) ---\n");
    rw_ret = write(fd, (uint8_t[8]){0}, 8);
    if (rw_ret < 0)
    {
        TEST_PASS("Write without addr: rejected as expected (ret=%zd)", rw_ret);
    }
    else
    {
        TEST_FAIL("Write without addr: should have been rejected");
        failures++;
    }

    close(fd);

    printf("--- Test 13: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Test 14: O_NONBLOCK flag test
 */
static int test_nonblock(void)
{
    int fd;
    int failures = 0;

    printf("\n========== Test 14: O_NONBLOCK Open ==========\n");

    /* 14.1: Open with O_NONBLOCK */
    printf("--- 14.1: Open with O_NONBLOCK ---\n");
    fd = open(TEST_I2C_DEV, O_RDWR | O_NONBLOCK);
    if (fd >= 0)
    {
        TEST_PASS("open with O_NONBLOCK: success (fd=%d)", fd);
        close(fd);
    }
    else
    {
        TEST_FAIL("open with O_NONBLOCK: failed with %d", fd);
        failures++;
    }

    printf("--- Test 14: %d failure(s) ---\n", failures);
    return failures;
}

/*
 * Main I2C FS comprehensive test
 */
static int i2c_fs_test(void)
{
    int total_failures = 0;

    printf("\n");
    printf("############################################################\n");
    printf("#       I2C FS Comprehensive Test Suite                    #\n");
    printf("############################################################\n");
    printf("Testing device: %s\n", TEST_I2C_DEV);

    total_failures += test_open_close();

    total_failures += test_slave_address();

    total_failures += test_tenbit();

    total_failures += test_pec();

    total_failures += test_funcs();

    total_failures += test_retries();

    total_failures += test_timeout();

    total_failures += test_read_write();

    total_failures += test_lseek();

    total_failures += test_rdwr();

    total_failures += test_smbus();

    total_failures += test_invalid_ioctl();

    total_failures += test_state_reset();

    total_failures += test_nonblock();

    /* Print final summary */
    printf("\n");
    printf("############################################################\n");
    printf("#  I2C FS Test Suite Complete                             #\n");
    printf("#  Total Failures: %d                                      #\n", total_failures);
    if (total_failures == 0)
    {
        printf("#  RESULT: ALL TESTS PASSED                               #\n");
    }
    else
    {
        printf("#  RESULT: %d TEST(S) FAILED                              #\n", total_failures);
    }
    printf("############################################################\n");
    printf("Note: Some tests (read/write, SMBus, RDWR) depend on\n");
    printf("      actual I2C hardware being connected on bus 2.\n");
    printf("      Failures in those tests may be due to missing\n");
    printf("      hardware rather than driver bugs.\n");

    return total_failures > 0 ? -1 : 0;
}

SHELL_CMD_REGISTER(i2c_fs_test, NULL, "i2c_fs_test: Comprehensive I2C FS test suite", i2c_fs_test);
