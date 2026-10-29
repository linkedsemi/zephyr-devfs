#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/posix/fcntl.h>
#include <zephyr/posix/sys/stat.h>
#include <zephyr/posix/unistd.h>
#include <zephyr/shell/shell.h>

#define RANDOM_TEST_PATH		"/dev/urandom"
#define RANDOM_TEST_OPEN_COUNT		8
#define RANDOM_TEST_POLL_COUNT		4
#define RANDOM_TEST_WORKER_COUNT	4
#define RANDOM_TEST_WORKER_READS	256
#define RANDOM_TEST_WORKER_STACK_SIZE	2048
#define RANDOM_TEST_MAX_READ		4096
#define RANDOM_TEST_DEFAULT_BURST	1024
#define RANDOM_TEST_MAX_BURST		100000

#define TEST_PASS(fmt, ...) \
    printf("  [PASS] " fmt "\n", ##__VA_ARGS__)
#define TEST_FAIL(fmt, ...) \
    printf("  [FAIL] " fmt "\n", ##__VA_ARGS__)

struct random_worker_ctx {
    uint32_t id;
    uint32_t failures;
    size_t bytes_read;
    uint64_t hash;
};

static K_MUTEX_DEFINE(random_test_lock);
static K_SEM_DEFINE(random_worker_start, 0, RANDOM_TEST_WORKER_COUNT);
static K_THREAD_STACK_ARRAY_DEFINE(random_worker_stacks,
                   RANDOM_TEST_WORKER_COUNT,
                   RANDOM_TEST_WORKER_STACK_SIZE);
static struct k_thread random_worker_threads[RANDOM_TEST_WORKER_COUNT];
static struct random_worker_ctx random_workers[RANDOM_TEST_WORKER_COUNT];

static int check_true(bool condition, const char *name)
{
    if (condition) {
        TEST_PASS("%s", name);
        return 0;
    }

    TEST_FAIL("%s", name);
    return 1;
}

static int check_read(int fd, size_t size, uint8_t *buffer,
              const char *name)
{
    ssize_t ret = read(fd, buffer, size);

    if (ret == (ssize_t)size) {
        TEST_PASS("%s: read %zd bytes", name, ret);
        return 0;
    }

    TEST_FAIL("%s: got %zd, expected %zd, errno=%d", name, ret,
          (ssize_t)size, errno);
    return 1;
}

static bool buffer_all_equal(const uint8_t *buffer, size_t size)
{
    for (size_t i = 1; i < size; i++) {
        if (buffer[i] != buffer[0]) {
            return false;
        }
    }

    return true;
}

static bool buffers_equal(const uint8_t *left, const uint8_t *right,
              size_t size)
{
    return memcmp(left, right, size) == 0;
}

static uint64_t hash_bytes(uint64_t hash, const uint8_t *buffer, size_t size)
{
    for (size_t i = 0; i < size; i++) {
        hash ^= buffer[i];
        hash *= 1099511628211ULL;
    }

    return hash;
}

static int test_open_close_fcntl(void)
{
    uint8_t buffer[16];
    int failures = 0;
    int fd;
    int fd2;
    int flags;
    ssize_t ret;

    printf("\n========== random: open/close/fcntl ==========\n");

    fd = open(RANDOM_TEST_PATH, O_RDONLY);
    failures += check_true(fd >= 0, "open(O_RDONLY)");
    if (fd < 0) {
        return failures;
    }

    flags = fcntl(fd, F_GETFL, 0);
    failures += check_true(flags >= 0, "F_GETFL succeeds");
    failures += check_true((flags & O_ACCMODE) == O_RDONLY,
                  "F_GETFL reports read-only access");

    ret = fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    failures += check_true(ret == 0, "F_SETFL(O_NONBLOCK) succeeds");
    flags = fcntl(fd, F_GETFL, 0);
    failures += check_true((flags & O_NONBLOCK) != 0,
                  "F_GETFL reports O_NONBLOCK");

    fd2 = open(RANDOM_TEST_PATH, O_RDONLY);
    failures += check_true(fd2 >= 0, "second simultaneous open");
    if (fd2 >= 0) {
        int flags2 = fcntl(fd2, F_GETFL, 0);

        failures += check_true((flags2 & O_NONBLOCK) == 0,
                      "file status flags are per-open");
        close(fd2);
    }

    errno = 0;
    fd2 = open(RANDOM_TEST_PATH, O_WRONLY);
    failures += check_true(fd2 < 0 && errno == EACCES,
                  "open(O_WRONLY) is rejected");
    if (fd2 >= 0) {
        close(fd2);
    }

    memset(buffer, 0, sizeof(buffer));
    errno = 0;
    ret = write(fd, buffer, sizeof(buffer));
    failures += check_true(ret < 0, "write is rejected");

    memset(buffer, 0, sizeof(buffer));
    failures += check_read(fd, 1, buffer, "short read after fcntl");
    failures += check_read(fd, sizeof(buffer), buffer,
                   "normal read after fcntl");

    ret = read(fd, buffer, 0);
    failures += check_true(ret == 0, "zero-length read returns 0");

    close(fd);

    errno = 0;
    ret = read(fd, buffer, sizeof(buffer));
    failures += check_true(ret < 0 && errno == EBADF,
                  "read on closed fd returns EBADF");

    printf("--- open/close/fcntl: %d failure(s) ---\n", failures);
    return failures;
}

static int test_multiple_opens(void)
{
    uint8_t buffers[RANDOM_TEST_OPEN_COUNT][64];
    int fds[RANDOM_TEST_OPEN_COUNT];
    int failures = 0;
    bool all_equal = true;

    printf("\n========== random: multiple opens ==========\n");

    for (int i = 0; i < RANDOM_TEST_OPEN_COUNT; i++) {
        fds[i] = open(RANDOM_TEST_PATH, O_RDONLY | O_NONBLOCK);
        if (fds[i] < 0) {
            TEST_FAIL("open #%d failed, errno=%d", i, errno);
            failures++;
        }
    }

    failures += check_true(fds[0] >= 0 && fds[1] >= 0 &&
                  fds[0] != fds[1],
                  "multiple descriptors are independent");

    for (int i = 0; i < RANDOM_TEST_OPEN_COUNT; i++) {
        if (fds[i] < 0) {
            continue;
        }

        memset(buffers[i], 0, sizeof(buffers[i]));
        failures += check_read(fds[i], sizeof(buffers[i]), buffers[i],
                       "read from each open descriptor");
    }

    if (fds[0] >= 0 && fds[1] >= 0) {
        for (int i = 1; i < RANDOM_TEST_OPEN_COUNT; i++) {
            if (fds[i] >= 0 &&
                !buffers_equal(buffers[0], buffers[i],
                       sizeof(buffers[0]))) {
                all_equal = false;
                break;
            }
        }
        failures += check_true(!all_equal,
                      "different opens return different data");
    }

    for (int i = 0; i < RANDOM_TEST_OPEN_COUNT; i++) {
        if (fds[i] >= 0) {
            close(fds[i]);
        }
    }

    printf("--- multiple opens: %d failure(s) ---\n", failures);
    return failures;
}

static int test_read_sizes(void)
{
    static const size_t sizes[] = {
        1, 2, 3, 7, 15, 16, 31, 32, 63, 127, 255, 256, 511, 1024, 4096
    };
    uint8_t first[64];
    uint8_t second[64];
    uint8_t buffer[RANDOM_TEST_MAX_READ + 1];
    int failures = 0;
    int fd;

    printf("\n========== random: read sizes and buffer boundaries ==========\n");

    fd = open(RANDOM_TEST_PATH, O_RDONLY);
    if (fd < 0) {
        TEST_FAIL("open failed, errno=%d", errno);
        return 1;
    }

    for (size_t i = 0; i < ARRAY_SIZE(sizes); i++) {
        size_t offset = i % 4;

        memset(buffer, 0xa5, sizeof(buffer));
        failures += check_read(fd, sizes[i], buffer + offset,
                       "read with varying/unaligned size");
    }

    failures += check_read(fd, sizeof(first), first, "first sample");
    failures += check_read(fd, sizeof(second), second, "second sample");
    failures += check_true(!buffer_all_equal(first, sizeof(first)),
                  "sample is not a constant byte stream");
    failures += check_true(!buffers_equal(first, second, sizeof(first)),
                  "successive reads differ");

    close(fd);
    printf("--- read sizes: %d failure(s) ---\n", failures);
    return failures;
}

static int test_burst_reads(uint32_t iterations)
{
    static const size_t sizes[] = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    uint8_t buffer[256];
    uint64_t total_hash = 1469598103934665603ULL;
    uint64_t previous_hash = 0;
    uint32_t changed_blocks = 0;
    size_t total_bytes = 0;
    int failures = 0;
    int fd;
    int64_t start;
    int64_t elapsed;

    printf("\n========== random: burst reads (%u iterations) ==========\n",
           iterations);

    fd = open(RANDOM_TEST_PATH, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        TEST_FAIL("open failed, errno=%d", errno);
        return 1;
    }

    start = k_uptime_get();
    for (uint32_t i = 0; i < iterations; i++) {
        size_t size = sizes[i % ARRAY_SIZE(sizes)];
        uint64_t block_hash = 1469598103934665603ULL;
        ssize_t ret = read(fd, buffer, size);

        if (ret != (ssize_t)size) {
            if (failures < 5) {
                TEST_FAIL("burst read %u: got %zd, expected %zu, errno=%d",
                      i, ret, size, errno);
            }
            failures++;
            continue;
        }

        block_hash = hash_bytes(block_hash, buffer, size);
        total_hash = hash_bytes(total_hash, buffer, size);
        if (i == 0 || block_hash != previous_hash) {
            changed_blocks++;
        }
        previous_hash = block_hash;
        total_bytes += size;
    }
    elapsed = k_uptime_get() - start;
    close(fd);

    failures += check_true(changed_blocks > iterations / 2,
                  "successive burst samples are not frozen");
    TEST_PASS("completed %u reads, %zu bytes, %lld ms, hash=0x%016llx",
          iterations, total_bytes, (long long)elapsed,
          (unsigned long long)total_hash);

    if (elapsed > 0) {
        TEST_PASS("burst throughput: %llu bytes/s",
              (unsigned long long)(total_bytes * 1000ULL /
                           (uint64_t)elapsed));
    }

    printf("--- burst reads: %d failure(s) ---\n", failures);
    return failures;
}

static int test_rapid_open_close(void)
{
    uint8_t buffer[8];
    int failures = 0;

    printf("\n========== random: rapid open/read/close ==========\n");

    for (int i = 0; i < 128; i++) {
        int fd = open(RANDOM_TEST_PATH, O_RDONLY);

        if (fd < 0) {
            TEST_FAIL("open cycle %d failed, errno=%d", i, errno);
            failures++;
            continue;
        }

        if (read(fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
            TEST_FAIL("read cycle %d failed, errno=%d", i, errno);
            failures++;
        }

        if (close(fd) != 0) {
            TEST_FAIL("close cycle %d failed, errno=%d", i, errno);
            failures++;
        }
    }

    failures += check_true(failures == 0,
                  "128 open/read/close cycles completed");
    printf("--- rapid open/close: %d failure(s) ---\n", failures);
    return failures;
}

static int test_poll(void)
{
    struct pollfd pfds[RANDOM_TEST_POLL_COUNT];
    uint8_t buffer;
    int failures = 0;

    printf("\n========== random: poll(POLLIN) ==========\n");

    for (int i = 0; i < RANDOM_TEST_POLL_COUNT; i++) {
        pfds[i].fd = open(RANDOM_TEST_PATH, O_RDONLY | O_NONBLOCK);
        pfds[i].events = POLLIN;
        pfds[i].revents = 0;
        if (pfds[i].fd < 0) {
            TEST_FAIL("poll fd open #%d failed, errno=%d", i, errno);
            failures++;
        }
    }

    for (int cycle = 0; cycle < 32; cycle++) {
        for (int i = 0; i < RANDOM_TEST_POLL_COUNT; i++) {
            pfds[i].revents = 0;
        }

        int ret = poll(pfds, RANDOM_TEST_POLL_COUNT, 0);

        if (ret != RANDOM_TEST_POLL_COUNT) {
            TEST_FAIL("poll cycle %d returned %d, expected %d, errno=%d",
                  cycle, ret, RANDOM_TEST_POLL_COUNT, errno);
            failures++;
            continue;
        }

        for (int i = 0; i < RANDOM_TEST_POLL_COUNT; i++) {
            if (!(pfds[i].revents & POLLIN)) {
                TEST_FAIL("poll cycle %d fd #%d lacks POLLIN",
                      cycle, i);
                failures++;
            }
        }
    }
    failures += check_true(failures == 0,
                  "poll(POLLIN) is immediately ready every cycle");

    for (int i = 0; i < RANDOM_TEST_POLL_COUNT; i++) {
        if (pfds[i].fd >= 0) {
            if (read(pfds[i].fd, &buffer, 1) != 1) {
                TEST_FAIL("post-poll read fd #%d failed", i);
                failures++;
            }
            close(pfds[i].fd);
        }
    }

    printf("--- poll: %d failure(s) ---\n", failures);
    return failures;
}

static void random_worker(void *arg1, void *arg2, void *arg3)
{
    struct random_worker_ctx *ctx = arg1;
    uint8_t buffer[128];

    ARG_UNUSED(arg2);
    ARG_UNUSED(arg3);

    k_sem_take(&random_worker_start, K_FOREVER);

    int fd = open(RANDOM_TEST_PATH, O_RDONLY | O_NONBLOCK);

    if (fd < 0) {
        ctx->failures++;
        return;
    }

    ctx->hash = 1469598103934665603ULL ^ ctx->id;
    for (uint32_t i = 0; i < RANDOM_TEST_WORKER_READS; i++) {
        size_t size = 1 + ((i + 1) * (ctx->id + 3)) % sizeof(buffer);
        ssize_t ret = read(fd, buffer, size);

        if (ret != (ssize_t)size) {
            ctx->failures++;
            continue;
        }

        ctx->hash = hash_bytes(ctx->hash, buffer, size);
        ctx->bytes_read += size;
        if ((i & 7) == 0) {
            k_yield();
        }
    }

    close(fd);
}

static int test_concurrent_reads(void)
{
    int failures = 0;
    bool different_hashes = false;

    printf("\n========== random: concurrent readers ==========\n");

    memset(random_workers, 0, sizeof(random_workers));
    k_sem_reset(&random_worker_start);

    for (int i = 0; i < RANDOM_TEST_WORKER_COUNT; i++) {
        random_workers[i].id = i;
        k_thread_create(&random_worker_threads[i],
                random_worker_stacks[i],
                K_THREAD_STACK_SIZEOF(random_worker_stacks[i]),
                random_worker, &random_workers[i], NULL, NULL,
                K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
        k_thread_name_set(&random_worker_threads[i],
                  "random_test_worker");
    }

    for (int i = 0; i < RANDOM_TEST_WORKER_COUNT; i++) {
        k_sem_give(&random_worker_start);
    }

    for (int i = 0; i < RANDOM_TEST_WORKER_COUNT; i++) {
        k_thread_join(&random_worker_threads[i], K_FOREVER);
    }

    for (int i = 0; i < RANDOM_TEST_WORKER_COUNT; i++) {
        printf("  worker %d: %zu bytes, failures=%u\n",
               i, random_workers[i].bytes_read,
               random_workers[i].failures);
        if (random_workers[i].failures != 0) {
            failures++;
        }
        if (i > 0 && random_workers[i].hash != random_workers[0].hash) {
            different_hashes = true;
        }
    }

    failures += check_true(different_hashes,
                  "concurrent readers produce different streams");
    printf("--- concurrent readers: %d failure(s) ---\n", failures);
    return failures;
}

static int test_stat(void)
{
    struct stat st;
    int failures = 0;

    printf("\n========== random: stat ==========\n");

    memset(&st, 0, sizeof(st));
    failures += check_true(stat(RANDOM_TEST_PATH, &st) == 0,
                  "stat(/dev/urandom) succeeds");
    failures += check_true(st.st_size == 0,
                  "random stream reports size 0");

    printf("--- stat: %d failure(s) ---\n", failures);
    return failures;
}

static int random_fs_test(const struct shell *sh, size_t argc, char **argv)
{
    uint32_t iterations = RANDOM_TEST_DEFAULT_BURST;
    int failures = 0;

    ARG_UNUSED(sh);

    if (argc > 1) {
        iterations = (uint32_t)strtoul(argv[1], NULL, 0);
        if (iterations == 0) {
            iterations = RANDOM_TEST_DEFAULT_BURST;
        }
        if (iterations > RANDOM_TEST_MAX_BURST) {
            printf("burst count capped to %d\n",
                   RANDOM_TEST_MAX_BURST);
            iterations = RANDOM_TEST_MAX_BURST;
        }
    }

    if (k_mutex_lock(&random_test_lock, K_NO_WAIT) != 0) {
        printf("random_fs_test is already running\n");
        return -EBUSY;
    }

    printf("\n========== /dev/urandom devfs test ==========\n");

    failures += test_open_close_fcntl();
    failures += test_multiple_opens();
    failures += test_read_sizes();
    failures += test_burst_reads(iterations);
    failures += test_rapid_open_close();
    failures += test_poll();
    failures += test_concurrent_reads();
    failures += test_stat();

    printf("\n========== /dev/urandom result: %d failure(s) ==========\n",
           failures);

    k_mutex_unlock(&random_test_lock);

    return failures == 0 ? 0 : -EIO;
}

SHELL_CMD_ARG_REGISTER(random_fs_test, NULL,
               "random_fs_test [burst_reads]",
               random_fs_test, 1, 1);
