if(CONFIG_ZEPHYR_DEVFS_DRIVER)
    zephyr_library_named(zephyr-devfs)

if(CONFIG_SOC_LSQSH_CPU1)
    zephyr_library_sources(
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/gpio/gpio_fs.c
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/gpio/gpio_testcase.c
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/jtag/jtagm_fs.c
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/jtag/jtagm_testcase.c
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/uart/uart_fs.c
        ${ZEPHYR_CURRENT_MODULE_DIR}/src/uart/uart_testcase.c
    )

    zephyr_library_include_directories(${ZEPHYR_CURRENT_MODULE_DIR}/)
    target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PUBLIC ${ZEPHYR_CURRENT_MODULE_DIR}/inc)

endif()

endif()
