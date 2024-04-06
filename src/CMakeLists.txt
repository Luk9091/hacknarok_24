cmake_minimum_required(VERSION 3.25)

# select board:
set(PICO_BOARD pico_w)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include($ENV{PICO_SDK_PATH}/tools/CMakeLists.txt)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17) 

pico_sdk_init()

add_executable(${PROJECT_NAME}
    main.c
    I2C/i2c.c
    ToF/tof.c

    # OLED/ssd1306.c
    # COMPASS/compass.c
    # BME280/bme280.c
)

pico_add_extra_outputs(${PROJECT_NAME})

include_directories(
    I2C
    ToF
    OLED
    COMPASS
    BME280
)
add_subdirectory(VL53L0X/Api)
target_link_libraries(${PROJECT_NAME}
    pico_stdlib

    pico_cyw43_arch_none

    hardware_gpio
    hardware_i2c

    vl53l0x_api_rp2040
)


pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)
