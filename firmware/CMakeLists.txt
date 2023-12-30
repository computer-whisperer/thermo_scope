# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH ../libs/pico-sdk)

set(PICO_BOARD pico_w CACHE STRING "Board type")
set(PICO_DEOPTIMIZED_DEBUG 1)


add_compile_definitions(PICO_STACK_SIZE=4096)
add_compile_definitions(PICO_USE_STACK_GUARDS=1)
# add_compile_definitions(SYS_CLK_KHZ=15306)
# add_compile_definitions(PLL_COMMON_REFDIV=2)
# add_compile_definitions(PLL_SYS_VCO_FREQ_KHZ=750*1000)
# add_compile_definitions(PLL_SYS_POSTDIV1=7)
# add_compile_definitions(PLL_SYS_POSTDIV2=7)
# add_compile_definitions(USB_CLK_KHZ=15306)
# add_compile_definitions(PLL_USB_VCO_FREQ_KHZ=750*1000)
# add_compile_definitions(PLL_USB_POSTDIV1=7)
# add_compile_definitions(PLL_USB_POSTDIV2=7)


# Pull in Raspberry Pi Pico SDK (must be before project)
include(libs/pico-sdk/external/pico_sdk_import.cmake)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.4.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.4.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

project(thermo_scope C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

set(UZLIB_DIR libs/uzlib)
set(CWLIB_DIR libs/cw_pico_library)

add_executable( thermo_scope
        src/main.cpp
        src/data_collection.cpp
        src/system_data_sources.cpp
        src/power_status.c
        src/display.cpp
        ${CWLIB_DIR}/drivers/tft_espi.cpp
        ${CWLIB_DIR}/drivers/st7789v2.cpp
        ${CWLIB_DIR}/drivers/gc9a01.cpp
        ${CWLIB_DIR}/drivers/bmp585.cpp
        ${CWLIB_DIR}/drivers/sht45.cpp
        ${CWLIB_DIR}/drivers/tsys01.cpp
        ${CWLIB_DIR}/drivers/i2c_bus_manager.cpp
        ${CWLIB_DIR}/drivers/ublox_ubx.cpp
        ${CWLIB_DIR}/utils/cyw43_shim.c
        ${CWLIB_DIR}/utils/gpio_isr_mux.cpp
        ${CWLIB_DIR}/utils/telemetry_manager.cpp
        ${CWLIB_DIR}/utils/time_manager.cpp
        ${CWLIB_DIR}/clients/ntp_client.cpp
        ${CWLIB_DIR}/clients/influxdb_client.cpp
        ${UZLIB_DIR}/src/adler32.c
        ${UZLIB_DIR}/src/crc32.c
        ${UZLIB_DIR}/src/defl_static.c
        ${UZLIB_DIR}/src/genlz77.c
        ${UZLIB_DIR}/src/tinfgzip.c
        ${UZLIB_DIR}/src/tinflate.c
        ${UZLIB_DIR}/src/tinfzlib.c
        )

pico_set_program_name(thermo_scope "lcdTest")
pico_set_program_version(thermo_scope "0.1")

pico_enable_stdio_uart(thermo_scope 1)
pico_enable_stdio_usb(thermo_scope 0)

# Add the standard library to the build
target_link_libraries(thermo_scope
        pico_stdlib)

# Add the standard include files to the build
target_include_directories(thermo_scope PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts or any other standard includes, if required
        )

target_include_directories(thermo_scope PRIVATE
        libs/TFT_eSPI/TFT_Drivers/
        ${UZLIB_DIR}/src/
        ${CWLIB_DIR}/clients/
        ${CWLIB_DIR}/drivers/
        ${CWLIB_DIR}/utils/
        )

# Add any user requested libraries
target_link_libraries(thermo_scope
        hardware_gpio
        hardware_spi
        hardware_dma
        hardware_pio
        hardware_adc
        hardware_i2c
        pico_cyw43_arch_lwip_poll
        pico_multicore
        hardware_pwm
        pico_sync
        lvgl
        )

set(LV_CONF_PATH ${CMAKE_CURRENT_LIST_DIR}/src/lv_conf.h)
set(LV_LVGL_H_INCLUDE_SIMPLE 1)

pico_add_extra_outputs(thermo_scope)

add_subdirectory(libs/lvgl)