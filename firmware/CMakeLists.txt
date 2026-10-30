cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(pa_detector C CXX ASM)

pico_sdk_init()

add_executable(pa_detector
    src/main.c
    src/protocol.c
    src/i2s_rx.c
    src/modulator.c
    src/tlv320adc6120.c
    src/lockin.c
    src/usb_descriptors.c
    src/usb_io.c
    src/selftest.c
    src/safety.c
)

pico_set_program_name(pa_detector "pa-detector")
pico_set_program_version(pa_detector "1.0.0")

pico_generate_pio_header(pa_detector ${CMAKE_CURRENT_LIST_DIR}/src/i2s_rx.pio)
pico_generate_pio_header(pa_detector ${CMAKE_CURRENT_LIST_DIR}/src/modulator.pio)
pico_generate_pio_header(pa_detector ${CMAKE_CURRENT_LIST_DIR}/src/framecount.pio)

target_include_directories(pa_detector PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)
target_compile_options(pa_detector PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_compile_definitions(pa_detector PRIVATE
    PICO_PANIC_FUNCTION=pa_panic        # panic -> GP0 low + reboot (safety.c)
    # 76.8 MHz system clock from boot: 25 PIO cycles per I2S bit clock -> 3.072 MHz,
    # fs = 48000.000 Hz (VCO 12 MHz x 128 = 1536 MHz, post-dividers 5 x 4)
    SYS_CLK_HZ=76800000
    PLL_SYS_VCO_FREQ_HZ=1536000000
    PLL_SYS_POSTDIV1=5
    PLL_SYS_POSTDIV2=4
)

# stdio must never touch GP0 (UART0 TX default) nor the USB CDC we drive ourselves
pico_enable_stdio_uart(pa_detector 0)
pico_enable_stdio_usb(pa_detector 0)

target_link_libraries(pa_detector
    pico_stdlib
    pico_unique_id
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_clocks
    hardware_watchdog
    hardware_irq
    tinyusb_device
    tinyusb_board
)

pico_add_extra_outputs(pa_detector)
