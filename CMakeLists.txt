cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(stan_base C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

if (PICO_SDK_VERSION_STRING VERSION_LESS "1.3.0")
    message(FATAL_ERROR "Raspberry Pi Pico SDK version 1.3.0 (or later) required. Your version is ${PICO_SDK_VERSION_STRING}")
endif()

# Initialize the SDK
pico_sdk_init()

include_directories(
  lib/bno055/
  lib/cam/
  lib/uip/
  lib/uti/
)

add_executable(stan_base
  src/main.c
  lib/bno055/bno055.c
  lib/cam/cam.c
  lib/uip/uip.c
)

# pull in common dependencies
target_link_libraries(stan_base 
  pico_stdlib 
  hardware_adc
  hardware_i2c
  hardware_pwm
)

# enable usb output, disable uart output
pico_enable_stdio_usb(stan_base 1)
pico_enable_stdio_uart(stan_base 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(stan_base)

add_compile_options(
  -Wall
  -Wno-format          # int != int32_t as far as the compiler is concerned because gcc has int32_t as long int
  -Wno-unused-function # we have some for the docs that aren't called
  -Wno-maybe-uninitialized
)

# Add all tests
add_subdirectory(test/test_bno055)
add_subdirectory(test/test_cam)
add_subdirectory(test/test_uip)
