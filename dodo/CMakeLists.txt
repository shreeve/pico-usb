cmake_minimum_required(VERSION 3.20)

# disable TinyUSB: Comment out in ${PICO_SDK_PATH}/src/rp2_common/CMakeLists.txt
#
#   # pico_add_subdirectory(tinyusb)
#   # pico_add_subdirectory(pico_stdio_usb)

# pull in standard Pico SDK settings
include("$ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake")

# set project name to the name of this folder
cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME project_name)

project(${project_name})

pico_sdk_init()

add_executable(${project_name}
  dodo.c
)

target_link_libraries(${project_name} PRIVATE
  pico_stdlib
  hardware_resets
  hardware_irq
)

# uncomment to create uf2 files
pico_add_extra_outputs(${project_name})
