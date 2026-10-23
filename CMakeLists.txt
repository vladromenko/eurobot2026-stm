cmake_minimum_required(VERSION 3.24)
project(Eurobot2026 LANGUAGES C CXX ASM)
if(NOT CMAKE_C_COMPILER MATCHES "arm-none-eabi-gcc")
  message(FATAL_ERROR "Use --preset debug or -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake")
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
include(cmake/sources.cmake)
add_executable(firmware ${EUROBOT_SOURCES} cmake/gcc_runtime.c)
set_target_properties(firmware PROPERTIES SUFFIX ".elf" C_STANDARD 99 C_STANDARD_REQUIRED YES C_EXTENSIONS NO
  CXX_STANDARD 20 CXX_STANDARD_REQUIRED YES CXX_EXTENSIONS NO)
target_include_directories(firmware PRIVATE ${EUROBOT_INCLUDES})
target_compile_definitions(firmware PRIVATE STM32F407xx
  USE_FULL_LL_DRIVER ARM_MATH_CM4 __FPU_PRESENT=1U)
set(_cpu -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(firmware PRIVATE ${_cpu}
  "$<$<COMPILE_LANGUAGE:C,CXX>:-ffunction-sections;-fdata-sections;-fshort-enums;-fshort-wchar;-funsigned-char;-fno-strict-aliasing>"
  "$<$<AND:$<COMPILE_LANGUAGE:C,CXX>,$<CONFIG:Debug>>:-O0;-g3>"
  "$<$<AND:$<COMPILE_LANGUAGE:C,CXX>,$<CONFIG:Release>>:-Os;-g>"
  "$<$<COMPILE_LANGUAGE:C,CXX>:-Wall;-Wextra>"
  # No exceptions, RTTI or guarded statics: keeps the C++ build free of the C++ runtime.
  "$<$<COMPILE_LANGUAGE:CXX>:-fno-exceptions;-fno-rtti;-fno-threadsafe-statics;-fno-use-cxa-atexit>"
  # GCC 15 enables libstdc++ bounds assertions at -O0; their failure path pulls in
  # fprintf/abort/malloc (~6 KB) that have nowhere to print on this board.
  "$<$<COMPILE_LANGUAGE:CXX>:-D_GLIBCXX_NO_ASSERTIONS>"
  "$<$<COMPILE_LANGUAGE:ASM>:-g3>")
# Existing Dynamixel/WS2812B code accesses pointer storage through uint32_t*.
# Disable type-based alias optimization to preserve those accesses in Release.
# Bit-banged timing in this driver relies on unoptimized code.
set_source_files_properties(Core/BSP/l6474_drvr.cpp PROPERTIES COMPILE_OPTIONS "-O0")
set(_ld "${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F407VG_FLASH.ld")
set_property(TARGET firmware APPEND PROPERTY LINK_DEPENDS "${_ld}")
target_link_options(firmware PRIVATE ${_cpu} -nostartfiles --specs=nano.specs
  --specs=nosys.specs "-T${_ld}" "-Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/firmware.map"
  -Wl,--gc-sections -Wl,--print-memory-usage)
target_link_libraries(firmware PRIVATE m)
add_custom_command(TARGET firmware POST_BUILD
  BYPRODUCTS "${CMAKE_CURRENT_BINARY_DIR}/firmware.map" "${CMAKE_CURRENT_BINARY_DIR}/firmware.hex" "${CMAKE_CURRENT_BINARY_DIR}/firmware.bin"
  COMMAND "${CMAKE_OBJCOPY}" -O ihex --gap-fill=0xFF "$<TARGET_FILE:firmware>" "${CMAKE_CURRENT_BINARY_DIR}/firmware.hex"
  COMMAND "${CMAKE_OBJCOPY}" -O binary --gap-fill=0xFF "$<TARGET_FILE:firmware>" "${CMAKE_CURRENT_BINARY_DIR}/firmware.bin"
  COMMAND "${CMAKE_SIZE}" "$<TARGET_FILE:firmware>"
  VERBATIM)
include(cmake/openocd.cmake)

find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_custom_target(check
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake/check_firmware.py"
      --build "${CMAKE_CURRENT_BINARY_DIR}"
    DEPENDS firmware USES_TERMINAL VERBATIM)
endif()
