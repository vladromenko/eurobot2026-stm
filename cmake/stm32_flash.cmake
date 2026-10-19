# Merely building firmware never connects to hardware.
set(_programmer_hints)
if(CMAKE_HOST_APPLE)
  file(GLOB _cube_programmers LIST_DIRECTORIES true
    "/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.macosaarch64_*/tools/bin")
  list(SORT _cube_programmers COMPARE NATURAL ORDER DESCENDING)
  list(APPEND _programmer_hints ${_cube_programmers}
    "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOs/bin")
endif()
find_program(STM32_PROGRAMMER_CLI STM32_Programmer_CLI HINTS ${_programmer_hints})
set(STM32_STLINK_SERIAL "" CACHE STRING "Select one ST-LINK by serial number")
# NORMAL uses a Cortex software reset and works without an NRST wire.
# UR requires the physical NRST connection; never switch modes/retry writes automatically.
set(STM32_FLASH_CONNECT_MODE "NORMAL" CACHE STRING "Flash connection: NORMAL (software reset) or UR (NRST required)")
set_property(CACHE STM32_FLASH_CONNECT_MODE PROPERTY STRINGS NORMAL UR)
if(NOT STM32_FLASH_CONNECT_MODE STREQUAL "NORMAL" AND NOT STM32_FLASH_CONNECT_MODE STREQUAL "UR")
  message(FATAL_ERROR "STM32_FLASH_CONNECT_MODE must be NORMAL or UR")
endif()

foreach(_action probe backup flash verify reset)
  if(_action STREQUAL "flash" OR _action STREQUAL "verify")
    set(_deps firmware)
  else()
    set(_deps)
  endif()
  add_custom_target(${_action}
    COMMAND "${CMAKE_COMMAND}" "-DPROGRAMMER=${STM32_PROGRAMMER_CLI}"
      "-DACTION=${_action}" "-DFLASH_CONNECT_MODE=${STM32_FLASH_CONNECT_MODE}" "-DIMAGE=$<TARGET_FILE_DIR:firmware>/firmware.hex"
      "-DBINARY=$<TARGET_FILE_DIR:firmware>/firmware.bin"
      "-DSERIAL=${STM32_STLINK_SERIAL}" "-DOUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/programmer.cmake"
    DEPENDS ${_deps} USES_TERMINAL VERBATIM)
endforeach()
