# Flashing through st-flash (stlink-org) + ST-LINK. Building firmware never touches hardware.
# st-flash verifies the written image itself; --reset starts the new firmware afterwards.
# NRST is not wired to the ST-LINK, so attach with --hot-plug (no reset on connect), as on the zagorodnyuk branch.
find_program(STFLASH_EXECUTABLE st-flash)
set(STFLASH_SERIAL "" CACHE STRING "Select one ST-LINK by serial number (st-info --probe)")
set(STFLASH_OPTIONS "" CACHE STRING "Extra st-flash options, e.g. --connect-under-reset;--freq=1M")
set(_stflash "${STFLASH_EXECUTABLE}" --hot-plug ${STFLASH_OPTIONS})
if(STFLASH_SERIAL)
  list(APPEND _stflash --serial "${STFLASH_SERIAL}")
endif()

if(EUROBOT_MECHANISM_ONLY)
  # This robot's ST-LINK has no NRST wire. Restart from the reset vector over SWD.
  set(_probe)
  if(STFLASH_SERIAL)
    list(APPEND _probe -c "adapter serial ${STFLASH_SERIAL}")
  endif()
  add_custom_target(flash
    COMMAND openocd --version
    COMMAND ${_stflash} write "${CMAKE_CURRENT_BINARY_DIR}/firmware.bin" 0x08000000
    COMMAND openocd -f interface/stlink-dap.cfg ${_probe} -f target/stm32f4x.cfg
      -c init -c "reset_config none" -c "cortex_m reset_config vectreset" -c "reset run" -c shutdown
    DEPENDS firmware USES_TERMINAL VERBATIM)
else()
  add_custom_target(flash
    COMMAND ${_stflash} --reset write "${CMAKE_CURRENT_BINARY_DIR}/firmware.bin" 0x08000000
    DEPENDS firmware USES_TERMINAL VERBATIM)
endif()
add_custom_target(reset
  COMMAND ${_stflash} reset
  USES_TERMINAL VERBATIM)
