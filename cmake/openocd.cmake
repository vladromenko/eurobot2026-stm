# Flashing through OpenOCD + ST-LINK. Building firmware never touches hardware.
# The target is reset by SYSRESETREQ, so no NRST wire is required.
find_program(OPENOCD_EXECUTABLE openocd)
set(OPENOCD_ADAPTER_SERIAL "" CACHE STRING "Select one ST-LINK by serial number")
set(_openocd "${OPENOCD_EXECUTABLE}" -f interface/stlink.cfg)
if(OPENOCD_ADAPTER_SERIAL)
  list(APPEND _openocd -c "adapter serial ${OPENOCD_ADAPTER_SERIAL}")
endif()
list(APPEND _openocd -f target/stm32f4x.cfg)

add_custom_target(flash
  COMMAND ${_openocd} -c "program $<TARGET_FILE:firmware> verify reset exit"
  DEPENDS firmware USES_TERMINAL VERBATIM)
add_custom_target(reset
  COMMAND ${_openocd} -c "init" -c "reset run" -c "exit"
  USES_TERMINAL VERBATIM)
