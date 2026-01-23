cmake_minimum_required(VERSION 3.20.0)

list(APPEND BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(EigenBLEC)

FILE(GLOB app_sources src/*.c src/inc/*.h drivers/*.c drivers/*.h)
target_sources(app PRIVATE ${app_sources} src/main.c)

# ---- Hard guard (file-based): strip UICR after zephyr.hex exists ----
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(ZEPHYR_HEX      ${PROJECT_BINARY_DIR}/zephyr/zephyr.hex)
set(ZEPHYR_HEX_NOUICR ${PROJECT_BINARY_DIR}/zephyr/zephyr.hex.nouicr)
set(STRIP_SCRIPT    ${CMAKE_CURRENT_LIST_DIR}/scripts/strip_uicr_hex.py)

add_custom_command(
  OUTPUT ${ZEPHYR_HEX_NOUICR}
  COMMAND ${CMAKE_COMMAND} -E echo "Stripping UICR from ${ZEPHYR_HEX}"
  COMMAND ${Python3_EXECUTABLE} ${STRIP_SCRIPT} ${ZEPHYR_HEX} ${ZEPHYR_HEX_NOUICR}
  COMMAND ${CMAKE_COMMAND} -E copy ${ZEPHYR_HEX_NOUICR} ${ZEPHYR_HEX}
  DEPENDS ${ZEPHYR_HEX} ${STRIP_SCRIPT}
  VERBATIM
)

add_custom_target(strip_uicr ALL DEPENDS ${ZEPHYR_HEX_NOUICR})

# Ensure west flash (ninja flash) runs strip_uicr first
if(TARGET flash)
  add_dependencies(flash strip_uicr)
endif()

# Some Zephyr versions use runners/flash targets with different names
if(TARGET runners)
  add_dependencies(runners strip_uicr)
endif()
