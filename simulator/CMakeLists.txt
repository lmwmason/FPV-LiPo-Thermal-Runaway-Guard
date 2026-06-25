cmake_minimum_required(VERSION 3.20)
project(fpv_battery_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(BATTERY_MONITOR_DIR ${CMAKE_SOURCE_DIR}/../c)

include_directories(
    include
    ${BATTERY_MONITOR_DIR}/include
)

file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/output)

add_executable(fpv_battery_sim
    src/main.c
    src/physics.c
    src/acro.c
    src/trajectory.c
    src/battery_sim.c
    ${BATTERY_MONITOR_DIR}/src/ecm.c
    ${BATTERY_MONITOR_DIR}/src/ekf.c
    ${BATTERY_MONITOR_DIR}/src/arrhenius.c
    ${BATTERY_MONITOR_DIR}/src/controller.c
)

target_compile_options(fpv_battery_sim PRIVATE -Wall -Wextra -O2)
target_compile_definitions(fpv_battery_sim PRIVATE OUTPUT_DIR=\"${CMAKE_SOURCE_DIR}/output\")
target_link_libraries(fpv_battery_sim m)
