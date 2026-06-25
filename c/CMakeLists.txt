cmake_minimum_required(VERSION 3.20)
project(FPV_LiPo_Thermal_Runaway_Guard C)

set(CMAKE_C_STANDARD 11)

include_directories(include)

add_executable(${PROJECT_NAME}
        main.c
        src/ecm.c
        src/ekf.c
        src/arrhenius.c
        src/controller.c
)

target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -O2)
target_link_libraries(${PROJECT_NAME} m)

add_executable(compare_a_ecm_order
        compare/compare_a_ecm_order.c
        src/ecm.c
        src/ecm2.c
)
add_executable(compare_b_soc_method
        compare/compare_b_soc_method.c
        src/ecm.c
        src/ekf.c
)
add_executable(compare_c_soh_method
        compare/compare_c_soh_method.c
        src/ecm.c
        src/ekf.c
)
add_executable(compare_d_runaway_alert
        compare/compare_d_runaway_alert.c
        src/ecm.c
        src/ekf.c
        src/arrhenius.c
)
add_executable(compare_e_power_control
        compare/compare_e_power_control.c
        src/ecm.c
        src/ekf.c
        src/arrhenius.c
)

foreach(target compare_a_ecm_order compare_b_soc_method compare_c_soh_method compare_d_runaway_alert compare_e_power_control)
    target_compile_options(${target} PRIVATE -Wall -Wextra -O2)
    target_link_libraries(${target} m)
endforeach()

file(COPY data DESTINATION ${CMAKE_BINARY_DIR})