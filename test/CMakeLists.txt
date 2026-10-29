cmake_minimum_required(VERSION 3.16)
project(BoozeCompassHostTests C)

enable_testing()
add_executable(test_navigation
    test_navigation.c
    ../components/navigation/navigation.c
    ../components/motor/motor_math.c)
target_include_directories(test_navigation PRIVATE
    ../components/navigation/include
    ../components/motor/include)
if(NOT MSVC)
    target_link_libraries(test_navigation PRIVATE m)
endif()
add_test(NAME navigation_and_motor_math COMMAND test_navigation)
