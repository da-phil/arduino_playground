# Every test_*.cpp file becomes its own test executable and CTest test.
# Each file brings its own main() via DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN.
file(GLOB TEST_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/test_*.cpp)

foreach(test_source ${TEST_SOURCES})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(${test_name} ${test_source})
    target_compile_features(${test_name} PRIVATE cxx_std_17)
    # Fake Arduino headers so sketch code compiles on the host
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/mocks)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
