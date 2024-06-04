add_executable(${PROJECT_NAME} test_ringbuffer.cpp)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_17)

add_test(NAME ${PROJECT_NAME} COMMAND ${PROJECT_NAME})
