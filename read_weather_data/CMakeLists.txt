# Unit tests of the weather station, built and run on the development computer: `make test`
cmake_minimum_required(VERSION 3.13)

project(read_weather_data LANGUAGES CXX)

enable_testing()

add_subdirectory(test)
