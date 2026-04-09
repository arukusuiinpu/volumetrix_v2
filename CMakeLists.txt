cmake_minimum_required(VERSION 4.1)
project(volumetrix)

set(CMAKE_CXX_STANDARD 20)

include_directories("./rpi-fb-matrix/rpi-rgb-led-matrix/include/")

add_executable(volumetrix
        rpi-fb-matrix/rpi-rgb-led-matrix/examples-api-use/demo-main.cc)
