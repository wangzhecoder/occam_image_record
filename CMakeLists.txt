cmake_minimum_required(VERSION 2.8.3)
project(test)

add_compile_options(--std=c++11)

find_package(OpenCV REQUIRED)
include_directories(
    ../include
)

link_directories(
    ../lib
)

add_executable(record_raw record_raw.cc)
target_link_libraries(record_raw ${OpenCV_LIBS} /home/wz/occam_ws/src/indigosdk-2.0.21/examples/lib/libindigo.so)
