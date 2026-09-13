cmake_minimum_required(VERSION 3.20)

project(dilation_task)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(dilation
    src/main.cpp
    src/Morphology.cpp
)

target_include_directories(dilation PRIVATE
    include
)
