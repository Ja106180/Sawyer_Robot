cmake_minimum_required(VERSION 3.0.2)
project(lesson_03_1d_kalman)

# C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 可执行文件
add_executable(simple_kalman simple_kalman.cpp)

# 编译选项
target_compile_options(simple_kalman PRIVATE -Wall -Wextra)

