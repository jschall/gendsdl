#
# Copyright (c) 2016-2017 UAVCAN Team
#

cmake_minimum_required(VERSION 2.8)
project(gendsdl_demo)

# Libcanard
include_directories(../build)
include_directories(../../libcanard)
include_directories(../../libcanard/drivers/socketcan)
include_directories(../common_inc)

# Compiler configuration - supporting only Clang and GCC
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Werror -m32 -lc -lm")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -std=c99   -Wall -Wextra -Werror -m32 -pedantic -lc -lm")

# Expose internal API for unit testing
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCANARD_INTERNAL=''")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -DCANARD_INTERNAL=''")

# Demo application
exec_program("git"
             ${CMAKE_CURRENT_SOURCE_DIR}
             ARGS "rev-parse --short=8 HEAD"
             OUTPUT_VARIABLE GIT_HASH)

add_executable(demo
               demo.c
               ../../libcanard/canard.c
               ../../libcanard/drivers/socketcan/socketcan.c)

target_compile_definitions(demo
                           PUBLIC GIT_HASH=0x${GIT_HASH})
