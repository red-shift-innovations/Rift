# Copyright (c) 2020, Redshift Innovations
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# File:   CMakeLists.txt
# Author: Nick Otero
# Website: www.redshiftinnovations.tech

project(rift)
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_FLAGS "--std=c++20 -Wfatal-errors")

# --- Get Catch2 ---
include(ExternalProject)
ExternalProject_Add(
    catch2
    PREFIX "${CMAKE_SOURCE_DIR}/external/"
    URL https://raw.githubusercontent.com/catchorg/Catch2/master/single_include/catch2/catch.hpp
    DOWNLOAD_NO_EXTRACT 1
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
)
include_directories(${CMAKE_SOURCE_DIR}/external/src)
# --- ---

include_directories(include)

add_executable(test_bounded test/numeric/test_bounded.cpp)
add_dependencies(test_bounded catch2)
add_executable(test_probability test/numeric/test_probability.cpp)
add_dependencies(test_probability catch2)
add_executable(test_dijkstra test/algorithms/search/test_dijkstra.cpp)
add_dependencies(test_dijkstra catch2)