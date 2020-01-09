project(rift)
cmake_minimum_required(VERSION 3.14)
include(ExternalProject)
include_directories(include src/external)
set(CMAKE_CXX_FLAGS --std=c++2a)

#ExternalProject_Add(
#    catch2
#    PREFIX "external/"
#    URL https://raw.githubusercontent.com/catchorg/Catch2/master/single_include/catch2/catch.hpp
#    CONFIGURE_COMMAND ""
#    BUILD_COMMAND ""
#    INSTALL_COMMAND ""
#)

add_executable(test_bounded test/numeric/test_bounded.cpp)
add_executable(test_probability test/numeric/test_probability.cpp)