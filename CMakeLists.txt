cmake_minimum_required(VERSION 3.9)
project(libsurvive-extras)

include(package.cmake)

add_definitions(-DUSE_DOUBLE)

add_subdirectory(src)


