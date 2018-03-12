cmake_minimum_required(VERSION 3.5)
project(libsurvive-extras)

include(package.cmake)

SET(CMAKE_CXX_STANDARD 11)
list (APPEND CMAKE_CXX_FLAGS "-fsanitize=address -fsanitize=undefined")
add_definitions(-DUSE_DOUBLE)
include_directories("${CGET_survive_INSTALL_DIR}/include/libsurvive")
add_subdirectory(src)


