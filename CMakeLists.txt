cmake_minimum_required(VERSION 3.8)
project(libsurvive-extras)

include(package.cmake)

list (APPEND CMAKE_CXX_FLAGS "-fsanitize=address -fsanitize=undefined")
add_definitions(-DUSE_DOUBLE)
include_directories("${CGET_survive_INSTALL_DIR}/include/libsurvive")
add_subdirectory(src)


