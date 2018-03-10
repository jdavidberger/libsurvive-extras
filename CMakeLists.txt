cmake_minimum_required(VERSION 3.8)
project(libsurvive-extras)

include(package.cmake)

add_definitions(-DUSE_DOUBLE)
include_directories("${CGET_survive_INSTALL_DIR}/include/libsurvive")
add_subdirectory(src)


