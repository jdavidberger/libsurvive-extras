cmake_minimum_required(VERSION 3.5)
project(mecbot_server)

set(CMAKE_CXX_STANDARD 14) 

add_custom_target(run_server
  COMMAND server  "${CMAKE_CURRENT_SOURCE_DIR}/web" WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  )

find_package(Poco COMPONENTS XML Foundation Util Net JSON REQUIRED)

add_library(server SurviveServer.h SurviveServer.cc
        PageRequestHandler.cc PageRequestHandler.h)

# Otherwise some versions of gcc can't find XML / JSON so
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")

FIND_LIBRARY(SBA_D_LIB sbad)

target_link_libraries(server ${SURVIVE_LIBRARY}  Poco::Foundation Poco::Util Poco::Net Poco::JSON Poco::XML
        Poco::Util ${OpenCV_LIBS} solvers)

add_executable(server_main server_main.cc)
target_link_libraries(server_main server)